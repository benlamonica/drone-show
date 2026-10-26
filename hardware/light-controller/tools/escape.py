"""Fan out the MCU pins the routers could not get out of the 0.4 mm pitch QFN.

For each open net, every RP2350 pad of that net gets a short stub straight out from
the package (then bent 45 degrees if straight is blocked), long enough to clear the
row of neighbouring pads. When `--via` is given the stub ends in a via, so the
grid router can carry on on either layer. Every candidate is checked against all
other copper with the 0.15 mm rule before it is kept.

    python3 tools/escape.py [--via] /NET ...
"""
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from gen_pcb import PCB  # noqa: E402
from finish_route import unconnected_nets  # noqa: E402

mm = pcbnew.FromMM
CLR = mm(0.152)
W = mm(0.15)
VIA_D, VIA_DRILL = mm(0.6), mm(0.3)


def collides(board, net, shapes):
    """shapes: list of (layer or None for both, SHAPE). True if any hits other-net copper."""
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == net:
                continue
            for ly, sh in shapes:
                for L in ((ly,) if ly is not None else (pcbnew.F_Cu, pcbnew.B_Cu)):
                    if p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(sh, CLR):
                        return True
    for t in board.GetTracks():
        if t.GetNetname() == net:
            continue
        for ly, sh in shapes:
            for L in ((ly,) if ly is not None else (pcbnew.F_Cu, pcbnew.B_Cu)):
                if t.IsOnLayer(L) and t.GetEffectiveShape(L).Collide(sh, CLR):
                    return True
    return False


def holes_ok(board, pos):
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA" and (t.GetPosition() - pos).EuclideanNorm() < mm(0.8 + 0.3):
            return False
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.HasHole() and (p.GetPosition() - pos).EuclideanNorm() < mm(1.5):
                return False
    return True


def escape_pad(board, pad, centre, use_via):
    net = pad.GetNetname()
    c = pad.GetPosition()
    dx, dy = c.x - centre.x, c.y - centre.y
    # outward axis: the larger component (pads sit on the four sides)
    ux, uy = (1 if dx > 0 else -1, 0) if abs(dx) > abs(dy) else (0, 1 if dy > 0 else -1)
    for first in (0.7, 0.9, 1.1):
        tip = pcbnew.VECTOR2I(c.x + int(mm(first) * ux), c.y + int(mm(first) * uy))
        for bend in (0, 1, -1):
            for blen in ((0,) if bend == 0 else (0.5, 0.8, 1.2)):
                # 45 degree bend sideways after the straight part
                if ux != 0:
                    sx, sy = ux, bend
                else:
                    sx, sy = bend, uy
                end = pcbnew.VECTOR2I(tip.x + int(mm(blen) * sx), tip.y + int(mm(blen) * sy))
                segs = [(c, tip)] + ([(tip, end)] if blen else [])
                shapes = [(pcbnew.F_Cu, pcbnew.SHAPE_SEGMENT(a, b, W)) for a, b in segs]
                if use_via:
                    shapes.append((None, pcbnew.SHAPE_CIRCLE(end, VIA_D // 2)))
                    if not holes_ok(board, end):
                        continue
                if collides(board, net, shapes):
                    continue
                for a, b in segs:
                    t = pcbnew.PCB_TRACK(board)
                    t.SetStart(a)
                    t.SetEnd(b)
                    t.SetWidth(W)
                    t.SetLayer(pcbnew.F_Cu)
                    t.SetNet(pad.GetNet())
                    board.Add(t)
                if use_via:
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(end)
                    v.SetWidth(VIA_D)
                    v.SetDrill(VIA_DRILL)
                    v.SetNet(pad.GetNet())
                    board.Add(v)
                return True
    return False


def has_track(board, pad):
    for t in board.GetTracks():
        if t.GetNetCode() == pad.GetNetCode() and t.IsOnLayer(pcbnew.F_Cu):
            if pad.GetEffectiveShape(pcbnew.F_Cu).Collide(t.GetEffectiveShape(pcbnew.F_Cu), 0):
                return True
    return False


def main():
    board = pcbnew.LoadBoard(PCB)
    use_via = "--via" in sys.argv
    nets = [a for a in sys.argv[1:] if a.startswith("/")] or unconnected_nets(board)
    u2 = board.FindFootprintByReference("U2")
    centre = u2.GetPosition()
    for p in u2.Pads():
        if p.GetNetname() in nets and p.GetNumber() != "61" and not has_track(board, p):
            ok = escape_pad(board, p, centre, use_via)
            print(f"U2.{p.GetNumber()} {p.GetNetname()}: {'stub' if ok else 'blocked'}")
    pcbnew.SaveBoard(PCB, board)


if __name__ == "__main__":
    main()


def inner_via(board, pad, centre, offsets=(0.0, 0.2, -0.2, 0.4, -0.4)):
    """Drop a via in the channel between the pad ring and the exposed pad, joined to
    the pin by a short stub pointing inward. Frees the outside of the pin for others."""
    c = pad.GetPosition()
    dx, dy = c.x - centre.x, c.y - centre.y
    ux, uy = (1 if dx > 0 else -1, 0) if abs(dx) > abs(dy) else (0, 1 if dy > 0 else -1)
    r = mm(2.42)
    for off in offsets:
        if ux:
            end = pcbnew.VECTOR2I(centre.x + ux * r, c.y + mm(off))
        else:
            end = pcbnew.VECTOR2I(c.x + mm(off), centre.y + uy * r)
        bend = pcbnew.VECTOR2I(end.x, c.y) if ux else pcbnew.VECTOR2I(c.x, end.y)
        segs = [(c, bend), (bend, end)] if off else [(c, end)]
        shapes = [(pcbnew.F_Cu, pcbnew.SHAPE_SEGMENT(a, b, W)) for a, b in segs]
        shapes.append((None, pcbnew.SHAPE_CIRCLE(end, VIA_D // 2)))
        if not holes_ok(board, end) or collides(board, pad.GetNetname(), shapes):
            continue
        for a, b in segs:
            if a == b:
                continue
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(a)
            t.SetEnd(b)
            t.SetWidth(W)
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(pad.GetNet())
            board.Add(t)
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(end)
        v.SetWidth(VIA_D)
        v.SetDrill(VIA_DRILL)
        v.SetNet(pad.GetNet())
        board.Add(v)
        return True
    return False
