"""Connect GND pads that the pours left stranded.

The autorouter leaves GND to the two pours. Where a pad's pour island doesn't reach the
rest of GND, this drops a via next to the pad (with a short stub on the pad's layer) at
the closest spot that clears everything, so the pad reaches the other layer's pour.
Repeats until DRC reports no unconnected GND pad or nothing more can be placed.
"""
import math
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from gen_pcb import PCB  # noqa: E402

mm = pcbnew.FromMM
CLR = mm(0.2)          # a little over the 0.15 minimum
VIA_D, VIA_DRILL = mm(0.6), mm(0.3)


def unconnected_gnd(board):
    rpt = "/tmp/lc_stitch.rpt"
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, False)
    out = []
    for blk in re.split(r"\n(?=\[)", open(rpt).read()):
        if blk.startswith("[unconnected_items]"):
            for ref, num in re.findall(r"Pad (\S+) \[/GND\] of (\S+) on", blk):
                out.append((num, ref))
    return out


def blocked(board, pos, layer, pad):
    circ = pcbnew.SHAPE_CIRCLE(pos, VIA_D // 2)
    seg = pcbnew.SHAPE_SEGMENT(pad.GetPosition(), pos, mm(0.3))
    hx, hy = pcbnew.FromMM(73.0 / 2 - 0.5), pcbnew.FromMM(35.0 / 2 - 0.5)
    if abs(pos.x) > hx or abs(pos.y) > hy or pcbnew.ToMM(pos.x) < -29.0:
        return True
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == "/GND" and p == pad:
                continue
            for ly in (pcbnew.F_Cu, pcbnew.B_Cu):
                if not p.IsOnLayer(ly):
                    continue
                sh = p.GetEffectiveShape(ly)
                if p.GetNetname() != "/GND" and sh.Collide(circ, CLR):
                    return True
                if ly == layer and p.GetNetname() != "/GND" and sh.Collide(seg, CLR):
                    return True
            if p.HasHole() and (p.GetPosition() - pos).EuclideanNorm() < mm(1.4):
                return True
    for t in board.GetTracks():
        sh_layers = (pcbnew.F_Cu, pcbnew.B_Cu) if t.GetClass() == "PCB_VIA" else (t.GetLayer(),)
        if t.GetClass() == "PCB_VIA" and (t.GetPosition() - pos).EuclideanNorm() < mm(0.9):
            return True
        if t.GetNetname() == "/GND":
            continue
        for ly in sh_layers:
            sh = t.GetEffectiveShape(ly)
            if sh.Collide(circ, CLR) or (ly == layer and sh.Collide(seg, CLR)):
                return True
    return False


def stitch_pad(board, pad):
    layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    c = pad.GetPosition()
    for r in [0.0] + [0.6 + 0.15 * i for i in range(12)]:
        n = 1 if r == 0 else 16
        for i in range(n):
            a = 2 * math.pi * i / n
            pos = pcbnew.VECTOR2I(c.x + int(mm(r) * math.cos(a)), c.y + int(mm(r) * math.sin(a)))
            if r == 0:
                continue        # never via-in-pad
            if blocked(board, pos, layer, pad):
                continue
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pos)
            v.SetWidth(VIA_D)
            v.SetDrill(VIA_DRILL)
            v.SetNet(pad.GetNet())
            board.Add(v)
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(c)
            t.SetEnd(pos)
            t.SetWidth(mm(0.3))
            t.SetLayer(layer)
            t.SetNet(pad.GetNet())
            board.Add(t)
            return True
    return False


def main():
    board = pcbnew.LoadBoard(PCB)
    filler = pcbnew.ZONE_FILLER(board)
    done = set()
    for _ in range(6):
        filler.Fill(board.Zones())
        todo = [x for x in unconnected_gnd(board) if x not in done]
        if not todo:
            break
        for ref, num in todo:
            fp = board.FindFootprintByReference(ref)
            pad = next(p for p in fp.Pads() if p.GetNumber() == num and p.GetNetname() == "/GND")
            ok = stitch_pad(board, pad)
            print(f"{ref}.{num}: {'via added' if ok else 'NO ROOM'}")
            done.add((ref, num))
    filler.Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)


if __name__ == "__main__":
    main()
