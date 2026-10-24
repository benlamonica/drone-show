"""Prepare the placed board for Freerouting. Run with KiCad's Python.

    python3 tools/route_prep.py signals build/placed.kicad_pcb build/fanned.kicad_pcb build/route.dsn
    python3 tools/route_prep.py phase build/fanned.kicad_pcb build/route.ses build/route_phase.dsn

GND and +BATT are not routed as tracks: they are the bottom and top pours.
Every top-side GND pad and bottom-side +BATT pad gets a short stub to its own
via here, so the pour on the other layer picks it up. The board-wide pours
are left out of the DSN (Freerouting would treat them as solid obstacles).

Routing is two passes. Pass 1 routes every signal with the phase strips
turned into keepouts, so no signal track cuts the motor current path. Pass 2
keeps pass 1's tracks fixed and routes only the phase nets' small-signal pins
(gate driver, bootstrap caps, BEMF dividers) out to their strips. Track
keepouts on top also keep signals out of the +BATT path from the centre to
each channel's high-side FETs.
"""
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

MM = pcbnew.FromMM
CX, CY = 150.0, 100.0
VIA, DRILL = 0.6, 0.3
GAP = 0.3


def v(x, y):
    return pcbnew.VECTOR2I(MM(CX + x), MM(CY + y))


def xy(p):
    return pcbnew.ToMM(p.x) - CX, pcbnew.ToMM(p.y) - CY


class Obstacles:
    def __init__(self, board):
        self.items = []   # (x0, y0, x1, y1, netcode, layers:set, is_via)
        for fp in board.GetFootprints():
            for p in fp.Pads():
                b = p.GetBoundingBox()
                x0, y0 = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
                x1, y1 = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
                layers = set()
                if p.IsOnLayer(pcbnew.F_Cu):
                    layers.add("F")
                if p.IsOnLayer(pcbnew.B_Cu):
                    layers.add("B")
                if p.GetDrillSize().x > 0:
                    layers = {"F", "B"}
                self.items.append((x0, y0, x1, y1, p.GetNetCode(), layers, False))
        for t in board.GetTracks():
            if t.GetClass() == "PCB_VIA":
                x, y = xy(t.GetPosition())
                r = pcbnew.ToMM(t.GetWidth(pcbnew.F_Cu)) / 2
                self.items.append((x - r, y - r, x + r, y + r, t.GetNetCode(), {"F", "B"}, True))

    def via_ok(self, x, y, net, r=VIA / 2):
        lim = L.CHAMFER * L.S2 - 0.8 * L.S2
        if abs(x) > L.HALF - 0.8 or abs(y) > L.HALF - 0.8 or abs(x) + abs(y) > lim:
            return False
        for hx, hy in L.holes():
            if math.hypot(x - hx, y - hy) < L.HOLE_PAD / 2 + 0.5 + r:
                return False
        for x0, y0, x1, y1, n, _, is_via in self.items:
            # same-net pads are fine to touch; same-net vias still need the
            # fab's 0.5 mm hole-to-hole spacing
            g = GAP if n != net else (0.25 if is_via else -10)
            if x + r + g > x0 and x - r - g < x1 and y + r + g > y0 and y - r - g < y1:
                return False
        return True

    def track_ok(self, ax, ay, bx, by, net, width, layer, own):
        """Stub from a pad to its via: clear of every other net's pad on that
        layer. own is the pad's bounding box, skipped."""
        n = max(2, int(math.hypot(bx - ax, by - ay) / 0.1))
        r = width / 2
        for x0, y0, x1, y1, nn, layers, _ in self.items:
            if nn == net or layer not in layers or (x0, y0, x1, y1) == own:
                continue
            for i in range(n + 1):
                x, y = ax + (bx - ax) * i / n, ay + (by - ay) * i / n
                if x + r + GAP > x0 and x - r - GAP < x1 and y + r + GAP > y0 and y - r - GAP < y1:
                    return False
        return True

    def add_track(self, ax, ay, bx, by, net, width, layer):
        n = max(1, int(math.hypot(bx - ax, by - ay) / 0.2))
        r = width / 2
        for i in range(n + 1):
            x, y = ax + (bx - ax) * i / n, ay + (by - ay) * i / n
            self.items.append((x - r, y - r, x + r, y + r, net, {layer}, False))

    def add_via(self, x, y, net):
        r = VIA / 2
        self.items.append((x - r, y - r, x + r, y + r, net, {"F", "B"}, True))


def zone_at(board, layer, x, y):
    pt = v(x, y)
    best = None
    for z in board.Zones():
        if z.GetIsRuleArea() or not z.IsOnLayer(layer):
            continue
        if z.Outline().Contains(pt):
            if best is None or z.GetAssignedPriority() > best.GetAssignedPriority():
                best = z
    return best


def add_track(board, pad, end, bottom, width=0.4):
    t = pcbnew.PCB_TRACK(board)
    t.SetStart(pad.GetPosition())
    t.SetEnd(end)
    t.SetWidth(MM(width))
    t.SetLayer(pcbnew.B_Cu if bottom else pcbnew.F_Cu)
    t.SetNet(pad.GetNet())
    board.Add(t)


def fanout(board):
    obs = Obstacles(board)
    gnd = board.FindNet("GND")
    batt = board.FindNet("+BATT")
    done, failed = 0, []
    placed_vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    for fp in board.GetFootprints():
        bottom = fp.IsFlipped()
        for p in fp.Pads():
            if p.GetDrillSize().x > 0:
                continue
            net = p.GetNetname()
            if not ((net == "GND" and not bottom) or (net == "+BATT" and bottom)):
                continue
            px, py = xy(p.GetPosition())
            if fp.GetValue() == "AT32F421G8U7" and p.GetNumber() == "29":
                # exposed pad: one via straight down in the middle of it
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(p.GetPosition())
                via.SetWidth(MM(VIA))
                via.SetDrill(MM(DRILL))
                via.SetNet(p.GetNet())
                board.Add(via)
                obs.add_via(px, py, p.GetNetCode())
                done += 1
                continue
            if fp.GetValue() == "AT32F421G8U7":
                # MCU ground pins: short track inward to the exposed pad
                ep = next(q for q in fp.Pads() if q.GetNumber() == "29")
                b = p.GetBoundingBox()
                if b.GetWidth() > b.GetHeight():     # pin on a left/right side
                    end = pcbnew.VECTOR2I(ep.GetPosition().x, p.GetPosition().y)
                else:
                    end = pcbnew.VECTOR2I(p.GetPosition().x, ep.GetPosition().y)
                add_track(board, p, end, bottom, 0.2)
                continue
            # skip pads that already have a via of their net close by
            near = [t for t in placed_vias if t.GetNetCode() == p.GetNetCode()
                    and math.hypot(*(a - b for a, b in zip(xy(t.GetPosition()), (px, py)))) < 1.6]
            if near:
                # a via placed with the part (bulk caps): just join them
                if not p.HitTest(near[0].GetPosition()):
                    add_track(board, p, near[0].GetPosition(), bottom)
                continue
            other = pcbnew.B_Cu if not bottom else pcbnew.F_Cu
            sx = pcbnew.ToMM(p.GetBoundingBox().GetWidth()) / 2
            sy = pcbnew.ToMM(p.GetBoundingBox().GetHeight()) / 2
            fx, fy = xy(fp.GetPosition())
            away = math.atan2(py - fy, px - fx)
            angles = sorted(range(0, 360, 30), key=lambda a: min(abs(a), 360 - a))
            if max(sx, sy) > 1.4 * min(sx, sy) and min(sx, sy) < 0.5:
                # fine-pitch pin: leave along its own axis, never across a neighbour
                if sx > sy:
                    away = 0.0 if px > fx else math.pi
                else:
                    away = math.pi / 2 if py > fy else -math.pi / 2
                angles = [0, 30, -30, 60, -60]
            width = 0.4 if min(sx, sy) > 0.2 else 0.2
            b = p.GetBoundingBox()
            own = (*xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop())), *xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom())))
            layer = "B" if bottom else "F"
            placed = False
            for dist in (0.15, 0.4, 0.7, 1.0, 1.4, 1.8):
                for da in angles:
                    a = away + math.radians(da)
                    ext = abs(math.cos(a)) * sx + abs(math.sin(a)) * sy
                    d = ext + VIA / 2 + dist
                    x, y = px + d * math.cos(a), py + d * math.sin(a)
                    if not obs.via_ok(x, y, p.GetNetCode()):
                        continue
                    if not obs.track_ok(px, py, x, y, p.GetNetCode(), width, layer, own):
                        continue
                    z = zone_at(board, other, x, y)
                    if z is None or z.GetNetname() != net:
                        continue
                    via = pcbnew.PCB_VIA(board)
                    via.SetPosition(v(x, y))
                    via.SetWidth(MM(VIA))
                    via.SetDrill(MM(DRILL))
                    via.SetNet(p.GetNet())
                    board.Add(via)
                    add_track(board, p, v(x, y), bottom, width)
                    obs.add_track(px, py, x, y, p.GetNetCode(), width, layer)
                    obs.add_via(x, y, p.GetNetCode())
                    placed = True
                    done += 1
                    break
                if placed:
                    break
            if not placed:
                failed.append(f"{fp.GetReference()}.{p.GetNumber()}")
    print(f"fanout: {done} vias, failed: {failed}")
    return gnd, batt


def add_keepout(board, outline, layer, name, vias=False):
    z = pcbnew.ZONE(board)
    z.SetIsRuleArea(True)
    z.SetLayer(layer)
    z.SetDoNotAllowTracks(True)
    z.SetDoNotAllowVias(vias)
    z.SetDoNotAllowPads(False)
    z.SetDoNotAllowZoneFills(False)
    z.SetDoNotAllowFootprints(False)
    o = z.Outline()
    if isinstance(outline, list):
        o.NewOutline()
        for x, y in outline:
            o.Append(MM(CX + x), MM(CY + y))
    else:
        o.AddOutline(outline)
    z.SetZoneName(name)
    board.Add(z)


def strip_pours(board):
    """Drop the board-wide pours and the MCU GND islands: Freerouting would
    treat them as solid copper."""
    for z in list(board.Zones()):
        if z.GetIsRuleArea():
            continue
        name = z.GetZoneName()
        if name in ("BATT top", "GND bottom") or name.startswith("MCU GND"):
            board.Remove(z)


def signals(src, fanned, dsn):
    """Pass 1: every signal except the phase nets. Freerouting doesn't treat
    another net's plane as an obstacle, so the phase strips become keepouts
    here; that keeps signal tracks from cutting the motor current path."""
    board = pcbnew.LoadBoard(src)
    fanout(board)
    board.Save(fanned)
    # +BATT highways on top, centre to each channel's high-side drains
    for k in range(4):
        pts = [L.rot(x, y, k) for x, y in
               ((7.5, -9.0), (L.FET_X - 1.0, -9.0), (L.FET_X - 1.0, 9.0), (7.5, 9.0))]
        add_keepout(board, pts, pcbnew.F_Cu, f"BATT highway {k}")
    strip_pours(board)
    for z in list(board.Zones()):
        if not z.GetIsRuleArea() and "/Motor" in z.GetNetname():
            add_keepout(board, z.Outline().COutline(0), z.GetLayer(), "phase " + z.GetZoneName(), vias=True)
            board.Remove(z)
    print("dsn", dsn, pcbnew.ExportSpecctraDSN(board, dsn))


def phase(fanned, ses, dsn):
    """Pass 2: the phase nets' small-signal pins (driver, bootstrap, BEMF
    divider) out to their strips. Pass 1's tracks go in as protected wiring."""
    board = pcbnew.LoadBoard(fanned)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("SES import failed")
    strip_pours(board)
    pcbnew.ExportSpecctraDSN(board, dsn)
    t = open(dsn).read().replace("(type route)", "(type protect)")
    open(dsn, "w").write(t)
    print("dsn", dsn)


if __name__ == "__main__":
    {"signals": signals, "phase": phase}[sys.argv[1]](*sys.argv[2:])
