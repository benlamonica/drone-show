"""Prepare the placed board for Freerouting. Run with KiCad's Python.

    python3 tools/route_prep.py fanout build/placed.kicad_pcb build/fanned.kicad_pcb
    python3 tools/route_prep.py resume build/prerouted.kicad_pcb build/route.dsn

GND and +BATT are not routed as tracks: they are the inner planes plus the
bottom and top pours. Every GND and +BATT pad gets a short stub to its own via
here, so the plane picks it up. The board-wide
pours are left out of the DSN (Freerouting would treat them as solid
obstacles).

The phase strips go into the DSN as protected copper, so signal tracks route
round them and the phase nets' small-signal pins (gate driver, bootstrap
caps, BEMF dividers) route to them.
"""
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

MM = pcbnew.FromMM
CX, CY = 150.0, 100.0
VIA, DRILL = 0.66, 0.3
# exposed pad number of each part that has one
# signal pins that get a via here, by part value: the MCU's BOOT0 pin goes
# straight down to its resistor on the other side (NRST and VDDA sit side by
# side at 0.4 mm, too close for two straight vias; the router staggers
# theirs, and 3V3 gets its via in pwm_lines).
SIGNAL_VIA_PINS = {"AT32F421G8U7": ("1",)}
EXPOSED_PADS = {"AT32F421G8U7": "29", "NSG2065Q": "25", "TLV76733DRVR": "7"}
GAP = 0.22
# a 0.2 mm stub leaving a 0.4 mm pitch QFN pin clears its neighbours by
# 0.21 mm, which the fab's 0.2 mm allows
STUB_GAP = 0.2


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
            else:
                ax, ay = xy(t.GetStart())
                bx, by = xy(t.GetEnd())
                self.add_track(ax, ay, bx, by, t.GetNetCode(), pcbnew.ToMM(t.GetWidth()),
                               "B" if t.GetLayer() == pcbnew.B_Cu else "F")

    def via_ok(self, x, y, net, r=VIA / 2):
        lim = L.CHAMFER * L.S2 - 0.8 * L.S2
        if abs(x) > L.HALF - 0.8 or abs(y) > L.HALF - 0.8 or abs(x) + abs(y) > lim:
            return False
        for hx, hy in L.holes():
            if math.hypot(x - hx, y - hy) < L.HOLE_COPPER_FREE / 2 + 0.2 + r:
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
                if own[0] <= x <= own[2] and own[1] <= y <= own[3]:
                    continue    # still on the pad itself, which is already there
                if x + r + STUB_GAP > x0 and x - r - STUB_GAP < x1 and y + r + STUB_GAP > y0 and y - r - STUB_GAP < y1:
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
        ep_num = EXPOSED_PADS.get(fp.GetValue())
        ep_via = False
        # exposed pad first: if it gets no via, its ground pins fan out too
        # the middle PWM pin of each group of three goes first and straight
        # out, so its neighbours fan out at an angle round its via
        for p in sorted(fp.Pads(), key=lambda q: (q.GetNumber() != ep_num, q.GetNumber() not in ("2", "23")
                                                   or fp.GetValue() != "NSG2065Q")):
            if p.GetDrillSize().x > 0:
                continue
            net = p.GetNetname()
            px, py = xy(p.GetPosition())
            if L.LAYERS == 2:
                # pads sitting in their own net's outer pour reach the other
                # layer through that pour's vias; only the ones in the other
                # net's pour need their own
                if not ((net == "GND" and not bottom) or (net == "+BATT" and bottom)):
                    continue
            else:
                # the outer pours get cut up by the routing, so every GND and
                # +BATT pad gets its own via to the inner planes, except the
                # FETs (big pads in the pours, low-side sources on their tabs)
                sig = p.GetNumber() in SIGNAL_VIA_PINS.get(fp.GetValue(), ())
                if not sig and (net not in ("GND", "+BATT") or fp.GetValue() == "SP40N01GHNK"):
                    continue
            if p.GetNumber() == ep_num:
                # exposed pad: one via in it, the middle if nothing on the
                # other side is in the way, else nudged towards an edge
                hw = max(0.0, pcbnew.ToMM(p.GetBoundingBox().GetWidth()) / 2 - 0.3)
                hh = max(0.0, pcbnew.ToMM(p.GetBoundingBox().GetHeight()) / 2 - 0.3)
                spots = [(px + dx, py + dy) for dx in (0, -hw, hw) for dy in (0, -hh, hh)]
                spot = next(((x, y) for x, y in spots if obs.via_ok(x, y, p.GetNetCode())), None)
                if spot is None:
                    failed.append(f"{fp.GetReference()}.{p.GetNumber()}")
                    continue
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(v(*spot))
                via.SetWidth(MM(VIA))
                via.SetDrill(MM(DRILL))
                via.SetNet(p.GetNet())
                board.Add(via)
                obs.add_via(*spot, p.GetNetCode())
                ep_via = True
                done += 1
                continue
            ep = next((q for q in fp.Pads() if q.GetNumber() == ep_num), None)
            if ep is not None and ep.GetNetCode() == p.GetNetCode():
                # ground pins of a part with a ground exposed pad: short track
                # inward to the exposed pad
                b = p.GetBoundingBox()
                if b.GetWidth() > b.GetHeight():     # pin on a left/right side
                    end = pcbnew.VECTOR2I(ep.GetPosition().x, p.GetPosition().y)
                else:
                    end = pcbnew.VECTOR2I(p.GetPosition().x, ep.GetPosition().y)
                add_track(board, p, end, bottom, 0.2)
                if ep_via:
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
                angles = [0, 30, -30, 45, -45, 60, -60]
            width = 0.4 if min(sx, sy) > 0.2 else 0.2
            b = p.GetBoundingBox()
            own = (*xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop())), *xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom())))
            layer = "B" if bottom else "F"
            placed = False
            for dist in (0.15, 0.4, 0.7, 1.0, 1.4, 1.8, 2.2):
                for da in angles:
                    a = away + math.radians(da)
                    ext = abs(math.cos(a)) * sx + abs(math.sin(a)) * sy
                    d = ext + VIA / 2 + dist
                    x, y = px + d * math.cos(a), py + d * math.sin(a)
                    if not obs.via_ok(x, y, p.GetNetCode()):
                        continue
                    if not obs.track_ok(px, py, x, y, p.GetNetCode(), width, layer, own):
                        continue
                    z = zone_at(board, other, x, y) if L.LAYERS == 2 else None
                    if L.LAYERS == 2 and (z is None or z.GetNetname() != net):
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
                # no room for a via of its own: share the nearest one of its
                # net that a clear stub can reach
                vias = sorted((math.hypot(vx - px, vy - py), vx, vy) for vx, vy, vn in
                              (((a + c) / 2, (b + d) / 2, n) for a, b, c, d, n, _, iv in obs.items if iv)
                              if vn == p.GetNetCode())
                for d, vx, vy in vias:
                    if d > 2.5:
                        break
                    if obs.track_ok(px, py, vx, vy, p.GetNetCode(), width, layer, own):
                        add_track(board, p, v(vx, vy), bottom, width)
                        obs.add_track(px, py, vx, vy, p.GetNetCode(), width, layer)
                        placed = True
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


def power_layers(dsn):
    """In1 is a solid GND plane: tell Freerouting it is a power layer, so it
    routes nothing there. In2 is a +BATT pour that signals may cross, so it
    stays a signal layer and its pour is left out like the outer ones."""
    t = open(dsn).read()
    for name in ("In1.Cu",):
        t = t.replace(f"(layer {name}\n      (type signal)", f"(layer {name}\n      (type power)")
    open(dsn, "w").write(t)


def strip_pours(board):
    """Drop the board-wide pours and the MCU GND islands: Freerouting would
    treat them as solid copper."""
    for z in [board.GetArea(i) for i in range(board.GetAreaCount())]:
        if z.GetIsRuleArea():
            continue
        name = z.GetZoneName()
        if name in ("BATT top", "GND bottom", "BATT plane") or name.startswith("MCU GND"):
            board.Remove(z)


def signals(src, fanned, dsn):
    """Fan out, then write the DSN for every net except GND and +BATT."""
    board = pcbnew.LoadBoard(src)
    fanout(board)
    board.Save(fanned)
    write_dsn(board, dsn)


def fanout_only(src, fanned):
    board = pcbnew.LoadBoard(src)
    pwm_lines(board)
    fanout(board)
    board.Save(fanned)


# The six PWM lines from each MCU to its gate driver, and the MCU's 3V3 pin
# with its via, drawn by hand: the MCU pins are 0.4 mm apart and boxed in on
# both sides, which neither router gets out of reliably. Channel coordinates
# for the MCU and driver at layout.MCU_POS and DRV_POS, both on the bottom;
# 0.18 mm tracks keep every gap at 0.22 mm or more.
PWM_W = 0.18
PWM_PATHS = {
    # high side: up out of the MCU, over the low-side lines, into the
    # driver's left-hand pins
    "CH": [(6.0, 1.87), (6.0, 3.45), (8.0, 3.45), (8.5, 2.95), (9.6, 2.95)],
    "BH": [(5.6, 1.87), (5.6, 3.95), (8.1, 3.95), (8.6, 3.45), (9.6, 3.45)],
    "AH": [(5.2, 1.87), (5.2, 4.45), (8.2, 4.45), (8.7, 3.95), (9.6, 3.95)],
    # low side: across underneath, into the driver's lower pins from below
    "AL": [(7.2, 1.87), (7.2, 2.2), (7.55, 2.55), (8.9, 2.55), (8.9, 1.7), (10.25, 1.7), (10.25, 2.28)],
    "BL": [(7.87, 1.2), (10.75, 1.2), (10.75, 2.28)],
    "CL": [(7.87, 0.8), (8.5, 0.8), (8.7, 0.6), (11.25, 0.6), (11.25, 2.28)],
}
# the other boxed-in pins, on the MCU's -y side: 3V3 (above), NRST and VDDA
# drop to staggered vias for their parts on top; FBA and FBCOMMON get short
# diverging escapes that the maze router carries on from
ESCAPES = {
    "+3V3": [(6.4, 1.87), (6.4, 2.45), (6.75, 2.8)],
    "NRST": [(6.0, -1.87), (6.0, -2.8), (5.3, -3.4)],
    "vdda": [(6.4, -1.87), (6.4, -3.6), (6.5, -3.9)],
    "FBA": [(6.8, -1.87), (6.8, -2.5), (7.3, -3.0)],
    "FBCOMMON": [(7.2, -1.87), (7.2, -2.1), (7.6, -2.5)],
}
# The gate driver's high-side row (pins 13-20) the same way: the 0.5 mm pins
# sit between the PWM lines and the bootstrap caps, and their gate lines have
# to get past both. The three high-side gate lines drop to staggered vias
# and cross on top, where the board over the driver is free; each bootstrap
# (VB) and phase (VS) pin runs in its own lane into its cap (layout.CB_POS,
# phase pad 1 at the top end, VB pad 2 at the bottom end). The phase nets
# then go on to the phase copper with the routers.
DRIVER_PATHS = {
    "VB1": [(9.6, 4.95), (8.3, 4.95), (8.3, 11.375), (9.6, 11.375)],
    "GHA": [(9.6, 5.45), (9.15, 5.45), (9.15, 7.0)],
    "VS1": [(10.25, 6.12), (10.25, 8.4), (9.7, 8.4), (10.25, 8.4), (10.25, 8.9), (9.75, 9.4), (9.6, 9.825)],
    "VB2": [(10.75, 6.12), (10.75, 11.375), (11.6, 11.375)],
    "GHB": [(11.25, 6.12), (11.25, 7.8), (11.4, 8.2)],
    "VS2": [(11.75, 6.12), (11.75, 7.2), (12.05, 7.5), (12.05, 9.4), (11.6, 9.825)],
    "VB3": [(12.25, 6.12), (12.25, 6.9), (12.75, 7.4), (12.75, 10.775), (13.6, 10.775)],
    "GHC": [(12.75, 6.12), (12.75, 6.6), (13.5, 7.35)],
    "VS3": [(13.4, 5.45), (14.2, 5.45), (14.2, 8.4), (13.9, 8.8), (13.6, 9.225)],
}
# the phase pads of caps A and B are boxed in by the bootstrap lanes, so
# their phase lines get a via each to leave by
DRIVER_VIAS = {"GHA": (9.15, 7.0), "GHB": (11.4, 8.2), "GHC": (13.5, 7.35),
               "VS1": (9.7, 8.4), "VS2": (12.05, 9.0)}
DRIVER_PIN = {"VB1": "20", "GHA": "19", "VS1": "18", "VB2": "17", "GHB": "16",
              "VS2": "15", "VB3": "14", "GHC": "13", "VS3": "12"}


def pwm_lines(board):
    assert L.MCU_POS == (6.0, 0.0) and L.DRV_POS == (11.5, 4.2), \
        "PWM_PATHS and ESCAPES are drawn for the MCU at (6, 0) and the driver at (11.5, 4.2)"
    for ch, k in L.CHANNELS.items():
        mcu = next(fp for fp in board.GetFootprints() if fp.GetValue() == "AT32F421G8U7"
                   and any(p.GetNetname() == f"/{ch}/AL" for p in fp.Pads()))
        pin_net = {p.GetNumber(): p.GetNet() for p in mcu.Pads()}
        own = {"+3V3": pin_net["17"], "NRST": pin_net["4"], "vdda": pin_net["5"],
               "FBA": pin_net["6"], "FBCOMMON": pin_net["7"]}
        drv = next(fp for fp in board.GetFootprints() if fp.GetValue() == "NSG2065Q"
                   and any(p.GetNetname() == f"/{ch}/AL" for p in fp.Pads()))
        dpin = {p.GetNumber(): p.GetNet() for p in drv.Pads()}
        own.update({name: dpin[num] for name, num in DRIVER_PIN.items()})
        for name, pts in list(PWM_PATHS.items()) + list(ESCAPES.items()) + list(DRIVER_PATHS.items()):
            net = own.get(name) or board.FindNet(f"/{ch}/{name}")
            pts = [L.rot(x, y, k) for x, y in pts]
            for (ax, ay), (bx, by) in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(v(ax, ay))
                t.SetEnd(v(bx, by))
                t.SetWidth(MM(PWM_W))
                t.SetLayer(pcbnew.B_Cu)
                t.SetNet(net)
                board.Add(t)
        # FBA and FBCOMMON escapes into RFCA's pads
        rfca = next(fp for fp in board.GetFootprints() if fp.IsFlipped() and math.hypot(
            *(a - b for a, b in zip(xy(fp.GetPosition()), L.rot(*L.RFCA_POS, k)))) < 0.05)
        for pad in rfca.Pads():
            name = "FBCOMMON" if pad.GetNetname().endswith("FBCOMMON") else "FBA"
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(v(*L.rot(*ESCAPES[name][-1], k)))
            t.SetEnd(pad.GetPosition())
            t.SetWidth(MM(PWM_W))
            t.SetLayer(pcbnew.B_Cu)
            t.SetNet(pad.GetNet())
            board.Add(t)
        # driver pin 4 (+10V) down to its supply cap's +10V pad
        cap = next(fp for fp in board.GetFootprints() if fp.IsFlipped() and math.hypot(
            *(a - b for a, b in zip(xy(fp.GetPosition()), L.rot(*L.CVCC_POS, k)))) < 0.05)
        pad = next(p for p in cap.Pads() if p.GetNetname() == "+10V")
        px, py = L.rot(*xy(pad.GetPosition()), -k)
        assert abs(px - (L.CVCC_POS[0] - 0.775)) < 0.01, "CVCC +10V pad must face the driver"
        pts = [(11.75, 2.28), (11.75, 1.3), (px - 0.25, py + 0.25), (px, py)]
        for (ax, ay), (bx, by) in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(v(*L.rot(ax, ay, k)))
            t.SetEnd(v(*L.rot(bx, by, k)))
            t.SetWidth(MM(PWM_W))
            t.SetLayer(pcbnew.B_Cu)
            t.SetNet(pad.GetNet())
            board.Add(t)
        # 3V3 via up to the decoupling cap's pad on top
        cap = next(fp for fp in board.GetFootprints() if not fp.IsFlipped() and any(
            p.GetNet().GetNetname() == "+3V3" for p in fp.Pads()) and math.hypot(
            *(a - b for a, b in zip(xy(fp.GetPosition()), L.rot(*L.C3V3_POS, k)))) < 0.05)
        pad = next(p for p in cap.Pads() if p.GetNetname() == "+3V3")
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(v(*L.rot(*L.MCU_VIAS["+3V3"], k)))
        t.SetEnd(pad.GetPosition())
        t.SetWidth(MM(0.25))
        t.SetLayer(pcbnew.F_Cu)
        t.SetNet(pad.GetNet())
        board.Add(t)
        for name, pos in list(L.MCU_VIAS.items()) + list(DRIVER_VIAS.items()):
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(v(*L.rot(*pos, k)))
            via.SetWidth(MM(VIA))
            via.SetDrill(MM(DRILL))
            via.SetNet(own[name])
            board.Add(via)


def resume(routed, dsn):
    """DSN of a board that already has some routing (the pre-routed local
    nets, or an earlier Freerouting run), kept as protected wiring."""
    write_dsn(pcbnew.LoadBoard(routed), dsn)


def write_dsn(board, dsn):
    # +BATT highways on top, centre to each channel's high-side drains
    for k in range(4 if L.LAYERS == 2 else 0):
        pts = [L.rot(x, y, k) for x, y in
               ((7.5, -9.0), (L.FET_X - 1.0, -9.0), (L.FET_X - 1.0, 9.0), (7.5, 9.0))]
        add_keepout(board, pts, pcbnew.F_Cu, f"BATT highway {k}")
    # the low-side source tabs are GND, which the router skips entirely, so
    # its copper would be invisible to it: keep the router out instead
    for z in [z for z in board.Zones() if z.GetZoneName().endswith(" source")]:
        add_keepout(board, z.Outline().COutline(0), z.GetLayer(), "keepout " + z.GetZoneName(), vias=True)
        board.Remove(z)
    strip_pours(board)
    print("dsn", dsn, pcbnew.ExportSpecctraDSN(board, dsn))
    power_layers(dsn)
    phase_wires(dsn)
    t = open(dsn).read()
    # Freerouting rounds: aim 5 um wide of the fab's 0.2 mm so the result
    # doesn't land at 0.199 mm
    t = t.replace("(clearance 200)", "(clearance 205)")
    open(dsn, "w").write(t.replace("(type route)", "(type protect)"))


def phase_wires(dsn):
    """KiCad exports the remaining zones (phase strips, the battery + pad) as
    Specctra planes, which Freerouting lets other nets
    run straight through. Only the In1 GND plane stays a plane. As protected polygon wiring they are
    solid copper to every other net and still a place for their own net's
    driver, bootstrap and BEMF pins to connect to."""
    t = open(dsn).read()
    wires = []
    out, i = [], 0
    while True:
        j = t.find("(plane ", i)
        if j < 0:
            out.append(t[i:])
            break
        depth, k = 0, j
        while True:
            depth += {"(": 1, ")": -1}.get(t[k], 0)
            k += 1
            if depth == 0:
                break
        block = t[j:k]
        net = block.split()[1]
        out.append(t[i:j])
        if "(polygon In1.Cu" not in block:
            poly = block[block.index("(polygon"):-1]
            wires.append(f"    (wire {poly} (net {net})(type protect))\n")
        else:
            out.append(block)
        i = k
    t = "".join(out)
    t = t.replace("  (wiring\n", "  (wiring\n" + "".join(wires), 1)
    open(dsn, "w").write(t)



if __name__ == "__main__":
    {"signals": signals, "fanout": fanout_only, "resume": resume}[sys.argv[1]](*sys.argv[2:])
