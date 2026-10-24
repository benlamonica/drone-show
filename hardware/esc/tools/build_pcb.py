"""Build drone-esc.kicad_pcb from the schematic netlist.

Run with KiCad's Python from hardware/esc (see tools/build.sh):
    python3 tools/build_pcb.py build/netlist.json build/placed.kicad_pcb

This places every part, draws the outline, the power copper (zones) and the
power vias. Signal routing is done afterwards by Freerouting, then
tools/finish_pcb.py imports the routes and fills the zones.
"""
import json
import math
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

MM = pcbnew.FromMM
PRJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
STOCK = os.environ.get("KICAD_FOOTPRINTS", "/usr/share/kicad/footprints")
LOCAL_LIB = os.path.join(PRJ, "lib", "drone-esc.pretty")
CX, CY = 150.0, 100.0            # board centre on the page
io = pcbnew.PCB_IO_KICAD_SEXPR()

POWER_NETS = {"+BATT", "GND", "/CSA+"}


def is_power(net):
    return net in POWER_NETS or "/Motor" in net


def v(x, y):
    return pcbnew.VECTOR2I(MM(CX + x), MM(CY + y))


def xy(vec):
    return pcbnew.ToMM(vec.x) - CX, pcbnew.ToMM(vec.y) - CY


# --------------------------------------------------------------- netlist
class Netlist:
    def __init__(self, path):
        d = json.load(open(path))
        self.comps = {c["ref"]: c for c in d["components"]}
        self.nets = d["nets"]
        self.pin_net = {}
        for name, nodes in self.nets.items():
            for ref, pin in nodes:
                self.pin_net[(ref, pin)] = name

    def refs_on(self, net, prefix=None, sheet=None):
        out = []
        for ref, pin in self.nets.get(net, []):
            if prefix and not ref.startswith(prefix):
                continue
            if sheet and self.comps[ref]["sheet"] != sheet:
                continue
            out.append(ref)
        return out

    def other_net(self, ref, net):
        for pin in ("1", "2"):
            if self.pin_net.get((ref, pin)) != net:
                return self.pin_net.get((ref, pin))


def channel_roles(nl, ch):
    """Map role names to references for one channel sheet, from connectivity."""
    s = f"/{ch}/"
    r = {}
    comps = [c for c in nl.comps.values() if c["sheet"] == s]
    r["MCU"] = next(c["ref"] for c in comps if c["value"] == "AT32F421G8U7")
    r["DRV"] = next(c["ref"] for c in comps if c["value"] == "NSG2065Q")
    for ph in "ABC":
        m = f"{s}Motor{ph}"
        r[f"HS{ph}"] = next(q for q, p in nl.nets[m] if q.startswith("Q") and p == "1")
        r[f"LS{ph}"] = next(q for q, p in nl.nets[m] if q.startswith("Q") and p == "3")
        r[f"RGH{ph}"] = nl.refs_on(f"{s}GH{ph}", "R")[0]
        r[f"RGL{ph}"] = nl.refs_on(f"{s}GL{ph}", "R")[0]
        r[f"CB{ph}"] = nl.refs_on(m, "C")[0]
        r[f"RFT{ph}"] = nl.refs_on(m, "R")[0]
        fb = f"{s}FB{ph}"
        for ref in nl.refs_on(fb, "R"):
            o = nl.other_net(ref, fb)
            if o == "GND":
                r[f"RFB{ph}"] = ref
            elif o.endswith("FBCOMMON"):
                r[f"RFC{ph}"] = ref
    mcu = r["MCU"]
    r["C3V3"] = nl.refs_on("+3V3", "C", s)[0]
    r["CVCC"] = nl.refs_on("+10V", "C", s)[0]
    r["RVDDA"] = nl.refs_on(f"{s}vdda", "R")[0]
    r["CVDDA"] = nl.refs_on(f"{s}vdda", "C")[0]
    r["RNRST"] = nl.refs_on(f"Net-({mcu}-NRST)", "R")[0]
    r["CNRST"] = nl.refs_on(f"Net-({mcu}-NRST)", "C")[0]
    r["RBOOT"] = nl.refs_on(f"Net-({mcu}-BOOT0)", "R")[0]
    r["RDSHOT"] = nl.refs_on(f"Net-({mcu}-PB4)", "R")[0]
    r["TPDIO"] = nl.refs_on(f"Net-({mcu}-PA13)", "TP")[0]
    r["TPCLK"] = nl.refs_on(f"Net-({mcu}-PA14)", "TP")[0]
    return r


# --------------------------------------------------------------- footprints
def load_fp(fpid):
    lib, name = fpid.split(":")
    path = LOCAL_LIB if lib == "drone-esc" else os.path.join(STOCK, f"{lib}.pretty")
    fp = io.FootprintLoad(path, name)
    if fp is None:
        raise RuntimeError(f"footprint {fpid} not found")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    return fp


def local_to_screen(lx, ly, theta, bottom):
    if bottom:
        ly = -ly
    c, s = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    return lx * c + ly * s, -lx * s + ly * c


def orient_for(local_axis, screen_angle, bottom):
    """Footprint orientation that points a local axis at a screen angle (deg, y down)."""
    want = (math.cos(math.radians(screen_angle)), math.sin(math.radians(screen_angle)))
    best = min(range(0, 360, 45), key=lambda t: -sum(
        a * b for a, b in zip(local_to_screen(*local_axis, t, bottom), want)))
    return best


AXES = {"+x": (1, 0), "-x": (-1, 0), "+y": (0, 1), "-y": (0, -1)}


def place(fp, x, y, axis="+x", angle=0.0, bottom=False):
    if bottom and not fp.IsFlipped():
        fp.Flip(fp.GetPosition(), pcbnew.FLIP_DIRECTION_LEFT_RIGHT)
    fp.SetOrientationDegrees(orient_for(AXES[axis], angle, bottom))
    fp.SetPosition(v(x, y))


def bbox(fp, margin=0.0):
    layer = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
    poly = fp.GetCourtyard(layer)
    if poly.OutlineCount():
        b = poly.BBox()
        x0, y0 = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
        x1, y1 = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
    else:
        xs, ys = [], []
        for p in fp.Pads():
            b = p.GetBoundingBox()
            a = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
            c = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
            xs += [a[0], c[0]]
            ys += [a[1], c[1]]
        x0, y0, x1, y1 = min(xs) - 0.25, min(ys) - 0.25, max(xs) + 0.25, max(ys) + 0.25
    return (x0 - margin, y0 - margin, x1 + margin, y1 + margin)


# --------------------------------------------------------------- legaliser
class Floor:
    """Keeps track of what is where on each side, and finds free spots."""

    def __init__(self):
        self.rects = {"top": [], "bot": []}
        self.circles = [(x, y, L.HOLE_PART_KEEPOUT) for x, y in L.holes()]
        self.circles_bot = []

    def add(self, side, r, tag=""):
        sides = ("top", "bot") if side == "both" else (side,)
        for s in sides:
            self.rects[s].append((r, tag))

    def inside(self, r):
        m = L.EDGE_MARGIN
        lim = L.CHAMFER * L.S2 - m * L.S2
        for x in (r[0], r[2]):
            for y in (r[1], r[3]):
                if abs(x) > L.HALF - m or abs(y) > L.HALF - m or abs(x) + abs(y) > lim:
                    return False
        return True

    def free(self, side, r, gap=0.22):
        if not self.inside(r):
            return False
        for (o, _) in self.rects[side]:
            if r[0] < o[2] + gap and r[2] > o[0] - gap and r[1] < o[3] + gap and r[3] > o[1] - gap:
                return False
        for cx, cy, rad in self.circles + (self.circles_bot if side == "bot" else []):
            nx = min(max(cx, r[0]), r[2])
            ny = min(max(cy, r[1]), r[3])
            if (nx - cx) ** 2 + (ny - cy) ** 2 < rad ** 2:
                return False
        return True

    def settle(self, fp, x, y, axis, angle, bottom, max_r=12.0, step=0.2, gap=0.22):
        """Place fp at the free spot nearest (x, y)."""
        side = "bot" if bottom else "top"
        place(fp, x, y, axis, angle, bottom)
        b = bbox(fp)
        w0, h0 = b[0] - x, b[1] - y
        w1, h1 = b[2] - x, b[3] - y
        r = 0.0
        while r <= max_r:
            n = max(1, int(2 * math.pi * r / step))
            for i in range(n):
                a = 2 * math.pi * i / n
                px, py = x + r * math.cos(a), y + r * math.sin(a)
                cand = (px + w0, py + h0, px + w1, py + h1)
                if self.free(side, cand, gap):
                    fp.SetPosition(v(px, py))
                    self.add(side, cand, fp.GetReference())
                    return px, py
            r += step
        print(f"WARNING: no room for {fp.GetReference()} near ({x:.1f}, {y:.1f}), left overlapping")
        fp.SetPosition(v(x, y))
        self.add(side, (x + w0, y + h0, x + w1, y + h1), fp.GetReference())
        return x, y

    def fix(self, fp, side=None):
        s = side or ("bot" if fp.IsFlipped() else "top")
        self.add(s, bbox(fp), fp.GetReference())


# --------------------------------------------------------------- board
def setup_board(board):
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(MM(1.6))
    pts = L.outline()
    for (x0, y0), (x1, y1) in zip(pts, pts[1:] + pts[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(v(x0, y0))
        seg.SetEnd(v(x1, y1))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(MM(0.1))
        board.Add(seg)


def add_text(board, text, x, y, size=1.0, layer=pcbnew.F_SilkS, angle=0):
    t = pcbnew.PCB_TEXT(board)
    t.SetText(text)
    t.SetPosition(v(x, y))
    t.SetLayer(layer)
    t.SetTextSize(pcbnew.VECTOR2I(MM(size), MM(size)))
    t.SetTextThickness(MM(0.15 if size < 1.2 else 0.2))
    t.SetTextAngleDegrees(angle)
    if layer in (pcbnew.B_SilkS,):
        t.SetMirrored(True)
    board.Add(t)


def add_via(board, net, x, y, size=0.8, drill=0.4):
    via = pcbnew.PCB_VIA(board)
    via.SetPosition(v(x, y))
    via.SetWidth(MM(size))
    via.SetDrill(MM(drill))
    via.SetNet(net)
    board.Add(via)
    return via


def add_zone(board, net, layer, pts, priority=0, clearance=0.3, name=""):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(net)
    o = z.Outline()
    o.NewOutline()
    for x, y in pts:
        o.Append(MM(CX + x), MM(CY + y))
    z.SetAssignedPriority(priority)
    z.SetLocalClearance(MM(clearance))
    z.SetMinThickness(MM(0.25))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    if name:
        z.SetZoneName(name)
    board.Add(z)
    return z


def rect_pts(x0, y0, x1, y1):
    return [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]


def rot_pts(pts, k):
    return [L.rot(x, y, k) for x, y in pts]


# --------------------------------------------------------------- main
PHASE_HW = 2.5   # half width of a phase strip; strips are 1.5 mm apart


def phase_top_pts(hs, y, k):
    """Top phase strip of one half-bridge, in channel coordinates. It covers the
    high-side source pins, with a notch round the gate pin so the gate track
    can leave sideways into the gap between strips instead of being boxed in
    by phase copper."""
    x0, x1 = L.FET_X + 3.37 - 1.15, L.HALF + 1
    y0, y1 = y - PHASE_HW, y + PHASE_HW
    gate = next(p for p in hs.Pads() if p.GetNumber() == "2")
    gx, gy = L.rot(*xy(gate.GetPosition()), -k)
    nx1 = gx + 0.54 + 0.4
    if gy > y:
        ny = gy - 0.29 - 0.3
        return [(x0, y0), (x1, y0), (x1, y1), (nx1, y1), (nx1, ny), (x0, ny)]
    ny = gy + 0.29 + 0.3
    return [(x0, ny), (nx1, ny), (nx1, y0), (x1, y0), (x1, y1), (x0, y1)]


def build(netlist_path, out_path):
    nl = Netlist(netlist_path)
    board = pcbnew.BOARD()
    setup_board(board)

    nets = {}
    for name in nl.nets:
        n = pcbnew.NETINFO_ITEM(board, name)
        board.Add(n)
        nets[name] = n

    fps = {}
    for ref, c in sorted(nl.comps.items()):
        fp = load_fp(c["footprint"])
        fp.SetReference(ref)
        fp.SetValue(c["value"])
        fp.SetPath(pcbnew.KIID_PATH(c["path"]))
        for name in ("MPN", "Manufacturer", "LCSC"):
            val = c["fields"].get(name, "")
            if val:
                fp.SetField(name, val)
                fp.GetField(name).SetVisible(False)
        board.Add(fp)
        fp.SetPosition(v(0, 0))
        for p in fp.Pads():
            net = nl.pin_net.get((ref, p.GetNumber()))
            if net:
                p.SetNet(nets[net])
        fps[ref] = fp

    floor = Floor()

    # composite pads footprint at the centre
    u3 = fps["U3"]
    u3.SetPosition(v(0, 0))
    for p in u3.Pads():
        b = p.GetBoundingBox()
        a = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
        c = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
        side = "both" if p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH else "top"
        floor.add(side, (a[0] - 0.2, a[1] - 0.2, c[0] + 0.2, c[1] + 0.2), "U3")

    bulk = sorted((r for r in nl.comps if r.startswith("C") and int(r[1:]) >= 100), key=lambda r: int(r[1:]))
    roles = {ch: channel_roles(nl, ch) for ch in L.CHANNELS}

    # ---- power stage, fixed positions, all four channels
    cap_x = L.FET_X - 3.37 - 0.3 - 2.05
    for ch, k in L.CHANNELS.items():
        r = roles[ch]
        caps = bulk[6 * k: 6 * k + 6]
        for i, ph in enumerate("ABC"):
            y = L.PHASE_Y[ph]
            x, yy = L.rot(L.FET_X, y, k)
            place(fps[r[f"HS{ph}"]], x, yy, "+y", 90 * k, bottom=False)
            place(fps[r[f"LS{ph}"]], x, yy, "+y", 180 + 90 * k, bottom=True)
            floor.fix(fps[r[f"HS{ph}"]])
            floor.fix(fps[r[f"LS{ph}"]])
            for j, dy in enumerate((-1.15, 1.15)):
                cx, cy = L.rot(cap_x, y + dy, k)
                place(fps[caps[2 * i + j]], cx, cy, "-x", 90 * k, bottom=False)
                floor.fix(fps[caps[2 * i + j]])
        # GND vias beside the bulk caps, phase stitching, gate resistors
        for i, ph in enumerate("ABC"):
            y = L.PHASE_Y[ph]
            for dy in (-1.15, 1.15):
                gx, gy = L.rot(cap_x - 2.05 - 0.7, y + dy, k)
                rr = (gx - 0.55, gy - 0.55, gx + 0.55, gy + 0.55)
                if floor.free("top", rr) and floor.free("bot", rr):
                    add_via(board, nets["GND"], gx, gy)
                    floor.add("both", rr, "via")
            m = nets[f"/{ch}/Motor{ph}"]
            for sx in (L.FET_X + 3.37 + 0.75, L.FET_X + 3.37 + 1.85):
                for sy in (-1.3, 0.0, 1.3):
                    px, py = L.rot(sx, y + sy, k)
                    add_via(board, m, px, py)
                    floor.add("both", (px - 0.6, py - 0.6, px + 0.6, py + 0.6), "via")

    # ---- channel logic. Gate driver, bootstrap caps and gate resistors on the
    # bottom next to the FETs; MCU and its parts on top, on a GND island in
    # the corner clockwise from the channel's edge.
    fx = L.FET_X - 3.37          # inner edge of the FETs
    for ch, k in L.CHANNELS.items():
        r = roles[ch]

        def put(role, x, y, axis="+x", ang=0.0, bottom=True, **kw):
            px, py = L.rot(x, y, k)
            floor.settle(fps[r[role]], px, py, axis, ang + 90 * k, bottom, **kw)

        for ph in "ABC":
            y = L.PHASE_Y[ph]
            put(f"RGL{ph}", fx - 1.7, y - 1.9)
            put(f"RGH{ph}", fx - 1.7, y - 3.25)
        put("DRV", 10.6, 10.6, "+y", 0)
        for ph, (x, y) in zip("ABC", ((13.6, 7.8), (14.2, 9.6), (13.4, 11.4))):
            put(f"CB{ph}", x, y, "+x", 90)
        put("CVCC", 7.4, 12.0, "+x", 90)
        # top side
        put("MCU", 21.6, 14.2, "+y", 270, False)
        put("C3V3", 18.6, 10.9, "+x", 0, False)
        for role, (x, y, a) in {"RBOOT": (25.6, 11.4, 90), "RNRST": (27.2, 11.4, 90),
                                "CNRST": (28.8, 11.4, 90), "RVDDA": (25.4, 14.6, 90),
                                "CVDDA": (27.0, 14.0, 90), "RFCA": (25.0, 17.4, 90),
                                "RFBA": (23.4, 17.6, 90), "RFCB": (21.8, 17.6, 90),
                                "RFBB": (20.2, 17.6, 90), "RFCC": (19.4, 19.6, 0),
                                "RFBC": (21.4, 19.8, 0), "RFTA": (30.0, 11.0, 0),
                                "RFTB": (29.4, 12.6, 0), "RFTC": (24.4, 10.8, 0),
                                "RDSHOT": (18.6, 22.0, 0), "TPDIO": (13.6, 21.6, 0),
                                "TPCLK": (11.6, 23.6, 0)}.items():
            put(role, x, y, "+x", a, False, step=0.15)

    # ---- shared parts
    def put_shared(ref, x, y, axis="+x", ang=0.0, bottom=False, **kw):
        floor.settle(fps[ref], x, y, axis, ang, bottom, **kw)

    # battery wires are soldered on the bottom too: keep a clear ring round them
    for x, y, _ in L.battery_pads().values():
        floor.circles_bot.append((x, y, L.BATT_PAD_D / 2 + 1.6))
    # FC connector, top entry, centre top
    put_shared("J1", L.JST_POS[0], L.JST_POS[1], "+x", 0)
    # current sense on top: battery + hole -> two 0.2 mOhm shunts -> +BATT pour
    bx, by = L.battery_pads()["1"][:2]
    put_shared("Rsense1", bx - 2.0, by + 7.2, "+x", 90, False, step=0.1)
    put_shared("Rsense2", bx + 2.0, by + 7.2, "+x", 90, False, step=0.1)
    for ref, (x, y, ang) in {"U12": (4.6, 8.4, 0), "R89": (2.6, 11.0, 90), "R90": (2.6, 6.4, 90),
                             "C40": (7.2, 10.8, 90), "C41": (7.2, 6.6, 90), "C42": (4.6, 11.4, 0),
                             "R73": (8.2, 8.6, 90), "C94": (9.6, 8.6, 90)}.items():
        put_shared(ref, x, y, "+x", ang, False)
    # supplies on the bottom, centre
    for ref, (x, y, ang) in {"U13": (-4.2, -5.0, 0), "U14": (-1.0, -5.6, 90), "C75": (-2.6, -7.8, 0),
                             "R120": (-7.0, -3.2, 90), "R121": (-7.0, -6.2, 90),
                             "C2": (-6.0, -9.6, 90), "C3": (-2.6, -10.2, 90), "C6": (0.8, -10.2, 90),
                             "C74": (-8.6, -9.0, 90), "C47": (1.6, -3.4, 0), "C76": (1.6, -1.6, 0),
                             "U15": (4.4, -4.4, 0), "C86": (1.6, -6.8, 90), "C4": (7.4, -3.0, 90),
                             "C51": (7.4, -6.4, 90), "C81": (4.4, -7.6, 0), "C93": (4.4, -1.4, 0)}.items():
        put_shared(ref, x, y, "+x", ang, True)

    # ---- power copper
    out = L.outline()
    add_zone(board, nets["+BATT"], pcbnew.F_Cu, out, 0, name="BATT top")
    add_zone(board, nets["GND"], pcbnew.B_Cu, out, 0, name="GND bottom")
    for ch, k in L.CHANNELS.items():
        for ph in "ABC":
            y = L.PHASE_Y[ph]
            n = nets[f"/{ch}/Motor{ph}"]
            top = phase_top_pts(fps[roles[ch][f"HS{ph}"]], y, k)
            bot = rect_pts(L.FET_X - 3.37 + 1.35, y - PHASE_HW, L.HALF + 1, y + PHASE_HW)
            add_zone(board, n, pcbnew.F_Cu, rot_pts(top, k), 2, name=f"{ch} {ph} top")
            add_zone(board, n, pcbnew.B_Cu, rot_pts(bot, k), 2, name=f"{ch} {ph} bottom")
    # top GND island under each channel's MCU, stitched to the bottom pour
    for corner_k in range(4):
        pts = rot_pts([(9.9, 9.9), (L.HALF + 2, 9.9), (9.9, L.HALF + 2)], corner_k)
        add_zone(board, nets["GND"], pcbnew.F_Cu, pts, 1, name=f"MCU GND {corner_k}")
        placed = 0
        for u in (13.0, 16.0, 19.0, 22.0, 25.0, 28.0):
            for w in (-9.0, -6.0, -3.0, 0.0, 3.0, 6.0, 9.0):
                x0 = (u + w) / L.S2
                y0 = (u - w) / L.S2
                if x0 < 10.6 or y0 < 10.6:
                    continue
                gx, gy = L.rot(x0, y0, corner_k)
                rr = (gx - 0.6, gy - 0.6, gx + 0.6, gy + 0.6)
                if floor.free("top", rr, 0.1) and floor.free("bot", rr, 0.1):
                    add_via(board, nets["GND"], gx, gy)
                    floor.add("both", rr, "via")
                    placed += 1
        print("corner", corner_k, "stitching vias", placed)
    # battery + hole to the shunts' input pads
    bx, by = L.battery_pads()["1"][:2]
    add_zone(board, nets["/CSA+"], pcbnew.F_Cu, rect_pts(bx - 4.2, by - 3.0, bx + 4.2, by + 4.2), 3,
             name="battery +")

    # silkscreen
    add_text(board, "drone-show ESC  4-6S  AM32", 0, -1.6, 0.8)
    bp = L.battery_pads()
    add_text(board, "BAT+", bp["1"][0], bp["1"][1] + 3.2, 0.8, pcbnew.B_SilkS)
    add_text(board, "BAT-", bp["2"][0], bp["2"][1] + 3.2, 0.8, pcbnew.B_SilkS)
    for ch, k in L.CHANNELS.items():
        x, y = L.rot(L.MOTOR_PAD_X - 4.4, -10.6, k)
        add_text(board, ch[-1], x, y, 1.4)

    board.Save(out_path)
    json.dump({ch: roles[ch] for ch in roles}, open(os.path.join(os.path.dirname(out_path), "roles.json"), "w"), indent=1)
    print("saved", out_path)


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2])
