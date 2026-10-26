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
    """Courtyard and pads together: some library courtyards are smaller than
    the pads."""
    xs, ys = [], []
    for p in fp.Pads():
        b = p.GetBoundingBox()
        a = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
        c = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
        xs += [a[0] - 0.1, c[0] + 0.1]
        ys += [a[1] - 0.1, c[1] + 0.1]
    layer = pcbnew.B_CrtYd if fp.IsFlipped() else pcbnew.F_CrtYd
    poly = fp.GetCourtyard(layer)
    if poly.OutlineCount():
        b = poly.BBox()
        a = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
        c = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
        xs += [a[0], c[0]]
        ys += [a[1], c[1]]
    x0, y0, x1, y1 = min(xs), min(ys), max(xs), max(ys)
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

    def settle(self, fp, x, y, axis, angle, bottom, max_r=20.0, step=0.2, gap=0.22):
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
    board.SetCopperLayerCount(L.LAYERS)
    if L.LAYERS == 4:
        # plane layers. They stay "signal" in KiCad so the DSN export
        # includes them; route_prep marks them as power layers for the router.
        pass
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
HALO = {"MCU": 1.0, "DRV": 0.8}
LOGIC = L.FET_X / 20.5
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
    source_tabs = []
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
            if L.LAYERS == 4:
                # low-side source pins: a bottom GND tab from the pins in to two
                # vias onto the GND plane. The router treats the tab as solid
                # copper, so gate tracks can't cut the sources off.
                src = [L.rot(*xy(p.GetPosition()), -k)[1] - y
                       for p in fps[r[f"LS{ph}"]].Pads() if p.GetNumber() == "1"]
                lo, hi = min(src) - 0.6, max(src) + 0.45
                tab = rect_pts(cap_x - 3.4, y + lo, L.FET_X - 2.83 + 0.55, y + hi)
                source_tabs.append((rot_pts(tab, k), f"{ch} {ph} source"))
                for dy in (lo + 0.6, hi - 0.6):
                    gx, gy = L.rot(cap_x - 2.75, y + dy, k)
                    add_via(board, nets["GND"], gx, gy, 0.66, 0.3)
                    floor.add("both", (gx - 0.55, gy - 0.55, gx + 0.55, gy + 0.55), "via")
            else:
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

    # ---- channel logic, all on the bottom (GND pour) in the channel's wedge
    # between the centre and the FETs, so the corners are free for the
    # battery and FC connections. Gate driver just inboard of the FETs with
    # its bootstrap caps and the gate resistors; the MCU on the centre side
    # of the driver, so the six PWM lines are short and stay on one layer.
    fx = L.FET_X - 3.37          # inner edge of the FETs
    for ch, k in L.CHANNELS.items():
        r = roles[ch]

        def put(role, x, y, axis="+x", ang=0.0, bottom=True, scale=True, exact=False, **kw):
            # the logic positions were laid out for FET_X = 20.5 and scale with it
            if scale:
                x, y = x * LOGIC, y * LOGIC
            px, py = L.rot(x, y, k)
            if exact:
                # pre-routed parts: exactly here in every channel
                place(fps[r[role]], px, py, axis, ang + 90 * k, bottom)
                floor.fix(fps[r[role]])
            else:
                floor.settle(fps[r[role]], px, py, axis, ang + 90 * k, bottom, **kw)
            if role in HALO:
                # keep a ring clear round the fine-pitch ICs so every pin can escape
                b = bbox(fps[r[role]])
                m = HALO[role]
                floor.add("bot" if bottom else "top", (b[0] - m, b[1] - m, b[2] + m, b[3] + m), role)

        for ph in "ABC":
            y = L.PHASE_Y[ph]
            put(f"RGL{ph}", fx - 1.7, y - 1.9, scale=False)
            put(f"RGH{ph}", fx - 1.7, y - 3.25, scale=False)
        # driver: low-side gate outputs face the FETs, high-side outputs and
        # bootstrap pins face +y, PWM inputs (pins 1-3, 22-24) face the MCU
        put("DRV", L.DRV_POS[0], L.DRV_POS[1], "+x", 90, scale=False, exact=True)
        # route_prep.pwm_lines draws the driver's high-side row into the
        # bootstrap caps (placed exactly) and drops its gate lines to vias:
        # keep that area clear
        for x0, y0, x1, y1, side in ((8.0, 4.7, 14.5, 11.9, "bot"), (8.6, 6.45, 14.0, 8.7, "both")):
            pts = [L.rot(x, y, k) for x, y in ((x0, y0), (x1, y1))]
            floor.add(side, (min(p[0] for p in pts), min(p[1] for p in pts),
                             max(p[0] for p in pts), max(p[1] for p in pts)), "drawn")
        for ph, (x, y) in L.CB_POS.items():
            put(f"CB{ph}", x, y, "+x", 90, scale=False, exact=True)
            c = fps[r[f"CB{ph}"]]
            vb = next(p for p in c.Pads() if p.GetNetname().startswith("Net-("))
            assert L.rot(*xy(vb.GetPosition()), -k)[1] > y, "bootstrap cap VB pad must be at the bottom end"
        # driver supply cap, exactly, +10V pad towards pin 4: route_prep
        # draws the hop (pin 4 is boxed in by the CL line)
        put("CVCC", L.CVCC_POS[0], L.CVCC_POS[1], "+x", 0, scale=False, exact=True)
        c = fps[r["CVCC"]]
        p10 = next(p for p in c.Pads() if p.GetNetname() == "+10V")
        if L.rot(*xy(p10.GetPosition()), -k)[0] > L.CVCC_POS[0]:
            c.SetOrientationDegrees(c.GetOrientationDegrees() + 180)
        # MCU: PWM pins (13-15, 18-20) face the driver; the low-side lines
        # go straight across, the high-side ones loop above them (drawn by
        # route_prep.pwm_lines). FBB/FBC
        # face +x and FBA/FBCOMMON sit at the +x end of the -y side, next to
        # the back-EMF dividers; DShot (PB4) faces the board centre.
        put("MCU", L.MCU_POS[0], L.MCU_POS[1], "+x", 90, scale=False, exact=True)
        # in the same order as the pins (FBB under FBC on the +x side, FBA
        # on the -y side), so no back-EMF line has to cross another
        # RFCA exactly, FBCOMMON pad towards the MCU: route_prep runs the FBA and
        # FBCOMMON escapes straight into it
        put("RFCA", L.RFCA_POS[0], L.RFCA_POS[1], "+x", 90, scale=False, exact=True)
        c = fps[r["RFCA"]]
        fbc = next(p for p in c.Pads() if p.GetNetname().endswith("FBCOMMON"))
        if L.rot(*xy(fbc.GetPosition()), -k)[1] < L.RFCA_POS[1]:
            c.SetOrientationDegrees(c.GetOrientationDegrees() + 180)
        rows = {"RFBB": (10.0, -1.9), "RFCB": (11.6, -1.9), "RFBC": (13.2, -1.9),
                "RFCC": (14.8, -1.9), "RFBA": (9.6, -4.0),
                "RFTA": (9.4, -6.2), "RFTB": (11.2, -6.2), "RFTC": (13.0, -6.2),
                "RDSHOT": (2.4, 0.0)}
        for role, (x, y) in rows.items():
            ax = "+y" if role == "RDSHOT" else "+x"
            put(role, x, y, ax, 90, scale=False, step=0.1)
        # BOOT0, NRST, VDDA and 3V3 drop through vias to their
        # parts on top, right over the MCU, where the board has room; the 3V3
        # pin is boxed in by the PWM lines on the bottom anyway. SWD test
        # points on top too, right over the MCU's SWD pins: the top side
        # over the driver is where its high-side gate lines cross, and the
        # back-EMF dividers need theirs for vias.
        for x, y in L.MCU_VIAS.values():
            vx, vy = L.rot(x, y, k)
            floor.add("both", (vx - 0.55, vy - 0.55, vx + 0.55, vy + 0.55), "via")
        for role, (x, y, a) in {"RBOOT": (4.2, -4.6, 90), "RNRST": (5.3, -4.9, 90),
                                "CNRST": (6.6, -5.4, 270), "RVDDA": (7.8, -4.7, 90),
                                "CVDDA": (8.4, -2.9, 0)}.items():
            put(role, x, y, "+x", a, False, scale=False, step=0.1)
        # 3V3 decoupling right over the 3V3 via; route_prep draws the hop
        put("C3V3", L.C3V3_POS[0], L.C3V3_POS[1], "+x", 0, False, scale=False, exact=True)
        put("TPDIO", 4.2, 1.2, "+x", 0, False, scale=False)
        put("TPCLK", 5.0, -1.0, "+x", 0, False, scale=False)

    # ---- shared parts
    def put_shared(ref, x, y, axis="+x", ang=0.0, bottom=False, **kw):
        floor.settle(fps[ref], x, y, axis, ang, bottom, **kw)

    def gnd_via(ref, num):
        """A GND via of its own next to a small bottom part's GND pin, reserved
        before the rest of its cluster settles round it: otherwise the pin can
        end up on a scrap of bottom pour boxed in by tracks."""
        pad = next(p for p in fps[ref].Pads() if p.GetNumber() == num)
        px, py = xy(pad.GetPosition())
        for rr_, a in ((r_, a_) for r_ in (0.9, 1.2, 1.5, 1.8, 2.2, 2.6, 3.0) for a_ in range(0, 360, 15)):
            gx, gy = px + rr_ * math.cos(math.radians(a)), py + rr_ * math.sin(math.radians(a))
            box = (gx - 0.55, gy - 0.55, gx + 0.55, gy + 0.55)
            # on top only real copper counts: the floor holds bounding boxes,
            # and the shunts above are 45 degree parts with big ones
            near = False
            for q in board.GetPads():
                if q.GetNetname() == "GND":
                    continue
                b = q.GetBoundingBox()
                x0, y0 = xy(pcbnew.VECTOR2I(b.GetLeft(), b.GetTop()))
                x1, y1 = xy(pcbnew.VECTOR2I(b.GetRight(), b.GetBottom()))
                if math.hypot(max(x0 - gx, 0, gx - x1), max(y0 - gy, 0, gy - y1)) < 0.6:
                    near = True
                    break
                # the stub on the bottom must clear the other pads too
                if q.IsOnLayer(pcbnew.B_Cu) and q is not pad:
                    for i in range(1, 11):
                        sx, sy = px + (gx - px) * i / 10, py + (gy - py) * i / 10
                        if math.hypot(max(x0 - sx, 0, sx - x1), max(y0 - sy, 0, sy - y1)) < 0.4:
                            near = True
                            break
                    if near:
                        break
            if not near and floor.free("bot", box, 0.1):
                add_via(board, nets["GND"], gx, gy, 0.66, 0.3)
                tr = pcbnew.PCB_TRACK(board)
                tr.SetStart(pad.GetPosition())
                tr.SetEnd(v(gx, gy))
                tr.SetWidth(MM(0.3))
                tr.SetLayer(pcbnew.B_Cu)
                tr.SetNet(nets["GND"])
                board.Add(tr)
                floor.add("bot", box, "via")
                return
        print("WARNING: no GND via for", ref, num)

    # battery leads and the electrolytic's legs are soldered on both sides:
    # keep a clear ring round the pads
    pad_r = math.hypot(L.BATT_PAD_L, L.BATT_PAD_W) / 2
    for x, y, _ in L.battery_pads().values():
        floor.circles.append((x, y, pad_r + 0.6))
    # FC connector, side entry, opening facing out of the front-left corner
    jx, jy, jang = L.j1_pose()
    place(fps["J1"], jx, jy, "+y", jang)
    floor.fix(fps["J1"])
    # current sense beside battery +: battery + pad -> "battery +" pour -> two
    # 0.2 mOhm shunts lying along the diagonal -> +BATT pour round the
    # mounting hole. The INA186 and its filter sit underneath on the bottom.
    diag = L.CORNER_ANGLE[L.BATT_CORNER]
    for ref, v_ in (("Rsense1", -3.8), ("Rsense2", -7.9)):
        x, y = L.corner_uv(30.6, v_, L.BATT_CORNER)
        # pin 1 (battery + side) outboard, pin 2 (+BATT) towards the hole
        place(fps[ref], x, y, "-x", diag)
        floor.fix(fps[ref])
    # (inboard, towards the mounting hole: further out it sits in the way
    # of the next channel's phase lines)
    cs = {"U12": (25.0, -5.0), "R89": (23.0, -3.5), "R90": (27.0, -3.0), "C40": (24.5, -7.5),
          "C41": (27.0, -6.0), "C42": (25.5, -2.5)}
    for ref, (u, v_) in cs.items():
        x, y = L.corner_uv(u, v_, L.BATT_CORNER)
        put_shared(ref, x, y, "+x", 0, True, step=0.1)
        if ref == "U12":
            gnd_via("U12", "2")
            # both GND pins on the one via
            p1, p2 = (next(p for p in fps["U12"].Pads() if p.GetNumber() == n) for n in ("1", "2"))
            tr = pcbnew.PCB_TRACK(board)
            tr.SetStart(p1.GetPosition())
            tr.SetEnd(p2.GetPosition())
            tr.SetWidth(MM(0.3))
            tr.SetLayer(pcbnew.B_Cu)
            tr.SetNet(nets["GND"])
            board.Add(tr)
    # the CURR output filter (R73 series, C94 to GND) at the FC end, under J1
    for ref, (u, v_) in {"R73": (30.0, -2.0), "C94": (30.0, 2.0)}.items():
        x, y = L.corner_uv(u, v_, L.J1_CORNER)
        put_shared(ref, x, y, "+x", 0, True, step=0.1)
    # supplies on the bottom of the rear-left corner, out of the way of the
    # MCU and driver vias in the middle, and away from the battery corner's
    # current-sense line: the channel between them would get both feeds. The
    # cluster was laid out for the front-right corner and is turned round.
    for ref, (x, y, ang) in {"U13": (-4.2, -5.0, 0), "U14": (-1.0, -5.6, 90), "C75": (-2.6, -7.8, 0),
                             "R120": (-7.0, -3.2, 90), "R121": (-7.0, -6.2, 90),
                             "C2": (-6.0, -9.6, 90), "C3": (-2.6, -10.2, 90), "C6": (0.8, -10.2, 90),
                             "C74": (-8.6, -9.0, 90), "C47": (1.6, -3.4, 0), "C76": (1.6, -1.6, 0),
                             "U15": (4.4, -4.4, 0), "C86": (1.6, -6.8, 90), "C4": (7.4, -3.0, 90),
                             "C51": (7.4, -6.4, 90), "C81": (4.4, -7.6, 0), "C93": (4.4, -1.4, 0)}.items():
        sx, sy = L.rot(x + 23.0, y - 13.0, 2)
        put_shared(ref, sx, sy, "+x", ang + 180, True)
        if ref == "U13":
            gnd_via("U13", "2")

    # ---- power copper
    out = L.outline()
    add_zone(board, nets["+BATT"], pcbnew.F_Cu, out, 0, name="BATT top")
    add_zone(board, nets["GND"], pcbnew.B_Cu, out, 0, name="GND bottom")
    if L.LAYERS == 4:
        add_zone(board, nets["GND"], pcbnew.In1_Cu, out, 0, name="GND plane")
        add_zone(board, nets["+BATT"], pcbnew.In2_Cu, out, 0, name="BATT plane")
    for ch, k in L.CHANNELS.items():
        for ph in "ABC":
            y = L.PHASE_Y[ph]
            n = nets[f"/{ch}/Motor{ph}"]
            top = phase_top_pts(fps[roles[ch][f"HS{ph}"]], y, k)
            bot = rect_pts(L.FET_X - 3.37 + 1.35, y - PHASE_HW, L.HALF + 1, y + PHASE_HW)
            add_zone(board, n, pcbnew.F_Cu, rot_pts(top, k), 2, name=f"{ch} {ph} top")
            add_zone(board, n, pcbnew.B_Cu, rot_pts(bot, k), 2, name=f"{ch} {ph} bottom")
    for pts, name in source_tabs:
        add_zone(board, nets["GND"], pcbnew.B_Cu, pts, 2, name=name)
    # battery + pad to the shunts' outboard pads
    uv = [(29.0, -2.4), (29.0, 4.6), (37.6, 4.6), (37.6, -10.0), (31.7, -10.0), (31.7, -2.4)]
    add_zone(board, nets["/CSA+"], pcbnew.F_Cu, [L.corner_uv(u, v_, L.BATT_CORNER) for u, v_ in uv], 3,
             name="battery +")
    # stitching round the battery entry: +BATT to the In2 plane round the
    # shunts' inboard pads, GND to the In1 plane round battery -
    for net, u0, u1, v0, v1 in (("+BATT", 23.0, 28.6, -9.6, 0.0), ("GND", 26.0, 36.0, 4.0, 16.0)):
        placed = 0
        u = u0
        while u <= u1:
            v_ = v0
            while v_ <= v1:
                gx, gy = L.corner_uv(u, v_, L.BATT_CORNER)
                rr = (gx - 0.6, gy - 0.6, gx + 0.6, gy + 0.6)
                ok = all(math.hypot(gx - hx, gy - hy) > L.HOLE_COPPER_FREE / 2 + 0.8 for hx, hy in L.holes())
                if ok and floor.free("top", rr, 0.15) and floor.free("bot", rr, 0.15):
                    add_via(board, nets[net], gx, gy)
                    floor.add("both", rr, "via")
                    placed += 1
                v_ += 1.6
            u += 1.6
        print(net, "battery stitching vias", placed)

    # silkscreen
    # in the empty front-right corner (the middle has the SWD test points)
    add_text(board, "drone-show ESC  4-6S  AM32", *L.corner_uv(29.0, 0.0, "TR"), 0.8, angle=45)
    for num, label in (("1", "BAT+"), ("2", "BAT-")):
        x, y = L.corner_uv(L.BATT_U - 5.3, L.BATT_V[num], L.BATT_CORNER)
        for layer in (pcbnew.F_SilkS, pcbnew.B_SilkS):
            add_text(board, label, x, y, 0.8, layer, -45)
    for ch, k in L.CHANNELS.items():
        x, y = L.rot(L.MOTOR_PAD_X - 4.4, -10.6, k)
        add_text(board, ch[-1], x, y, 1.4)

    board.Save(out_path)
    json.dump({ch: roles[ch] for ch in roles}, open(os.path.join(os.path.dirname(out_path), "roles.json"), "w"), indent=1)
    print("saved", out_path)


if __name__ == "__main__":
    build(sys.argv[1], sys.argv[2])
