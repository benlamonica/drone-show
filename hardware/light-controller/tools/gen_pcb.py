"""Build light-controller.kicad_pcb: outline, footprints, nets, zones, keep-outs.

Run with KiCad's Python (pcbnew). The nets come from the netlist KiCad exports from
the schematic, so the board matches the schematic exactly. Placement is the PLACE
table below. Routing is done afterwards by route.py.
"""
import json
import math
import os
import re
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import design  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
NAME = "light-controller"
PCB = os.path.join(ROOT, NAME + ".kicad_pcb")
LIBDIR = os.path.join(ROOT, "lightctl.pretty")
mm = pcbnew.FromMM

# PCBWay standard 2-layer process (no extra cost options): 6/6 mil, 0.3 mm drill.
RULES = {
    "min_clearance": 0.15, "min_track_width": 0.15, "min_connection": 0.15,
    "min_via_diameter": 0.6, "min_through_hole_diameter": 0.3, "min_via_annular_width": 0.15,
    "min_hole_to_hole": 0.5, "min_hole_clearance": 0.25, "min_copper_edge_clearance": 0.3,
    "min_silk_clearance": 0.0, "min_text_height": 0.8, "min_text_thickness": 0.15,
    "min_microvia_diameter": 0.6, "min_microvia_drill": 0.3, "max_error": 0.005,
    "min_resolved_spokes": 2, "solder_mask_to_copper_clearance": 0.0,
    "use_height_for_length_calcs": True, "min_groove_width": 0.0,
}

NETCLASSES = [
    # name, track, clearance, via, drill, patterns
    ("Default", 0.15, 0.15, 0.6, 0.3, []),
    ("Power", 0.5, 0.15, 0.8, 0.4,
     ["/VIN", "/VBAT_IN", "/SW_*", "/LEDA_*", "/LEDK_*", "/R1", "/R2", "/G1", "/G2", "/B1", "/B2",
      "/W1", "/W2", "/BUCK_SW"]),
    ("Supply", 0.25, 0.15, 0.6, 0.3, ["/+3V3", "/+1V1", "/VREG_LX"]),
    ("GND", 0.25, 0.15, 0.6, 0.3, ["/GND"]),
]

# Anchors: ref -> (x, y, rotation, side). KiCad axes: +x toward the open end, +y toward
# one long edge. Top-side parts stay 2 mm inside the long edges and the stop end.
# Everything not listed here is a small part placed by legalize() next to the pad
# in NEAR (or, by default, next to the anchor pad it shares its most specific net with).
PLACE = {
    "M1": (-26.0, 0.0, 90, "F"),          # RM2, antenna toward the stop end
    "U2": (-9.0, 0.0, 180, "F"),          # RP2350A: GPIO0-15 face the drivers, GPIO23-29 the RM2
    "Y1": (-8.4, -8.6, 0, "F"),
    "U3": (-7.4, 9.8, 90, "B"),           # QSPI flash, under the MCU
    "L2": (-12.6, 6.4, 0, "F"),
    "J3": (-12.0, -12.4, 180, "B"),       # SWD
    "U1": (-25.0, 11.0, 0, "B"), "L1": (-19.6, 11.4, 0, "B"),     # 3.3 V buck, bottom, under the RM2
    "J1": (33.0, -12.8, 270, "F"), "Q1": (27.0, -11.6, 0, "F"), "D2": (27.2, -5.4, 0, "F"),
    "J4": (31.3, 12.0, 90, "F"),
    "TP1": (-17.6, -13.4, 0, "B"), "TP2": (-20.2, -13.4, 0, "B"), "TP3": (-22.8, -13.4, 0, "B"),
    "TP4": (-25.4, -13.4, 0, "B"),
}
NEAR = {
    "C11": ("U2", "1"), "C12": ("U2", "11"), "C13": ("U2", "20"), "C14": ("U2", "30"),
    "C15": ("U2", "38"), "C16": ("U2", "45"), "C17": ("U2", "6"), "C18": ("U2", "23"),
    "C19": ("U2", "39"), "C20": ("U2", "50"), "C21": ("U2", "49"), "C22": ("U2", "46"),
    "R7": ("U2", "46"), "C23": ("U2", "44"), "C24": ("U2", "53"), "C25": ("U2", "54"),
    "C26": ("U2", "11"), "C29": ("U3", "8"),
    "C31": ("M1", "16"), "C32": ("M1", "14"), "C33": ("M1", "16"),
    "R18": ("M1", "6"), "R20": ("M1", "10"), "R19": ("M1", "8"), "D4": ("M1", "8"),
    "C9": ("L1", "2"), "C10": ("L1", "2"), "C6": ("U1", "3"), "C7": ("U1", "3"),
    "R4": ("U1", "5"), "C8": ("U1", "6"), "R5": ("U1", "4"), "R6": ("U1", "4"),
    "C1": ("Q1", "3"), "C2": ("Q1", "3"), "C3": ("D2", "1"), "C4": ("Q1", "3"),
    "D1": ("Q1", "1"), "R1": ("Q1", "1"),
    "R2": ("U2", "41"), "R3": ("U2", "41"), "C5": ("U2", "41"),
    "R22": ("U2", "40"), "C34": ("U2", "40"), "TH1": ("U2", "40"),
    "R11": ("U2", "26"), "R21": ("U2", "9"), "D5": ("U2", "9"),
}

# Decoupling for the MCU goes on the bottom, directly under the QFN, so the top-side ring
# around it is free for the pin escapes.
BOTTOM = {"C11", "C12", "C13", "C14", "C15", "C16", "C17", "C18", "C19", "C21", "C23", "C24",
          "C25", "C26", "R11", "R2", "R3", "C5", "R22", "C34", "TH1",
          "R18", "R19", "R20", "D4", "C32", "C33"}

# LED driver cells: two columns x two rows between the MCU and the power entry.
CELLS = {"R": (4.6, -1), "G": (4.6, 1), "B": (16.2, -1), "W": (16.2, 1)}
# LEDs on the bottom: a 4 x 3 array under the drivers, one row per colour, so each
# string is three LEDs in a straight line (anode on the left) and no string crosses
# another. The gaps between rows carry the anode feeds.
LED_COLS = [5.0, 13.0, 21.0]
LED_ROWS = {"R": -8.4, "B": -2.8, "G": 2.8, "W": 8.4}
LED_ROT = 180


def cell_parts():
    for ch, c in enumerate("RGBW"):
        n = ch + 1
        cx, s = CELLS[c]                  # s = -1: top row (y < 0); +1: mirrored bottom row
        PLACE[f"L{2 + n}"] = (cx + 2.4, s * 11.6, 0, "F")
        PLACE[f"U{4 + n}"] = (cx - 3.2, s * 11.4, 90 if s < 0 else 270, "F")
        PLACE[f"D{5 + n}"] = (cx, s * 5.8, 0, "F")
        PLACE[f"R{30 + n}"] = (cx - 2.8, s * 2.4, 0, "F")
        PLACE[f"C{40 + n}"] = (cx + 2.8, s * 2.4, 0, "F")
        NEAR[f"R{40 + n}"] = (f"U{4 + n}", "4")
        NEAR[f"R{50 + n}"] = (f"U{4 + n}", "4")
        NEAR[f"C{50 + n}"] = (f"U{4 + n}", "4")
        for k, x0 in enumerate(LED_COLS):
            PLACE[f"LED{ch * 3 + k + 1}"] = (x0, LED_ROWS[c], LED_ROT, "B")


def write_project():
    pro = {
        "board": {"design_settings": {
            "defaults": {"board_outline_line_width": 0.1, "copper_line_width": 0.2, "copper_text_size_h": 1.0,
                         "copper_text_size_v": 1.0, "copper_text_thickness": 0.15, "silk_line_width": 0.15,
                         "silk_text_size_h": 1.0, "silk_text_size_v": 1.0, "silk_text_thickness": 0.15,
                         "pads": {"drill": 0.8, "height": 1.6, "width": 1.6}, "zones": {"min_clearance": 0.25}},
            "diff_pair_dimensions": [], "drc_exclusions": [],
            "meta": {"version": 2},
            "rule_severities": {"silk_overlap": "ignore", "silk_over_copper": "warning",
                                "lib_footprint_issues": "ignore", "lib_footprint_mismatch": "ignore"},
            "rules": RULES,
            "track_widths": [0.0, 0.2, 0.4, 0.6, 1.0],
            "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3},
                               {"diameter": 0.8, "drill": 0.4}],
            "zones_allow_external_fillets": False}},
        "boards": [],
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {"filename": NAME + ".kicad_pro", "version": 1},
        "net_settings": {
            "classes": [{"name": n, "track_width": t, "clearance": c, "via_diameter": v, "via_drill": d,
                         "bus_width": 12, "wire_width": 6, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25,
                         "diff_pair_width": 0.2, "line_style": 0, "microvia_diameter": 0.6,
                         "microvia_drill": 0.3, "pcb_color": "rgba(0, 0, 0, 0.000)",
                         "schematic_color": "rgba(0, 0, 0, 0.000)"} for n, t, c, v, d, _ in NETCLASSES],
            "meta": {"version": 3},
            "net_colors": None, "netclass_assignments": None,
            "netclass_patterns": [{"netclass": n, "pattern": p} for n, *_, ps in NETCLASSES for p in ps]},
        "pcbnew": {"page_layout_descr_file": ""},
        "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [],
        "text_variables": {},
    }
    json.dump(pro, open(os.path.join(ROOT, NAME + ".kicad_pro"), "w"), indent=2)


def netlist():
    net = os.path.join("/tmp", NAME + ".net")
    subprocess.run(["kicad-cli", "sch", "export", "netlist", "--format", "kicadsexpr", "-o", net,
                    os.path.join(ROOT, NAME + ".kicad_sch")], check=True, capture_output=True)
    t = open(net).read()
    pads, comps = {}, {}
    for m in re.finditer(r'\(net \(code "?\d+"?\) \(name "([^"]*)"\)(.*?)(?=\(net \(code|\Z)', t, re.S):
        for ref, pin in re.findall(r'\(node \(ref "([^"]+)"\) \(pin "([^"]+)"\)', m.group(2)):
            pads[(ref, pin)] = m.group(1)
    for m in re.finditer(r'\(comp \(ref "([^"]+)"\).*?\(tstamps "([^"]+)"\)', t, re.S):
        comps[m.group(1)] = m.group(2)
    return pads, comps


def line(board, a, b, layer=pcbnew.Edge_Cuts, w=0.1):
    s = pcbnew.PCB_SHAPE(board)
    s.SetShape(pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(pcbnew.VECTOR2I(mm(a[0]), mm(a[1])))
    s.SetEnd(pcbnew.VECTOR2I(mm(b[0]), mm(b[1])))
    s.SetLayer(layer)
    s.SetWidth(mm(w))
    board.Add(s)


def rect_poly(x0, y0, x1, y1):
    ch = pcbnew.SHAPE_LINE_CHAIN()
    for x, y in ((x0, y0), (x1, y0), (x1, y1), (x0, y1)):
        ch.Append(mm(x), mm(y))
    ch.SetClosed(True)
    return ch


def zone(board, net, layer, x0, y0, x1, y1, name, keepout=False, prio=0):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    if keepout:
        z.SetIsRuleArea(True)
        z.SetDoNotAllowCopperPour(True)
        z.SetDoNotAllowTracks(True)
        z.SetDoNotAllowVias(True)
        z.SetDoNotAllowPads(True)
        z.SetDoNotAllowFootprints(False)
        ls = pcbnew.LSET()
        ls.AddLayer(pcbnew.F_Cu)
        ls.AddLayer(pcbnew.B_Cu)
        z.SetLayerSet(ls)
    else:
        z.SetNet(net)
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetThermalReliefGap(mm(0.3))
        z.SetThermalReliefSpokeWidth(mm(0.4))
        z.SetLocalClearance(mm(0.25))
        z.SetMinThickness(mm(0.25))
        z.SetAssignedPriority(prio)
    z.SetZoneName(name)
    z.Outline().AddOutline(rect_poly(x0, y0, x1, y1))
    board.Add(z)
    return z


def sexp(text):
    tokens = re.findall(r'\(|\)|"(?:[^"\\]|\\.)*"|[^\s()]+', text)
    stack = [[]]
    for t in tokens:
        if t == "(":
            stack.append([])
        elif t == ")":
            last = stack.pop()
            stack[-1].append(last)
        else:
            stack[-1].append(t.strip('"'))
    return stack[0][0]


def extents(fp):
    """Local (x0, y0, x1, y1) of courtyard and pads, footprint at the origin unrotated."""
    tree = sexp(open(os.path.join(LIBDIR, fp.GetFPID().GetLibItemName().wx_str() + ".kicad_mod")).read())
    xs, ys = [], []
    for item in tree:
        if not isinstance(item, list) or item[0] not in ("fp_line", "fp_rect", "fp_circle", "fp_poly", "fp_arc"):
            continue
        d = {k[0]: k[1:] for k in item[1:] if isinstance(k, list)}
        if "layer" not in d or not d["layer"][0].endswith("CrtYd"):
            continue
        if item[0] == "fp_circle":
            cx, cy = map(float, d["center"])
            ex, ey = map(float, d["end"])
            r = math.hypot(ex - cx, ey - cy)
            xs += [cx - r, cx + r]
            ys += [cy - r, cy + r]
        elif item[0] == "fp_poly":
            for pt in d["pts"]:
                xs.append(float(pt[1]))
                ys.append(float(pt[2]))
        else:
            for k in ("start", "end", "mid"):
                if k in d:
                    xs.append(float(d[k][0]))
                    ys.append(float(d[k][1]))
    for pad in fp.Pads():
        bb = pad.GetBoundingBox()
        xs += [pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetRight())]
        ys += [pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetBottom())]
    return min(xs), min(ys), max(xs), max(ys)


def rot_box(e, rot):
    x0, y0, x1, y1 = e
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    a = math.radians(rot)
    r = [(x * math.cos(a) + y * math.sin(a), -x * math.sin(a) + y * math.cos(a)) for x, y in pts]
    return min(p[0] for p in r), min(p[1] for p in r), max(p[0] for p in r), max(p[1] for p in r)


GAP = 0.35       # courtyard-to-courtyard spacing for placed small parts
ANT_X = PLACE["M1"][0] - 3.3     # inner edge of the antenna keep-out


def legalize(fps, pads):
    hx, hy = design.BOARD_L / 2, design.BOARD_W / 2
    loc = {}
    for ref, fp in fps.items():
        fp.SetPosition(pcbnew.VECTOR2I(0, 0))
        fp.SetOrientationDegrees(0)
        loc[ref] = extents(fp)
    occ = []                      # (side, box)

    def put(ref, x, y, rot, side):
        fp = fps[ref]
        fp.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        fp.SetOrientationDegrees(rot)
        if side == "B":
            fp.Flip(fp.GetPosition(), False)
        b = rot_box(loc[ref], rot)
        if side == "B":
            b = (b[0], -b[3], b[2], -b[1])
        occ.append((side, (x + b[0], y + b[1], x + b[2], y + b[3]), ref))

    for ref, (x, y, rot, side) in PLACE.items():
        put(ref, x, y, rot, side)
    # via fields that build() adds: under the MCU pad and beside each LED
    ux = PLACE["U2"][0]
    occ.append(("B", (ux - 1.6, -1.6, ux + 1.6, 1.6), "EP vias"))
    # the LED array and its lanes: nothing else on the bottom there
    ys = LED_ROWS.values()
    occ.append(("B", (LED_COLS[0] - 3.0, min(ys) - 2.2, LED_COLS[-1] + 3.0, max(ys) + 2.2), "LED array"))

    def free(box, side):
        x0, y0, x1, y1 = box
        if x0 < ANT_X or x1 > hx - 0.3 or y0 < -hy + 2.0 or y1 > hy - 2.0:
            return False
        for s, (a0, b0, a1, b1), _ in occ:
            if s == side and x0 < a1 + GAP and x1 > a0 - GAP and y0 < b1 + GAP and y1 > b0 - GAP:
                return False
        return True

    def pad_pos(ref, num):
        for pad in fps[ref].Pads():
            if pad.GetNumber() == num:
                v = pad.GetPosition()
                return pcbnew.ToMM(v.x), pcbnew.ToMM(v.y)
        sys.exit(f"no pad {ref}.{num}")

    placed = set(PLACE)
    side_of = {r: v[3] for r, v in PLACE.items()}
    todo = [p["ref"] for p in design.PARTS if p["ref"] not in PLACE]
    todo.sort(key=lambda r: (r not in NEAR, r))
    by_net = {}
    for (ref, num), n in pads.items():
        by_net.setdefault(n, []).append((ref, num))
    while todo:
        ref = next((r for r in todo if r in NEAR or any(
            o in placed for (r2, _), n in pads.items() if r2 == r and n != "/GND"
            for o, _ in by_net[n] if o != r)), None)
        if ref is None:
            sys.exit(f"cannot anchor {todo}")
        todo.remove(ref)
        side = "F"
        if ref in NEAR:
            h = NEAR[ref]
            tx, ty = h if isinstance(h[0], float) else pad_pos(*h)
            if not isinstance(h[0], float):
                side = side_of[h[0]]
        else:
            cands = []
            for (r, num), n in pads.items():
                if r != ref or n in ("/GND",):
                    continue
                others = [(o, on) for o, on in by_net[n] if o != ref and o in placed]
                if others:
                    cands.append((len(by_net[n]), others[0]))
            if not cands:
                sys.exit(f"{ref}: nothing placed to put it next to")
            nb = min(cands)[1]
            tx, ty = pad_pos(*nb)
            side = side_of[nb[0]]
        if ref in BOTTOM:
            side = "B"
        best = None
        r = 0.0
        while best is None and r < 20:
            n = max(1, int(2 * math.pi * r / 0.3))
            for i in range(n):
                a = 2 * math.pi * i / n
                x, y = round(tx + r * math.cos(a), 2), round(ty + r * math.sin(a), 2)
                for rot in (0, 90):
                    b = rot_box(loc[ref], rot)
                    if side == "B":
                        b = (b[0], -b[3], b[2], -b[1])
                    if free((x + b[0], y + b[1], x + b[2], y + b[3]), side):
                        best = (x, y, rot)
                        break
                if best:
                    break
            r += 0.25
        if best is None:
            sys.exit(f"no room for {ref}")
        put(ref, *best, side)
        placed.add(ref)
        side_of[ref] = side


def build():
    cell_parts()
    write_project()
    pads, comps = netlist()
    board = pcbnew.NewBoard(PCB)
    board.SetCopperLayerCount(2)
    ds = board.GetDesignSettings()
    ds.SetBoardThickness(mm(1.6))

    hx, hy = design.BOARD_L / 2, design.BOARD_W / 2
    for a, b in (((-hx, -hy), (hx, -hy)), ((hx, -hy), (hx, hy)), ((hx, hy), (-hx, hy)), ((-hx, hy), (-hx, -hy))):
        line(board, a, b)
    # rail ledges and the stop end: no parts there (drawn on User.1 for reference)
    for y in (-hy + 2, hy - 2):
        line(board, (-hx, y), (hx, y), pcbnew.User_1, 0.05)
    line(board, (-hx + 2, -hy), (-hx + 2, hy), pcbnew.User_1, 0.05)

    nets = {}
    for n in sorted(set(pads.values())):
        ni = pcbnew.NETINFO_ITEM(board, n)
        board.Add(ni)
        nets[n] = ni

    fps = {}
    for p in design.PARTS:
        fp = pcbnew.FootprintLoad(LIBDIR, p["fp"])
        if fp is None:
            sys.exit(f"cannot load {p['fp']}")
        fp.SetReference(p["ref"])
        fp.SetValue(p["value"])
        fp.SetFPIDAsString(f"{design.LIB}:{p['fp']}")
        fp.SetPath(pcbnew.KIID_PATH(f"/{comps[p['ref']]}"))
        if p["dnp"]:
            fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_POS_FILES | pcbnew.FP_EXCLUDE_FROM_BOM)
        for k, v in (("MPN", p["mpn"]), ("Manufacturer", p["mfr"])):
            fp.SetProperty(k, v)
        board.Add(fp)
        for pad in fp.Pads():
            n = pads.get((p["ref"], pad.GetNumber()))
            if n:
                pad.SetNet(nets[n])
            if p["sym"] == "LED_XPE2" and pad.GetNumber() == "3":
                pad.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)
        fp.Reference().SetTextSize(pcbnew.VECTOR2I(mm(0.8), mm(0.8)))
        fp.Reference().SetTextThickness(mm(0.15))
        fps[p["ref"]] = fp
    legalize(fps, pads)

    gnd = nets["/GND"]
    # GND pours on both layers; LEDs' thermal pads connect solid.
    zone(board, gnd, pcbnew.F_Cu, -hx + 0.2, -hy + 0.2, hx - 0.2, hy - 0.2, "GND top")
    zone(board, gnd, pcbnew.B_Cu, -hx + 0.2, -hy + 0.2, hx - 0.2, hy - 0.2, "GND bottom")
    # Antenna end: no copper on either layer from the stop end to the antenna's inner edge.
    m1 = PLACE["M1"]
    zone(board, None, pcbnew.F_Cu, -hx - 0.1, -hy - 0.1, m1[0] - 3.3, hy + 0.1, "Antenna keep-out", keepout=True)

    # thermal vias beside each LED's thermal pad, and GND stitching
    padboxes = []
    for fp in board.GetFootprints():
        for pd in fp.Pads():
            bb = pd.GetBoundingBox()
            padboxes.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()),
                             pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()), pd.GetNetname()))

    def via(x, y, net, d=0.6, drill=0.3):
        m = d / 2 + 0.3
        for x0, y0, x1, y1, n in padboxes:
            if x0 - m < x < x1 + m and y0 - m < y < y1 + m and n != net.GetNetname():
                return
        v = pcbnew.PCB_VIA(board)
        v.SetPosition(pcbnew.VECTOR2I(mm(x), mm(y)))
        v.SetWidth(mm(d))
        v.SetDrill(mm(drill))
        v.SetNet(net)
        board.Add(v)

    # thermal vias at the four corners of each LED, clear of the string running through
    for y in LED_ROWS.values():
        for x in LED_COLS:
            for vx in (-2.4, 2.4):
                for vy in (-1.0, 1.0):
                    via(x + vx, y + vy, gnd)
    # MCU exposed pad: 3 x 3 vias
    ux, uy = PLACE["U2"][:2]
    for i in (-1, 0, 1):
        for j in (-1, 0, 1):
            via(ux + i * 1.0, uy + j * 1.0, gnd)

    pcbnew.SaveBoard(PCB, board)
    print("saved", PCB, len(design.PARTS), "footprints", len(nets), "nets")


if __name__ == "__main__":
    build()
