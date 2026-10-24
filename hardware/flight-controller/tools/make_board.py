"""Build show-fc.kicad_pcb from design.py: outline, rules, footprints, nets,
placement. Routing is a separate step (route.py) so placement can be tuned
without re-routing by hand.

Run with the Python that ships with KiCad (it must import pcbnew).
"""

import math
import os
import sys

import pcbnew

import design
import make_schematic as ms

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "show-fc.kicad_pcb")
STD = "/usr/share/kicad/footprints"
if not os.path.isdir(STD):
    STD = os.environ.get("KICAD7_FOOTPRINT_DIR", STD)

# ---------------------------------------------------------------------------
# Board: 50 x 50 mm, corners cut so no corner reaches past 27 mm from the
# centre along the frame's arm axes (the hub's arm-socket fins start at
# ~27.7 mm). Stack holes on the 30.5 mm pattern.

CX, CY = 100.0, 100.0
HALF = 27.0
CORNER_REACH = 27.0
HOLE = 15.25
CUT = CORNER_REACH * math.sqrt(2)          # x + y = CUT on the chamfer
MM = pcbnew.FromMM


def P(x, y):
    return pcbnew.VECTOR2I(MM(CX + x), MM(CY + y))


def outline_pts():
    c = CUT - HALF
    return [(-c, -HALF), (c, -HALF), (HALF, -c), (HALF, c), (c, HALF), (-c, HALF), (-HALF, c), (-HALF, -c)]


def inside(x, y, margin=0.0):
    if abs(x) > HALF - margin or abs(y) > HALF - margin:
        return False
    return abs(x) + abs(y) <= CUT - margin * math.sqrt(2)


# ---------------------------------------------------------------------------
# Rules. PCBWay standard 2-layer service (no surcharge): 6/6 mil track and
# space, 0.3 mm minimum drill, 0.15 mm annular ring.

RULES = dict(
    min_clearance=0.15, min_track=0.15, min_via_dia=0.6, min_via_drill=0.3,
    min_through_hole=0.3, min_hole_to_hole=0.4, min_edge_clearance=0.3,
    min_annular=0.15, silk_min_width=0.15, silk_min_text=0.8,
)
NETCLASSES = {
    # name: (track, clearance, via dia, via drill, nets)
    "Default": (0.2, 0.15, 0.6, 0.3, []),
    "Power": (0.25, 0.15, 0.6, 0.3, ["GND", "+3V3", "+1V1", "+4V5", "+1V8_IMU", "VREG_AVDD", "VBUS", "VREG_LX"]),
    "Supply": (0.5, 0.2, 0.8, 0.4, ["VBAT", "+5V", "BUCK_SW"]),
    "USB": (0.25, 0.15, 0.6, 0.3, ["USB_DP", "USB_DM", "USB_DP_C", "USB_DM_C"]),
}

# ---------------------------------------------------------------------------
# Placement. (x, y, rotation, side) relative to the board centre. Rotation
# follows KiCad (counter-clockwise). Connectors face out of the nearest edge.

FIXED = {
    "U1": (0, 0, 0),
    "J1": (4.0, -23.9, 180),       # USB-C, top edge
    "J6": (-8.0, -22.6, 180),      # RC, top edge
    "J2": (-20.4, -3.8, 270),      # microSD, card goes in from the left edge
    "J4": (-6.8, 22.6, 0),         # GPS, bottom edge
    "J3": (5.3, 23.7, 0),          # ESC, bottom edge
    "J5": (23.8, 2.6, 90),         # LIGHT, right edge
    "J8": (23.4, -8.2, 90),        # SWD, right edge
    "H1": (-HOLE, -HOLE, 0), "H2": (HOLE, -HOLE, 0), "H3": (HOLE, HOLE, 0), "H4": (-HOLE, HOLE, 0),
    # sensors, bottom-left; IMU1 nearest the board centre
    "U6": (-8.0, 7.0, 0),
    "U7": (-8.0, 12.5, 0),
    "U5": (-12.6, 7.0, 90),
    "U8": (-7.0, -14.0, 0),
    # clock and core regulator
    "Y1": (0.5, 9.0, 0),
    "L1": (3.4, -8.4, 90),
    # 5 V buck, bottom-right, next to where VBAT comes in
    "U3": (9.4, 9.0, 0),
    "L2": (13.6, 8.6, 0),
    "D3": (8.6, 15.8, 0),
    # USB / 5 V diode-OR and the 3.3 V LDO, top-right
    "U2": (0.6, -13.6, 0),
    "D5": (8.4, -15.2, 0),
    "D4": (11.4, -9.6, 90),
    "U4": (8.4, -11.4, 0),
    "SW1": (12.0, -22.2, 0),
    "SW2": (-2.8, 13.8, 0),
}

# Small parts go next to the pad they serve. ref: (anchor ref, anchor pad)
ANCHOR = {
    "C1": ("U1", "5"), "C2": ("U1", "15"), "C3": ("U1", "24"), "C4": ("U1", "29"),
    "C5": ("U1", "41"), "C6": ("U1", "50"), "C7": ("U1", "60"), "C8": ("U1", "76"),
    "C9": ("U1", "69"), "C10": ("U1", "68"), "C11": ("U1", "59"),
    "C12": ("U1", "64"), "C13": ("L1", "2"), "R1": ("U1", "61"), "C14": ("U1", "61"),
    "C15": ("U1", "10"), "C16": ("U1", "32"), "C17": ("U1", "51"), "C18": ("U1", "64"),
    "R2": ("Y1", "3"), "C19": ("Y1", "1"), "C20": ("Y1", "3"),
    "R3": ("SW1", "1"), "R4": ("SW2", "1"), "C21": ("SW2", "1"),
    "D1": ("SW2", "2"), "R5": ("SW2", "2"), "D2": ("SW2", "2"), "R6": ("SW2", "2"),
    "R7": ("J1", "A5"), "R8": ("J1", "B5"), "R9": ("U2", "1"), "R10": ("U2", "3"),
    "C22": ("U3", "3"), "C23": ("U3", "3"), "C24": ("U3", "3"), "R11": ("U3", "2"), "R12": ("U3", "2"),
    "C25": ("U3", "6"), "R13": ("U3", "1"), "R14": ("U3", "1"), "C26": ("L2", "2"), "C27": ("L2", "2"),
    "C28": ("L2", "2"),
    "C29": ("D4", "1"), "C30": ("U4", "1"), "C31": ("U5", "1"), "C32": ("U4", "6"),
    "D6": ("D4", "1"), "R15": ("D4", "1"),
    "R16": ("U1", "49"), "R17": ("U1", "49"), "C33": ("U1", "49"),
    "R18": ("U1", "52"), "C34": ("U1", "52"),
    "C37": ("U6", "5"), "C38": ("U6", "8"), "C39": ("U7", "5"), "C40": ("U7", "8"),
    "R22": ("U6", "12"), "R23": ("U7", "12"),
    "C41": ("U8", "8"), "C42": ("U8", "6"), "R24": ("U8", "3"), "R25": ("U8", "4"),
    "R26": ("J2", "2"), "R27": ("J2", "7"),
    "C43": ("J2", "4"), "C44": ("J2", "4"),
    "R31": ("J5", "6"), "R32": ("J4", "5"), "R33": ("J4", "4"),
    "Q1": ("TP6", "1"), "R34": ("Q1", "1"), "R35": ("Q1", "1"), "D7": ("Q1", "3"),
}
# Test pads, by hand
PADS = {
    "TP1": (13.6, 20.9), "TP2": (12.6, 23.6),        # VBAT, GND
    "TP3": (-1.3, 16.9), "TP4": (1.3, 16.9),         # +5V, GND
    "TP5": (-22.0, 9.8), "TP6": (-22.0, 12.4),       # buzzer +5V, BUZ-
    "TP7": (22.1, 11.2), "TP8": (24.7, 11.2),       # AUX5, AUX6
    "TP9": (-10.2, 18.0),                            # GPS_PPS
    "TP10": (-3.0, -13.6), "TP11": (-4.1, -16.6),    # +3V3, GND
    "TP12": (-23.9, -12.0), "TP13": (-21.3, -12.0),      # TELEM2 TX, RX
}
# Each small part stays inside its block. (x0, y0, x1, y1), board-centre mm
TR = (6.4, -18.8, 18.4, -5.0)
TC = (-6.6, -17.2, 6.4, -6.4)
TL = (-13.6, -18.8, -4.0, -10.6)
L_ = (-11.9, -10.8, -6.4, 4.6)
BL = (-14.6, 4.4, -4.0, 18.6)
BC = (-4.4, 6.4, 6.6, 18.6)
R_ = (6.4, -5.2, 18.4, 5.4)
LB = (-19.4, 3.4, -12.4, 13.0)
BR = (6.4, 5.2, 18.6, 18.6)
ZONES = {
    "a": ([TR, R_], "D6 R15 C29 C30 C32 R3 R8"),
    "buz": ([LB], "Q1 R34 R35"),
    "buz2": ([LB, BL, TL], "D7"),
    "r31": ([R_, BR, TR], "R31"),
    "b": ([TC], "C12 C13 C14 R1 C18 C8 C9 C10 R9 R10 R7"),
    "c": ([TL], "C41 C42 R24 R25"),
    "d": ([L_, TL], "C1 C2 C15 R26 R27 C43 C44"),
    "e": ([BL], "C31 C37 C38 C39 C40 R22 R23"),
    "i2c0": ([BL, LB, BC, TL, L_, TC, R_, TR], "R32 R33"),
    "f": ([BC, BL], "C3 C4 C16 R2 C19 C20 R4 C21 D1 D2 R5 R6"),
    "g": ([R_], "C5 C6 C7 C11 C17 R16 R17 C33 R18 C34"),
    "h": ([BR, R_, BC, TR], "C22 C23 C24 C25 R11 R12 R13 R14 C26 C27 C28"),
}
ZONE_OF = {r: z for _, (z, refs) in ZONES.items() for r in refs.split()}

# Order for the automatic placement: tightest constraints first
ORDER = ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9", "C10", "C11", "C15", "C16", "C17",
         "C12", "C13", "C14", "R1", "C18", "R2", "C19", "C20", "C38", "C37", "C40", "C39", "C31",
         "R22", "R23", "C41", "C42", "R24", "R25", "R9", "R10", "R7", "R8",
         "R16", "R17", "C33", "R18", "C34",
         "C22", "C23", "C24", "C25", "R11", "R12", "R13", "R14", "C26", "C27", "C28",
         "C29", "C30", "C32", "D6", "R15",
         "R26", "R27", "C43", "C44", "R3", "R4", "C21", "D1", "R5", "D2", "R6",
         "Q1", "R34", "R35", "D7", "R31", "R32", "R33"]


def load_fp(libid):
    lib, name = libid.split(":")
    path = os.path.join(ROOT, "showfc.pretty") if lib == "showfc" else os.path.join(STD, lib + ".pretty")
    fp = pcbnew.FootprintLoad(path, name)
    if fp is None:
        sys.exit(f"cannot load footprint {libid}")
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    # stock thermal vias are 0.2 mm; PCBWay's standard minimum drill is 0.3 mm
    for pad in fp.Pads():
        d = pad.GetDrillSize()
        if 0 < d.x < pcbnew.FromMM(0.3):
            pad.SetDrillSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.3), pcbnew.FromMM(0.3)))
            pad.SetSize(pcbnew.VECTOR2I(pcbnew.FromMM(0.6), pcbnew.FromMM(0.6)))
    return fp


def courtyard(fp, margin=0.2):
    xs, ys = [], []
    for g in fp.GraphicalItems():
        if g.GetLayer() == pcbnew.F_CrtYd:
            b = g.GetBoundingBox()
            xs += [b.GetLeft(), b.GetRight()]
            ys += [b.GetTop(), b.GetBottom()]
    if not xs:
        for pad in fp.Pads():
            b = pad.GetBoundingBox()
            xs += [b.GetLeft(), b.GetRight()]
            ys += [b.GetTop(), b.GetBottom()]
    return (pcbnew.ToMM(min(xs)) - CX - margin, pcbnew.ToMM(min(ys)) - CY - margin,
            pcbnew.ToMM(max(xs)) - CX + margin, pcbnew.ToMM(max(ys)) - CY + margin)


def overlap(a, b):
    return a[0] < b[2] and b[0] < a[2] and a[1] < b[3] and b[1] < a[3]


def setup_rules(board):
    ds = board.GetDesignSettings()
    ds.m_MinClearance = MM(RULES["min_clearance"])
    ds.m_TrackMinWidth = MM(RULES["min_track"])
    ds.m_ViasMinSize = MM(RULES["min_via_dia"])
    ds.m_MinThroughDrill = MM(RULES["min_through_hole"])
    ds.m_HoleToHoleMin = MM(RULES["min_hole_to_hole"])
    ds.m_CopperEdgeClearance = MM(RULES["min_edge_clearance"])
    ds.m_ViasMinAnnularWidth = MM(RULES["min_annular"])
    ds.m_MinSilkTextHeight = MM(RULES["silk_min_text"])
    ds.m_MinSilkTextThickness = MM(RULES["silk_min_width"])
    ds.m_SilkClearance = MM(0.0)
    ds.m_SolderMaskExpansion = MM(0.0)
    ds.m_SolderMaskMinWidth = MM(0.1)
    ds.SetBoardThickness(MM(1.6))
    ds.m_HoleClearance = MM(0.15)
    # Net classes live in show-fc.kicad_pro (written by make_project.py).
    # pre-defined track widths and vias offered in the editor
    ds.m_TrackWidthList.clear()
    for w in (0.0, 0.2, 0.3, 0.5, 0.8):
        ds.m_TrackWidthList.append(MM(w))


def build():
    board = pcbnew.BOARD()
    board.SetCopperLayerCount(2)
    setup_rules(board)

    pts = outline_pts()
    for a, b in zip(pts, pts[1:] + pts[:1]):
        seg = pcbnew.PCB_SHAPE(board)
        seg.SetShape(pcbnew.SHAPE_T_SEGMENT)
        seg.SetStart(P(*a))
        seg.SetEnd(P(*b))
        seg.SetLayer(pcbnew.Edge_Cuts)
        seg.SetWidth(MM(0.1))
        board.Add(seg)

    root, sym_uuid = ms.uid("root"), {p.ref: ms.uid("sym", p.ref) for p in design.PARTS}
    nets = {}
    for name in design.nets():
        ni = pcbnew.NETINFO_ITEM(board, name)
        board.Add(ni)
        nets[name] = ni

    fps = {}
    for p in design.PARTS:
        fp = load_fp(p.footprint)
        fp.SetReference(p.ref)
        fp.SetValue(p.value)
        fp.SetPath(pcbnew.KIID_PATH(f"/{sym_uuid[p.ref]}"))
        setf = fp.SetField if hasattr(fp, "SetField") else fp.SetProperty
        if hasattr(fp, "SetSheetname"):
            fp.SetSheetname("Root")
            fp.SetSheetfile("show-fc.kicad_sch")
        else:
            setf("Sheetname", "")
            setf("Sheetfile", "show-fc.kicad_sch")
        for k, v in (("MPN", p.mpn), ("Manufacturer", p.mfr), ("LCSC", p.lcsc)):
            if v:
                setf(k, v)
        for pad in fp.Pads():
            n = p.pins.get(pad.GetNumber())
            if n and n != "NC":
                pad.SetNet(nets[n])
        if p.kind == "HOLE":
            fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
        # reference text: small, on silk
        ref = fp.Reference()
        ref.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        ref.SetTextThickness(MM(0.15))
        # too dense for silkscreen designators: they go on the fab layer (assembly drawing)
        ref.SetLayer(pcbnew.F_Fab)
        fp.Value().SetVisible(False)
        board.Add(fp)
        fps[p.ref] = fp

    placed = []   # courtyards of everything placed so far

    def put(ref, x, y, rot):
        fp = fps[ref]
        fp.SetOrientationDegrees(rot)
        fp.SetPosition(P(x, y))
        placed.append((ref, courtyard(fp, 0.0)))

    for ref, (x, y, rot) in FIXED.items():
        put(ref, x, y, rot)
    for ref, (x, y) in PADS.items():
        put(ref, x, y, 0)

    # hole keep-outs (M3 head and standoff, 6.5 mm)
    holes = [(sx * HOLE, sy * HOLE) for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    placed[:] = [(r, c) for r, c in placed if not r.startswith("H")]

    def hits_hole(c, r=3.45):
        for hx, hy in holes:
            dx = max(c[0] - hx, 0, hx - c[2])
            dy = max(c[1] - hy, 0, hy - c[3])
            if dx * dx + dy * dy < r * r:
                return True
        return False

    # sanity check the hand placement
    for i, (ra, ca) in enumerate(placed):
        if hits_hole(ca) and not ra.startswith("keepout"):
            print("WARNING: hand-placed", ra, "is inside a mounting-hole keep-out")
        for rb, cb in placed[i + 1:]:
            if overlap(ca, cb):
                print("WARNING: hand-placed", ra, "overlaps", rb)

    # spiral search from each anchor pad for a free spot, both orientations
    mcu = courtyard(fps["U1"], 0.0)
    failed = []
    byref = {p.ref: p for p in design.PARTS}
    todo = [r for r in ORDER] + [p.ref for p in design.PARTS if p.ref not in ORDER and p.ref not in FIXED
                                  and p.ref not in PADS and p.kind != "HOLE"]
    for ref in todo:
        p = byref[ref]
        aref, apad = ANCHOR[p.ref]
        pad = next(q for q in fps[aref].Pads() if q.GetNumber() == apad)
        ax, ay = pcbnew.ToMM(pad.GetPosition().x) - CX, pcbnew.ToMM(pad.GetPosition().y) - CY
        best = None
        for r in [i * 0.25 for i in range(0, 120)]:
            steps = max(8, int(2 * math.pi * r / 0.5))
            for k in range(steps):
                t = 2 * math.pi * k / steps
                x = round((ax + r * math.cos(t)) / 0.25) * 0.25
                y = round((ay + r * math.sin(t)) / 0.25) * 0.25
                for rot in (0, 90):
                    fps[p.ref].SetOrientationDegrees(rot)
                    fps[p.ref].SetPosition(P(x, y))
                    cy_ = courtyard(fps[p.ref], 0.02)
                    # keep a 1.2 mm escape ring around the QFN free for routing
                    ring = (mcu[0] - 1.0, mcu[1] - 1.0, mcu[2] + 1.0, mcu[3] + 1.0)
                    if overlap(cy_, ring):
                        continue
                    if not all(inside(cx, cyy, 0.4) for cx, cyy in
                               ((cy_[0], cy_[1]), (cy_[2], cy_[1]), (cy_[0], cy_[3]), (cy_[2], cy_[3]))):
                        continue
                    zs = ZONE_OF.get(ref)
                    if zs and not any(z[0] <= cy_[0] and z[1] <= cy_[1] and cy_[2] <= z[2] and cy_[3] <= z[3]
                                      for z in zs):
                        continue
                    if hits_hole(cy_) or any(overlap(cy_, c) for _, c in placed):
                        continue
                    best = (x, y, rot)
                    break
                if best:
                    break
            if best:
                break
        if not best:
            failed.append(p.ref)
            best = (35 + 3 * (len(failed) % 6), -20 + 3 * (len(failed) // 6), 0)
        put(p.ref, *best)

    # Silkscreen: board name, rev, connector labels
    def silk(text, x, y, size=1.0, rot=0, layer=pcbnew.F_SilkS):
        t = pcbnew.PCB_TEXT(board)
        t.SetText(text)
        t.SetPosition(P(x, y))
        t.SetLayer(layer)
        t.SetTextSize(pcbnew.VECTOR2I(MM(size), MM(size)))
        t.SetTextThickness(MM(0.15 if size < 1.2 else 0.2))
        t.SetTextAngleDegrees(rot)
        t.SetMirrored(layer == pcbnew.B_SilkS)
        board.Add(t)

    silk("SHOW FC rev A", 0, 23.6 - 47.2 + 0.2 + 46.0, 1.2, layer=pcbnew.B_SilkS)
    silk("drone-show 2-layer RP2354B", 0, 20.6, 1.0, layer=pcbnew.B_SilkS)
    for ref, text in (("J3", "ESC"), ("J4", "GPS"), ("J5", "LIGHT"), ("J6", "RC"),
                      ("J8", "SWD"), ("J1", "USB"), ("J2", "SD")):
        x, y, rot = FIXED[ref]
        silk(text, x * 0.8, y * 0.8, 0.9, layer=pcbnew.B_SilkS)
    # front arrow (frame +X between two arms after the 45 deg stack rotation)
    silk("^ FRONT", 0, -10.6, 0.9, layer=pcbnew.B_SilkS)


    pcbnew.SaveBoard(OUT, board)
    if failed:
        print("NO ROOM:", " ".join(failed))
    return board


def patch_project():
    """Net classes and the PCBWay rules go in the project file, where KiCad keeps them."""
    import json
    pro = os.path.join(ROOT, "show-fc.kicad_pro")
    d = json.load(open(pro))
    base = d["net_settings"]["classes"][0]
    classes, patterns = [], []
    for name, (tw, cl, vd, vdr, nets) in NETCLASSES.items():
        c = dict(base)
        c.update(name=name, track_width=tw, clearance=cl, via_diameter=vd, via_drill=vdr,
                 diff_pair_width=0.25, diff_pair_gap=0.15)
        classes.append(c)
        patterns += [{"netclass": name, "pattern": n} for n in nets]
    d["net_settings"]["classes"] = classes
    d["net_settings"]["netclass_patterns"] = patterns
    ds = d["board"]["design_settings"]
    ds["rules"].update(min_clearance=0.15, min_track_width=0.15, min_via_diameter=0.6,
                       min_via_annular_width=0.15, min_through_hole_diameter=0.3, min_hole_to_hole=0.4,
                       min_copper_edge_clearance=0.3, min_hole_clearance=0.15, min_text_height=0.8,
                       min_text_thickness=0.15, min_silk_clearance=0.0, min_microvia_diameter=0.2,
                       min_microvia_drill=0.1, allow_microvias=False, allow_blind_buried_vias=False)
    sev = ds.setdefault("rule_severities", {})
    # footprints come from the system KiCad library, which DRC can't see on a fab machine
    sev["lib_footprint_issues"] = "ignore"
    sev["lib_footprint_mismatch"] = "ignore"
    ds["track_widths"] = [0.0, 0.2, 0.3, 0.5, 0.8]
    ds["via_dimensions"] = [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3},
                            {"diameter": 0.8, "drill": 0.4}]
    d["meta"]["filename"] = "show-fc.kicad_pro"
    d.setdefault("sheets", [])
    json.dump(d, open(pro, "w"), indent=2)


if __name__ == "__main__":
    b = build()
    patch_project()
    print("saved", OUT, "footprints", len(b.GetFootprints()))
