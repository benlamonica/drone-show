"""Build rtk_gps.kicad_pcb from the schematic netlist: outline, holes, placement, planes.

Runs inside the kicad/kicad:9.0 container (pcbnew Python). Routing is a separate step (route.py).
"""
import math, os, sys
import pcbnew
import sexpdata
from sexpdata import Symbol as S

PRJ = sys.argv[1]
NET = sys.argv[2]
FPLIB = "/usr/share/kicad/footprints/"
X0, Y0, W, H = 100.0, 100.0, 45.0, 45.0
HOLE_INSET = 3.0

# ref: (x, y, rot) in mm, board origin at (X0, Y0). All parts on the top side.
P = {
    # connectors
    "J5": (111.0, 104.0, 180),     # USB-C, mouth on the top edge
    "J2": (110.5, 140.5, 0),       # AUX GH-4, mouth on the bottom edge
    "J1": (125.0, 140.5, 0),       # FC GH-6, mouth on the bottom edge
    "J3": (141.6, 131.8, 90),      # SWD SH-3, mouth on the right edge
    "J4": (141.6, 125.0, 270),     # U.FL, signal pad up toward U3 pin 11
    # GNSS: rotated so the RF pin sits in the bottom-right corner, next to J4
    "U3": (135.5, 113.0, 90),
    "C23": (128.6, 104.2, 0), "C22": (131.7, 104.2, 0), "C27": (134.8, 104.2, 0),
    "TP1": (130.0, 122.0, 0),
    "R11": (136.5, 122.0, 90), "L2": (139.3, 122.2, 0),
    "C24": (134.2, 121.6, 0), "C28": (136.5, 125.2, 90),
    # compass: bottom-right corner, away from the regulator, USB and RP2350
    "U4": (136.5, 138.0, 0), "C25": (140.2, 137.0, 90), "C26": (136.5, 134.6, 0),
    "R9": (131.4, 134.6, 90), "R10": (133.2, 134.6, 90),
    # MCU, rotated so QSPI/USB/VREG face left, GPIO0-9 face the connectors below
    "U1": (119.5, 125.0, 90),
    "U2": (110.5, 128.5, 90),
    "C1": (113.8, 122.4, 0), "L1": (113.4, 120.2, 0), "C3": (112.7, 117.6, 0),
    "C2": (109.6, 117.6, 0), "R1": (106.5, 117.6, 0),
    "C13": (106.5, 119.6, 0), "C14": (109.6, 119.6, 0), "C18": (110.5, 134.8, 0),
    "C12": (116.6, 119.3, 90), "C15": (118.3, 119.3, 90), "C6": (120.0, 119.3, 90), "C11": (121.7, 119.3, 90),
    "C7": (114.6, 131.0, 90), "C4": (125.2, 131.6, 0), "C8": (127.0, 133.4, 0),
    "C10": (125.2, 122.0, 0), "R4": (125.2, 125.2, 0), "C9": (125.2, 128.4, 0), "C5": (125.2, 130.0, 0),
    "Y1": (129.0, 125.6, 0), "C16": (132.4, 125.6, 90), "C17": (129.0, 128.8, 0),
    "R5": (106.0, 128.5, 90), "SW1": (118.9, 102.9, 0),
    "R6": (122.4, 134.6, 90), "SW2": (124.2, 102.9, 0),
    "R13": (119.0, 116.5, 0), "D4": (122.2, 116.5, 0),
    # USB
    "R7": (106.0, 111.0, 90), "R8": (116.0, 111.0, 90), "R2": (112.2, 112.2, 90), "R3": (110.0, 112.2, 90),
    # power
    "D1": (120.0, 110.6, 0), "D2": (120.0, 108.2, 0),
    "U5": (123.6, 109.4, 90), "C20": (120.0, 113.0, 0), "C21": (124.0, 113.0, 0),
    "R12": (104.0, 135.0, 90), "D3": (104.0, 138.0, 90),
}

# reference text moved off the board edge / connector bodies
REF_AT = {"SW1": (118.9, 105.6), "SW2": (124.2, 105.6), "J4": (138.4, 127.6), "J3": (137.6, 129.0),
          "J5": (111.0, 109.6), "J1": (125.0, 136.4), "J2": (110.5, 136.4)}


def mm(v):
    return pcbnew.FromMM(v)


def vec(x, y):
    return pcbnew.VECTOR2I(mm(x), mm(y))


def read_netlist(path):
    t = sexpdata.loads(open(path).read(), nil=None, true=None, false=None)
    get = lambda e, k: [x for x in e if isinstance(x, list) and x and x[0] == S(k)]
    comps = {}
    for c in get(t, "components")[0][1:]:
        ref = get(c, "ref")[0][1]
        fields = {f[1][1]: (f[2] if len(f) > 2 else "") for f in (get(get(c, "fields")[0], "field") if get(c, "fields") else [])}
        comps[ref] = dict(value=get(c, "value")[0][1], fp=get(c, "footprint")[0][1] if get(c, "footprint") else "",
                          tstamp=get(c, "tstamps")[0][1], fields=fields)
    pads = {}
    for n in get(t, "nets")[0][1:]:
        name = get(n, "name")[0][1]
        for node in get(n, "node"):
            pads[(get(node, "ref")[0][1], get(node, "pin")[0][1])] = name
    return comps, pads


comps, padnet = read_netlist(NET)
board = pcbnew.BOARD()
board.SetCopperLayerCount(2)
ds = board.GetDesignSettings()
ds.SetBoardThickness(mm(1.6))

nets = {}
for name in sorted(set(padnet.values())):
    ni = pcbnew.NETINFO_ITEM(board, name)
    board.Add(ni)
    nets[name] = ni

# outline: rounded rectangle, r = 1.5 mm
r = 1.5
def seg(a, b):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_SEGMENT)
    s.SetStart(vec(*a)); s.SetEnd(vec(*b)); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); board.Add(s)
def arc(c, start_ang):
    s = pcbnew.PCB_SHAPE(board, pcbnew.SHAPE_T_ARC)
    pts = [(c[0] + r * math.cos(math.radians(a)), c[1] + r * math.sin(math.radians(a))) for a in (start_ang, start_ang + 45, start_ang + 90)]
    s.SetArcGeometry(vec(*pts[0]), vec(*pts[1]), vec(*pts[2])); s.SetLayer(pcbnew.Edge_Cuts); s.SetWidth(mm(0.1)); board.Add(s)
x1, y1, x2, y2 = X0, Y0, X0 + W, Y0 + H
seg((x1 + r, y1), (x2 - r, y1)); seg((x2, y1 + r), (x2, y2 - r)); seg((x2 - r, y2), (x1 + r, y2)); seg((x1, y2 - r), (x1, y1 + r))
arc((x2 - r, y1 + r), 270); arc((x2 - r, y2 - r), 0); arc((x1 + r, y2 - r), 90); arc((x1 + r, y1 + r), 180)

# footprints from the netlist
for ref, c in comps.items():
    lib, name = c["fp"].split(":")
    fp = pcbnew.FootprintLoad(FPLIB + lib + ".pretty", name)
    fp.SetFPID(pcbnew.LIB_ID(lib, name))
    fp.SetReference(ref)
    fp.SetValue(c["value"])
    fp.SetPath(pcbnew.KIID_PATH("/" + c["tstamp"]))
    for k in ("MPN", "Manufacturer", "Mouser"):
        if c["fields"].get(k):
            fp.SetField(k, c["fields"][k])
            fp.GetFieldByName(k).SetVisible(False)
            fp.GetFieldByName(k).SetLayer(pcbnew.F_Fab)
    for pad in fp.Pads():
        n = padnet.get((ref, pad.GetNumber()))
        if n in nets:
            pad.SetNet(nets[n])
    x, y, rot = P[ref]
    fp.SetPosition(vec(x, y))
    fp.SetOrientationDegrees(rot)
    fp.Reference().SetTextSize(vec(0.8, 0.8)); fp.Reference().SetTextThickness(mm(0.15))
    fp.Value().SetVisible(False)
    if ref in REF_AT:
        fp.Reference().SetPosition(vec(*REF_AT[ref]))
    board.Add(fp)

# mounting holes (board-only), M2 on a 39 mm square
for i, (hx, hy) in enumerate([(X0 + HOLE_INSET, Y0 + HOLE_INSET), (X0 + W - HOLE_INSET, Y0 + HOLE_INSET),
                              (X0 + HOLE_INSET, Y0 + H - HOLE_INSET), (X0 + W - HOLE_INSET, Y0 + H - HOLE_INSET)]):
    fp = pcbnew.FootprintLoad(FPLIB + "MountingHole.pretty", "MountingHole_2.2mm_M2")
    fp.SetFPID(pcbnew.LIB_ID("MountingHole", "MountingHole_2.2mm_M2"))
    fp.SetReference(f"H{i+1}")
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    fp.SetAttributes(fp.GetAttributes() | pcbnew.FP_BOARD_ONLY | pcbnew.FP_EXCLUDE_FROM_BOM | pcbnew.FP_EXCLUDE_FROM_POS_FILES)
    fp.SetPosition(vec(hx, hy))
    board.Add(fp)

# GND planes, both layers; the bottom is the reference plane under everything
def zone(layer, prio):
    z = pcbnew.ZONE(board)
    z.SetLayer(layer)
    z.SetNet(nets["/GND"])
    z.SetZoneName(f"GND_{'top' if layer == pcbnew.F_Cu else 'bottom'}")
    z.SetAssignedPriority(prio)
    z.SetLocalClearance(mm(0.25))
    z.SetMinThickness(mm(0.2))
    z.SetThermalReliefGap(mm(0.25))
    z.SetThermalReliefSpokeWidth(mm(0.35))
    z.SetPadConnection(pcbnew.ZONE_CONNECTION_FULL)
    z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    o = z.Outline()
    o.NewOutline()
    for px, py in ((x1 + 0.3, y1 + 0.3), (x2 - 0.3, y1 + 0.3), (x2 - 0.3, y2 - 0.3), (x1 + 0.3, y2 - 0.3)):
        o.Append(mm(px), mm(py))
    board.Add(z)
# Only the bottom plane exists while autorouting, so every GND pad gets its own via to it;
# route_post.py adds the top-side GND pour afterwards.
zone(pcbnew.B_Cu, 0)

# pick-and-place origin at the bottom-left board corner
ds.SetAuxOrigin(vec(X0, Y0 + H))

# title block + silkscreen label
tb = board.GetTitleBlock()
tb.SetTitle("RTK GPS + compass"); tb.SetRevision("A"); tb.SetCompany("drone-show")
t = pcbnew.PCB_TEXT(board)
t.SetText("RTK GPS rev A  NEO-F10N + RP2350 + MMC5983MA")
t.SetLayer(pcbnew.B_SilkS); t.SetPosition(vec(X0 + W / 2, Y0 + H / 2)); t.SetMirrored(True)
t.SetTextSize(vec(1.0, 1.0)); t.SetTextThickness(mm(0.15))
board.Add(t)
t2 = pcbnew.PCB_TEXT(board)
t2.SetText("FC: 5V TX RX SCL SDA GND")
t2.SetLayer(pcbnew.B_SilkS); t2.SetPosition(vec(X0 + W / 2, Y0 + H / 2 + 2.5)); t2.SetMirrored(True)
t2.SetTextSize(vec(1.0, 1.0)); t2.SetTextThickness(mm(0.15))
board.Add(t2)

out = os.path.join(PRJ, "rtk_gps.kicad_pcb")
board.Save(out)
print("saved", out, len(comps), "parts")
