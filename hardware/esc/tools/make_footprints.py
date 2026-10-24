"""Finish the local footprint library. Run with KiCad's Python from hardware/esc.

* QFN-28 for the AT32F421G8U7: the prototype footprint had pads 8, 23 and 24
  removed. They are restored so every pin is soldered.
* JST SH 8-pin top entry: the stock KiCad footprint names both mounting pads
  "MP"; the schematic symbol calls them P1 and P2.
* ESC_Pads_Octagon60: battery pads, FC wire pads, mounting holes and motor pads
  in one footprint, matching the pin numbers of the prototype's 4in1ESC symbol.
  Its geometry comes from layout.py so the board script and the footprint agree.
"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import layout as L  # noqa: E402

MM = pcbnew.FromMM
PRJ = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
LIB = os.path.join(PRJ, "lib", "drone-esc.pretty")
STOCK = os.environ.get("KICAD_FOOTPRINTS", "/usr/share/kicad/footprints")
io = pcbnew.PCB_IO_KICAD_SEXPR()


def v(x, y):
    return pcbnew.VECTOR2I(MM(x), MM(y))


def fix_qfn28():
    name = "QFN-28_L4.0-W4.0-P0.40-TL-EP2.4"
    fp = io.FootprintLoad(LIB, name)
    have = {p.GetNumber() for p in fp.Pads()}
    ref = next(p for p in fp.Pads() if p.GetNumber() == "9")      # bottom row
    top = next(p for p in fp.Pads() if p.GetNumber() == "22")     # top row
    for num, x, src in (("8", -1.2, ref), ("23", 0.8, top), ("24", 0.4, top)):
        if num in have:
            continue
        p = pcbnew.PAD(src)
        p.SetNumber(num)
        p.SetFPRelativePosition(v(x, pcbnew.ToMM(src.GetFPRelativePosition().y)))
        fp.Add(p)
    io.FootprintSave(LIB, fp)


def jst_vertical():
    fp = io.FootprintLoad(os.path.join(STOCK, "Connector_JST.pretty"),
                          "JST_SH_BM08B-SRSS-TB_1x08-1MP_P1.00mm_Vertical")
    mps = sorted((p for p in fp.Pads() if p.GetNumber() == "MP"),
                 key=lambda p: p.GetFPRelativePosition().x)
    mps[0].SetNumber("P1")
    mps[1].SetNumber("P2")
    fp.SetFPID(pcbnew.LIB_ID("", "JST_SH_BM08B-SRSS-TB_Vertical"))
    io.FootprintSave(LIB, fp)


def add_pad(fp, num, x, y, w, h, drill=0.0, angle=0.0, layers="smd_top", shape=pcbnew.PAD_SHAPE_ROUNDRECT):
    p = pcbnew.PAD(fp)
    p.SetNumber(num)
    if drill:
        p.SetAttribute(pcbnew.PAD_ATTRIB_PTH)
        p.SetLayerSet(pcbnew.PAD.PTHMask())
        p.SetDrillSize(v(drill, drill))
    elif layers == "smd_bot":
        p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        ls = pcbnew.LSET()
        for layer in (pcbnew.B_Cu, pcbnew.B_Mask):
            ls.AddLayer(layer)
        p.SetLayerSet(ls)
    else:
        p.SetAttribute(pcbnew.PAD_ATTRIB_SMD)
        p.SetLayerSet(pcbnew.PAD.SMDMask())
    p.SetShape(pcbnew.F_Cu, shape)
    if shape == pcbnew.PAD_SHAPE_ROUNDRECT:
        p.SetRoundRectRadiusRatio(pcbnew.F_Cu, 0.15)
    p.SetSize(pcbnew.F_Cu, v(w, h))
    p.SetFPRelativePosition(v(x, y))
    p.SetFPRelativeOrientation(pcbnew.EDA_ANGLE(angle, pcbnew.DEGREES_T))
    fp.Add(p)
    return p


def esc_pads():
    fp = pcbnew.FOOTPRINT(None)
    fp.SetFPID(pcbnew.LIB_ID("", "ESC_Pads_Octagon60"))
    fp.SetReference("U3")
    fp.SetValue("ESC_Pads_Octagon60")
    fp.Reference().SetVisible(False)
    fp.Value().SetVisible(False)
    fp.SetAttributes(pcbnew.FP_EXCLUDE_FROM_POS_FILES | pcbnew.FP_EXCLUDE_FROM_BOM)
    # battery: 1 = battery + (feeds the shunts), 2 = battery -
    for num, (x, y, ang) in L.battery_pads().items():
        add_pad(fp, num, x, y, L.BATT_PAD_D, L.BATT_PAD_D, drill=L.BATT_PAD_DRILL,
                shape=pcbnew.PAD_SHAPE_CIRCLE)
    # FC wire pads, top side next to the connector, same order as the JST: +BATT GND CURR TX M1 M2 M3 M4
    for num, (x, y) in L.wire_pads().items():
        add_pad(fp, num, x, y, L.WIRE_PAD_W, L.WIRE_PAD_H)
    # mounting holes, plated, tied to GND like the prototype
    for x, y in L.holes():
        add_pad(fp, "11", x, y, L.HOLE_PAD, L.HOLE_PAD, drill=L.HOLE_D, shape=pcbnew.PAD_SHAPE_CIRCLE)
    # motor pads, plated through so the wire's current reaches both layers
    for num, (x, y, ang) in L.motor_pads().items():
        add_pad(fp, num, x, y, L.MOTOR_PAD_W, L.MOTOR_PAD_H, drill=L.MOTOR_PAD_DRILL, angle=ang)
    io.FootprintSave(LIB, fp)


if __name__ == "__main__":
    fix_qfn28()
    jst_vertical()
    esc_pads()
    print("footprints written to", LIB)
