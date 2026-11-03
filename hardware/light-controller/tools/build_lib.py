"""Fill lightctl.pretty with every footprint the board uses.

Stock footprints are copied from the KiCad library so the project opens on any
machine. Three are not in the stock library and are written here:

* RPi_RM2: pad positions from the MIT-licensed eelstretching/rp2350b-rm2-reference
  footprint (21 castellated pads, 1.5 mm pitch, pads run 0.5 mm past the module edge),
  with its antenna keep-out.
* RP2350A_QFN-60: copied from OpenFC-Lite-Mini's QFN-60 (the RP2354A package, the same
  land pattern as the RP2350A).
* L_Abracon_AOTA-B201610: the 2.0 x 1.6 mm RP2350 core-regulator inductor, also from
  OpenFC-Lite-Mini.
"""
import os
import re
import shutil
import sys

sys.path.insert(0, os.path.dirname(__file__))
from design import F  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "..", "lightctl.pretty")
STOCK = "/usr/share/kicad/footprints"
OPENFC = os.environ.get("OPENFC", os.path.join(HERE, "..", "..", "..", "..", "OpenFC-Lite-Mini", "hardware", "lib.pretty"))

STOCK_LIBS = ["Resistor_SMD", "Capacitor_SMD", "LED_SMD", "Diode_SMD", "Package_TO_SOT_SMD",
              "Package_SO", "Crystal", "Inductor_SMD", "Connector_USB", "Connector_JST",
              "Connector_Wire", "Button_Switch_SMD", "TestPoint", "Fiducial"]


def find_stock(name):
    for lib in STOCK_LIBS:
        p = os.path.join(STOCK, lib + ".pretty", name + ".kicad_mod")
        if os.path.exists(p):
            return p
    return None


def rm2():
    pads = []
    for i in range(7):                       # 1-7 down the left edge
        pads.append((str(i + 1), -6.75, -2.55 + 1.5 * i, 2, 1))
    for i in range(7):                       # 8-14 along the bottom
        pads.append((str(i + 8), -4.5 + 1.5 * i, 7.75, 1, 2))
    for i in range(7):                       # 15-21 up the right edge
        pads.append((str(i + 15), 6.75, 6.45 - 1.5 * i, 2, 1))
    s = ['(footprint "RPi_RM2" (version 20221018) (generator pcbnew)',
         '  (layer "F.Cu")',
         '  (descr "Raspberry Pi Radio Module 2 (RM2, CYW43439), 21 castellated pads, 1.5 mm pitch, '
         '14.5 x 16.5 mm. Pads after eelstretching/rp2350b-rm2-reference (MIT). Antenna at the top, '
         'above pins 1 and 21: keep copper out of that area on every layer.")',
         '  (tags "RM2 CYW43439 wifi")',
         '  (attr smd)',
         '  (fp_text reference "REF**" (at 0 -9.6) (layer "F.SilkS") (effects (font (size 1 1) (thickness 0.15))))',
         '  (fp_text value "RPi_RM2" (at 0 10.2) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))',
         '  (fp_text user "ANTENNA" (at 0 -5.8) (layer "F.Fab") (effects (font (size 1 1) (thickness 0.15))))',
         '  (fp_rect (start -7.25 -8.25) (end 7.25 8.25) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))',
         '  (fp_rect (start -7.75 -8.5) (end 7.75 8.75) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))',
         '  (fp_line (start -7.37 -8.37) (end 7.37 -8.37) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))',
         '  (fp_line (start -7.37 -8.37) (end -7.37 -3.4) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))',
         '  (fp_line (start 7.37 -8.37) (end 7.37 -3.4) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))',
         '  (fp_poly (pts (xy -8.1 -2.55) (xy -8.6 -2.85) (xy -8.6 -2.25)) (stroke (width 0.12) (type solid)) (fill solid) (layer "F.SilkS"))',
         '  (fp_rect (start -7.25 -8.25) (end 7.25 -3.3) (stroke (width 0.1) (type dash)) (fill none) (layer "Cmts.User"))']
    for num, x, y, w, h in pads:
        s.append(f'  (pad "{num}" smd rect (at {x:g} {y:g}) (size {w:g} {h:g}) (layers "F.Cu" "F.Paste" "F.Mask"))')
    s.append('  (zone (net 0) (net_name "") (layers "F.Cu" "B.Cu") (hatch full 0.508) (name "RM2 antenna keep-out")'
             ' (connect_pads (clearance 0)) (min_thickness 0.254)'
             ' (keepout (tracks not_allowed) (vias not_allowed) (pads not_allowed) (copperpour not_allowed) (footprints allowed))'
             ' (fill (thermal_gap 0.508) (thermal_bridge_width 0.508))'
             ' (polygon (pts (xy -7.25 -8.25) (xy 7.25 -8.25) (xy 7.25 -3.3) (xy -7.25 -3.3))))')
    s.append(')')
    return "\n".join(s) + "\n"


def aota():
    # pads as in OpenFC-Lite-Mini's IND-SMD_L2.0-W1.6_AOTA-B201610S3R3-101-T
    return "\n".join([
        '(footprint "L_Abracon_AOTA-B201610" (version 20221018) (generator pcbnew)',
        '  (layer "F.Cu")',
        '  (descr "Abracon AOTA-B201610 2.0 x 1.6 mm power inductor (RP2350 VREG)")',
        '  (attr smd)',
        '  (fp_text reference "REF**" (at 0 -1.6) (layer "F.SilkS") (effects (font (size 0.8 0.8) (thickness 0.12))))',
        '  (fp_text value "AOTA-B201610" (at 0 1.6) (layer "F.Fab") (effects (font (size 0.8 0.8) (thickness 0.12))))',
        '  (fp_rect (start -1 -0.8) (end 1 0.8) (stroke (width 0.1) (type solid)) (fill none) (layer "F.Fab"))',
        '  (fp_rect (start -1.75 -1.05) (end 1.75 1.05) (stroke (width 0.05) (type solid)) (fill none) (layer "F.CrtYd"))',
        '  (fp_line (start -0.3 -0.92) (end 0.3 -0.92) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))',
        '  (fp_line (start -0.3 0.92) (end 0.3 0.92) (stroke (width 0.12) (type solid)) (layer "F.SilkS"))',
        '  (pad "1" smd rect (at -1 0) (size 1 1.6) (layers "F.Cu" "F.Paste" "F.Mask"))',
        '  (pad "2" smd rect (at 1 0) (size 1 1.6) (layers "F.Cu" "F.Paste" "F.Mask"))',
        ')', ""])


def renamed(src, new):
    t = open(src).read()
    t = re.sub(r'^\((module|footprint) [^\s(]+', f'(footprint "{new}"', t, count=1)
    t = t.replace("easyeda2kicad:", "")
    return t


def main():
    os.makedirs(OUT, exist_ok=True)
    for f in os.listdir(OUT):
        os.remove(os.path.join(OUT, f))
    names = set(F.values()) | {"Fiducial_1mm_Mask2mm"}
    custom = {"RPi_RM2", "RP2350A_QFN-60_7x7mm_P0.4mm", "L_Abracon_AOTA-B201610"}
    for n in sorted(names - custom):
        p = find_stock(n)
        if not p:
            sys.exit(f"missing stock footprint {n}")
        shutil.copy(p, os.path.join(OUT, n + ".kicad_mod"))
    open(os.path.join(OUT, "RPi_RM2.kicad_mod"), "w").write(rm2())
    open(os.path.join(OUT, "RP2350A_QFN-60_7x7mm_P0.4mm.kicad_mod"), "w").write(
        renamed(os.path.join(OPENFC, "QFN-60_L7.0-W7.0-P0.40-TL-EP3.4.kicad_mod"), "RP2350A_QFN-60_7x7mm_P0.4mm"))
    open(os.path.join(OUT, "L_Abracon_AOTA-B201610.kicad_mod"), "w").write(aota())
    print(len(os.listdir(OUT)), "footprints in", os.path.normpath(OUT))


if __name__ == "__main__":
    main()
