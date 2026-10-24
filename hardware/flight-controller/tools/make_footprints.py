"""Write the project footprints that KiCad's standard library lacks into
hardware/flight-controller/showfc.pretty (KiCad 7 format, opens in 8-10).

QFN-80 and LGA-14 copy the land patterns OpenFC-Lite ships with. The DPS368
VLGA-8 follows Infineon's recommended land pattern; check it against the
current DPS368 datasheet before the first order (see README).
"""

import os
import uuid

OUT = os.path.join(os.path.dirname(__file__), "..", "showfc.pretty")


def _u():
    return str(uuid.uuid4())


def fp_text(kind, text, x, y, layer, size=0.8, thick=0.12, hide=False):
    h = " hide" if hide else ""
    return (f'  (fp_text {kind} "{text}" (at {x:g} {y:g}) (layer "{layer}"){h}\n'
            f'    (effects (font (size {size:g} {size:g}) (thickness {thick:g})))\n'
            f'    (tstamp {_u()})\n  )\n')


def rect(layer, x0, y0, x1, y1, width):
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1), (x0, y0)]
    return "".join(
        f'  (fp_line (start {a[0]:g} {a[1]:g}) (end {b[0]:g} {b[1]:g}) (stroke (width {width:g}) (type solid)) (layer "{layer}") (tstamp {_u()}))\n'
        for a, b in zip(pts, pts[1:]))


def line(layer, x0, y0, x1, y1, width=0.12):
    return f'  (fp_line (start {x0:g} {y0:g}) (end {x1:g} {y1:g}) (stroke (width {width:g}) (type solid)) (layer "{layer}") (tstamp {_u()}))\n'


def pad(num, x, y, w, h, shape="roundrect", paste=True, extra=""):
    layers = '"F.Cu" "F.Paste" "F.Mask"' if paste else '"F.Cu" "F.Mask"'
    rr = " (roundrect_rratio 0.2)" if shape == "roundrect" else ""
    return (f'  (pad "{num}" smd {shape} (at {x:g} {y:g}) (size {w:g} {h:g}) (layers {layers}){rr}{extra}'
            f' (tstamp {_u()}))\n')


def write(name, descr, tags, body, bx, by, model=""):
    s = f'(footprint "{name}" (version 20221018) (generator pcbnew)\n  (layer "F.Cu")\n'
    s += f'  (descr "{descr}")\n  (tags "{tags}")\n  (attr smd)\n'
    s += fp_text("reference", "REF**", 0, -by / 2 - 1.0, "F.SilkS")
    s += fp_text("value", name, 0, by / 2 + 1.0, "F.Fab")
    s += fp_text("user", "${REFERENCE}", 0, 0, "F.Fab", size=0.5, thick=0.08)
    s += rect("F.Fab", -bx / 2, -by / 2, bx / 2, by / 2, 0.1)
    s += body
    s += ")\n"
    os.makedirs(OUT, exist_ok=True)
    with open(os.path.join(OUT, name + ".kicad_mod"), "w") as f:
        f.write(s)


def qfn80():
    # RP2354B, 10 x 10 mm, 0.4 mm pitch. Pads 1-20 down the left side, 21-40
    # along the bottom, 41-60 up the right, 61-80 along the top (top view).
    body = ""
    for i in range(20):
        o = -3.8 + 0.4 * i
        body += pad(1 + i, -4.91, o, 0.78, 0.22)
        body += pad(21 + i, o, 4.91, 0.22, 0.78)
        body += pad(41 + i, 4.91, -o, 0.78, 0.22)
        body += pad(61 + i, -o, -4.91, 0.22, 0.78)
    # exposed pad: copper and mask whole, paste as a 3x3 window array (~60 %)
    body += pad(81, 0, 0, 3.4, 3.4, shape="rect", paste=False)
    for ix in (-1.1, 0, 1.1):
        for iy in (-1.1, 0, 1.1):
            body += (f'  (pad "" smd roundrect (at {ix:g} {iy:g}) (size 0.85 0.85) (layers "F.Paste")'
                     f' (roundrect_rratio 0.25) (tstamp {_u()}))\n')
    # 9 thermal vias to the bottom ground pour, 0.3 mm drill (PCBWay standard)
    for ix in (-1.1, 0, 1.1):
        for iy in (-1.1, 0, 1.1):
            body += (f'  (pad "81" thru_hole circle (at {ix:g} {iy:g}) (size 0.6 0.6) (drill 0.3)'
                     f' (layers "*.Cu") (tstamp {_u()}))\n')
    s = 5.0
    body += line("F.SilkS", -s - 0.1, -s - 0.1, -4.3, -s - 0.1)
    body += line("F.SilkS", -s - 0.1, -s - 0.1, -s - 0.1, -4.3)
    for sx, sy in ((1, -1), (1, 1), (-1, 1)):
        body += line("F.SilkS", sx * (s + 0.1), sy * (s + 0.1), sx * 4.3, sy * (s + 0.1))
        body += line("F.SilkS", sx * (s + 0.1), sy * (s + 0.1), sx * (s + 0.1), sy * 4.3)
    body += f'  (fp_circle (center -5.6 -4.6) (end -5.45 -4.6) (stroke (width 0.3) (type solid)) (fill solid) (layer "F.SilkS") (tstamp {_u()}))\n'
    body += rect("F.CrtYd", -5.55, -5.55, 5.55, 5.55, 0.05)
    write("QFN-80-1EP_10x10mm_P0.4mm_EP3.4x3.4mm", "RP2354B QFN-80, land pattern from OpenFC-Lite",
          "QFN-80 RP2354B RP2350B", body, 10, 10)


def lga14():
    # Same land as OpenFC-Lite's multi-source IMU pattern (BMI270, ICM-42688-P)
    pts = {1: (1.17, 0.75), 2: (1.17, 0.25), 3: (1.17, -0.25), 4: (1.17, -0.75),
           5: (0.5, -0.92), 6: (0, -0.92), 7: (-0.5, -0.92),
           8: (-1.17, -0.75), 9: (-1.17, -0.25), 10: (-1.17, 0.25), 11: (-1.17, 0.75),
           12: (-0.5, 0.92), 13: (0, 0.92), 14: (0.5, 0.92)}
    body = ""
    for n, (x, y) in pts.items():
        w, h = (0.6, 0.27) if n in (1, 2, 3, 4, 8, 9, 10, 11) else (0.27, 0.6)
        body += pad(n, x, y, w, h, shape="rect")
    body += f'  (fp_circle (center 1.85 1.3) (end 1.95 1.3) (stroke (width 0.2) (type solid)) (fill solid) (layer "F.SilkS") (tstamp {_u()}))\n'
    body += rect("F.CrtYd", -1.75, -1.5, 1.75, 1.5, 0.05)
    write("LGA-14_3x2.5mm_P0.5mm", "LGA-14 3.0 x 2.5 mm IMU, BMI270 / ICM-42688-P (OpenFC-Lite land)",
          "LGA-14 BMI270 ICM-42688", body, 3.0, 2.5)


def vlga8():
    # DPS368 / DPS310, PG-VLGA-8-2, 2.0 x 2.5 x 1.1 mm, 0.65 mm pitch.
    # Top view: pins 1-4 along one long edge, 5-8 back along the other,
    # counter-clockwise, pin 1 at the marked corner.
    body = ""
    for i in range(4):
        x = -0.975 + 0.65 * i
        body += pad(1 + i, x, 0.8, 0.35, 0.5, shape="rect")
        body += pad(8 - i, x, -0.8, 0.35, 0.5, shape="rect")
    body += f'  (fp_circle (center -1.5 1.35) (end -1.4 1.35) (stroke (width 0.2) (type solid)) (fill solid) (layer "F.SilkS") (tstamp {_u()}))\n'
    body += rect("F.CrtYd", -1.5, -1.25, 1.5, 1.25, 0.05)
    write("Infineon_VLGA-8_2.0x2.5mm_P0.65mm", "Infineon PG-VLGA-8-2 (DPS310 / DPS368). Verify pin order against the datasheet.",
          "DPS368 DPS310 VLGA-8 barometer", body, 2.5, 2.0)


if __name__ == "__main__":
    qfn80()
    lga14()
    vlga8()
    print("wrote", sorted(os.listdir(OUT)))
