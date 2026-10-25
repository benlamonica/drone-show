"""Generate showfc.kicad_sym and show-fc.kicad_sch from design.py.

The schematic is one A1 sheet, one block per function. Every pin ends in a
global net label, so the drawing reads like a netlist: find a net by name with
KiCad's search. Re-run after any change to design.py; hand edits to the
generated files are overwritten.
"""

import os
import uuid
from collections import defaultdict

import design

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PROJECT = "show-fc"
LIB = "showfc"
NS = uuid.UUID("6f1c0b52-6d2e-4f43-9a52-5a0f3c4e7d10")   # stable uuids between runs


def uid(*key):
    return str(uuid.uuid5(NS, "/".join(map(str, key))))


# ---------------------------------------------------------------------------
# Symbol definitions: pin number -> (name, electrical type, side)

PWR = "power_in"


def mcu_pins():
    pins = {}
    for num, net in design.MCU_PINS.items():
        pass
    names = {  # from the RP2354B symbol in OpenFC-Lite lib.kicad_sym
        5: "IOVDD", 10: "DVDD", 15: "IOVDD", 24: "IOVDD", 29: "IOVDD", 30: "XIN", 31: "XOUT",
        32: "DVDD", 33: "SWCLK", 34: "SWDIO", 35: "RUN", 41: "IOVDD", 50: "IOVDD", 51: "DVDD",
        59: "ADC_AVDD", 60: "IOVDD", 61: "VREG_AVDD", 62: "VREG_PGND", 63: "VREG_LX",
        64: "VREG_VIN", 65: "VREG_FB", 66: "USB_DM", 67: "USB_DP", 68: "USB_OTP_VDD",
        69: "QSPI_IOVDD", 70: "QSPI_SD3", 71: "QSPI_SCLK", 72: "QSPI_SD0", 73: "QSPI_SD2",
        74: "QSPI_SD1", 75: "QSPI_SS", 76: "IOVDD", 81: "GND"}
    gpio = {}
    g = 0
    order = [77, 78, 79, 80] + list(range(1, 5)) + list(range(6, 10)) + list(range(11, 15)) + \
        list(range(16, 24)) + list(range(25, 29)) + list(range(36, 41)) + list(range(42, 50)) + \
        list(range(52, 59))
    for n in order:
        gpio[n] = g
        g += 1
    for n in range(1, 82):
        if n in gpio:
            name = f"GPIO{gpio[n]}"
            if gpio[n] >= 40:
                name += f"/ADC{gpio[n] - 40}"
            side = "L" if gpio[n] < 24 else "R"
            pins[str(n)] = (name, "bidirectional", side)
        else:
            name = names[n]
            if name in ("GND", "VREG_PGND"):
                pins[str(n)] = (name, PWR, "B")
            elif name in ("IOVDD", "DVDD", "ADC_AVDD", "VREG_AVDD", "VREG_VIN", "USB_OTP_VDD", "QSPI_IOVDD"):
                pins[str(n)] = (name, PWR, "T")
            elif name.startswith("QSPI") or name.startswith("USB") or name.startswith("VREG"):
                pins[str(n)] = (name, "passive", "R")
            else:
                pins[str(n)] = (name, "passive", "L")
    return pins


def conn(n, mp=True):
    p = {str(i): (f"Pin_{i}", "passive", "L") for i in range(1, n + 1)}
    if mp:
        p["MP"] = ("MP", "passive", "B")
    return p


KINDS = {
    "RP2354B": dict(ref="U", pins=mcu_pins()),
    "R": dict(ref="R", two="res"),
    "C": dict(ref="C", two="cap"),
    "L": dict(ref="L", two="ind"),
    "LED": dict(ref="D", two="led"),
    "D": dict(ref="D", two="diode"),
    "TVS": dict(ref="D", two="diode"),
    "SW": dict(ref="SW", two="sw"),
    "TP": dict(ref="TP", pins={"1": ("1", "passive", "L")}),
    "HOLE": dict(ref="H", pins={}),
    "XTAL4": dict(ref="Y", pins={"1": ("X1", "passive", "L"), "3": ("X2", "passive", "R"),
                                  "2": ("GND", "passive", "B"), "4": ("GND", "passive", "B")}),
    "NMOS": dict(ref="Q", pins={"1": ("G", "passive", "L"), "2": ("S", "passive", "B"),
                                "3": ("D", "passive", "T")}),
    "CONN3": dict(ref="J", pins=conn(3)),
    "CONN4": dict(ref="J", pins=conn(4)),
    "CONN6": dict(ref="J", pins=conn(6)),
    "CONN8": dict(ref="J", pins=conn(8)),
    "USB_C": dict(ref="J", pins={
        "A4": ("VBUS", "passive", "L"), "A9": ("VBUS", "passive", "L"),
        "B4": ("VBUS", "passive", "L"), "B9": ("VBUS", "passive", "L"),
        "A5": ("CC1", "passive", "L"), "B5": ("CC2", "passive", "L"),
        "A6": ("D+", "passive", "R"), "B6": ("D+", "passive", "R"),
        "A7": ("D-", "passive", "R"), "B7": ("D-", "passive", "R"),
        "A8": ("SBU1", "passive", "R"), "B8": ("SBU2", "passive", "R"),
        "A1": ("GND", "passive", "B"), "A12": ("GND", "passive", "B"),
        "B1": ("GND", "passive", "B"), "B12": ("GND", "passive", "B"),
        "S1": ("SHIELD", "passive", "B")}),
    "USBLC6": dict(ref="U", pins={"1": ("I/O1", "passive", "L"), "6": ("I/O1", "passive", "R"),
                                  "3": ("I/O2", "passive", "L"), "4": ("I/O2", "passive", "R"),
                                  "5": ("VBUS", "passive", "T"), "2": ("GND", "passive", "B")}),
    "LMR51635": dict(ref="U", pins={"3": ("VIN", PWR, "L"), "2": ("EN", "input", "L"),
                                    "1": ("FB", "input", "L"), "6": ("CB", "passive", "R"),
                                    "5": ("SW", "power_out", "R"), "4": ("GND", PWR, "B")}),
    "LP5912": dict(ref="U", pins={"6": ("IN", PWR, "L"), "4": ("EN", "input", "L"),
                                  "1": ("OUT", "power_out", "R"), "3": ("PG", "open_collector", "R"),
                                  "2": ("NC", "no_connect", "R"), "5": ("GND", PWR, "B"),
                                  "7": ("EP", "passive", "B")}),
    "IMU_LGA14": dict(ref="U", pins={
        "1": ("SDO", "passive", "R"), "14": ("SDI", "passive", "R"), "13": ("SCK", "passive", "R"),
        "12": ("CS", "passive", "R"), "4": ("INT1", "passive", "R"), "9": ("INT2/CLKIN", "passive", "R"),
        "5": ("VDDIO", PWR, "T"), "8": ("VDD", PWR, "T"),
        "2": ("ASDX/RESV", "passive", "L"), "3": ("ASCX/RESV", "passive", "L"),
        "10": ("OCSB/RESV", "passive", "L"), "11": ("OSDO/RESV", "passive", "L"),
        "6": ("GNDIO", PWR, "B"), "7": ("GND", PWR, "B")}),
    "DPS368": dict(ref="U", pins={
        "8": ("VDD", PWR, "T"), "6": ("VDDIO", PWR, "T"),
        "2": ("CSB", "input", "L"), "5": ("SDO", "passive", "L"),
        "3": ("SDI", "passive", "R"), "4": ("SCK", "input", "R"),
        "1": ("GND", PWR, "B"), "7": ("GND", PWR, "B")}),
    "MICROSD": dict(ref="J", pins={
        "1": ("DAT2", "passive", "L"), "2": ("DAT3/CS", "passive", "L"), "3": ("CMD/DI", "passive", "L"),
        "4": ("VDD", PWR, "T"), "5": ("CLK", "passive", "L"), "6": ("VSS", PWR, "B"),
        "7": ("DAT0/DO", "passive", "L"), "8": ("DAT1", "passive", "L"),
        "9": ("DET_B", "passive", "R"), "10": ("DET_A", "passive", "R"), "11": ("SHIELD", "passive", "B")}),
    "PWR_FLAG": dict(ref="#FLG", pins={"1": ("pwr", "power_out", "B")}),
}
NAMES = {"CONN3": "Conn_01x03_MP", "CONN4": "Conn_01x04_MP", "CONN6": "Conn_01x06_MP",
         "CONN8": "Conn_01x08_MP"}

G = 2.54


def fmt(v):
    return f"{v:.4f}".rstrip("0").rstrip(".")


def pin_sexpr(num, name, etype, x, y, ang, length=G, hide_name=False):
    return (f'      (pin {etype} line (at {fmt(x)} {fmt(y)} {ang}) (length {fmt(length)})\n'
            f'        (name "{name}" (effects (font (size 1.27 1.27))))\n'
            f'        (number "{num}" (effects (font (size 1.27 1.27))))\n      )\n')


def two_pin(kind, art):
    """Vertical two-pin part: pin 1 on top, pin 2 at the bottom.

    LED and diodes put the cathode (pin 1) on top, so current flows upward.
    """
    gfx = ""
    if art in ("res", "ind"):
        gfx = '      (rectangle (start -1.016 -2.032) (end 1.016 2.032) (stroke (width 0.254) (type default)) (fill (type none)))\n'
    elif art == "cap":
        gfx = ('      (polyline (pts (xy -2.032 0.508) (xy 2.032 0.508)) (stroke (width 0.508) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy -2.032 -0.508) (xy 2.032 -0.508)) (stroke (width 0.508) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy 0 2.54) (xy 0 0.508)) (stroke (width 0) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy 0 -2.54) (xy 0 -0.508)) (stroke (width 0) (type default)) (fill (type none)))\n')
    elif art in ("led", "diode"):
        gfx = ('      (polyline (pts (xy -1.27 -1.016) (xy 1.27 -1.016) (xy 0 1.016) (xy -1.27 -1.016)) (stroke (width 0.254) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy -1.27 1.016) (xy 1.27 1.016)) (stroke (width 0.254) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy 0 2.54) (xy 0 1.016)) (stroke (width 0) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy 0 -2.54) (xy 0 -1.016)) (stroke (width 0) (type default)) (fill (type none)))\n')
        if art == "led":
            gfx += ('      (polyline (pts (xy 1.778 0) (xy 3.048 1.27)) (stroke (width 0) (type default)) (fill (type none)))\n'
                    '      (polyline (pts (xy 1.778 -1.016) (xy 3.048 0.254)) (stroke (width 0) (type default)) (fill (type none)))\n')
    elif art == "sw":
        gfx = ('      (polyline (pts (xy 0 2.54) (xy 0 1.27) (xy -1.524 -1.016)) (stroke (width 0.254) (type default)) (fill (type none)))\n'
               '      (polyline (pts (xy 0 -2.54) (xy 0 -1.27)) (stroke (width 0.254) (type default)) (fill (type none)))\n')
    names = {"led": ("K", "A"), "diode": ("K", "A")}.get(art, ("~", "~"))
    body = f'    (symbol "{kind}_0_1"\n{gfx}    )\n    (symbol "{kind}_1_1"\n'
    body += pin_sexpr("1", names[0], "passive", 0, 5.08, 270)
    body += pin_sexpr("2", names[1], "passive", 0, -5.08, 90)
    body += "    )\n"
    pins = {"1": (0, -5.08, 90), "2": (0, 5.08, 270)}   # schematic coords (y down), label angle
    return body, pins, ("hide",)


def box(kind, pins):
    """Rectangle with pins on its four sides. Returns body, pin positions."""
    sides = defaultdict(list)
    for num, (name, et, side) in pins.items():
        sides[side].append((num, name, et))

    def key(p):
        n = p[0]
        return (0, int(n)) if n.isdigit() else (1, n)
    for s in sides:
        if kind != "RP2354B":
            sides[s].sort(key=key)
    if kind == "RP2354B":
        for s in ("L", "R"):
            sides[s].sort(key=lambda p: (0, int(p[1][4:].split("/")[0])) if p[1].startswith("GPIO") else (1, p[1]))
    maxl = max([len(p[1]) for p in sides["L"]] or [0])
    maxr = max([len(p[1]) for p in sides["R"]] or [0])
    tb = max(len(sides["T"]), len(sides["B"]))
    w = max((maxl + maxr) * 0.9 + 6, tb * 2 * G + 2 * G, 10.16)
    w = round(w / (2 * G) + 0.5) * 2 * G
    h = max(len(sides["L"]), len(sides["R"])) * G + 2 * G
    h = max(round(h / (2 * G) + 0.5) * 2 * G, 7.62)
    body = f'    (symbol "{kind}_0_1"\n      (rectangle (start {fmt(-w / 2)} {fmt(h / 2)}) (end {fmt(w / 2)} {fmt(-h / 2)}) (stroke (width 0.254) (type default)) (fill (type background)))\n    )\n'
    body += f'    (symbol "{kind}_1_1"\n'
    pos = {}
    L = G * 2
    for s in ("L", "R"):
        lst = sides[s]
        y0 = (len(lst) - 1) * G / 2
        y0 = round(y0 / G) * G
        for i, (num, name, et) in enumerate(lst):
            y = y0 - i * G
            x = -w / 2 - L if s == "L" else w / 2 + L
            ang = 0 if s == "L" else 180
            body += pin_sexpr(num, name, et, x, y, ang, L)
            pos[num] = (x, -y, 180 if s == "L" else 0)
    for s in ("T", "B"):
        lst = sides[s]
        x0 = -(len(lst) - 1) * G
        x0 = round(x0 / G) * G
        for i, (num, name, et) in enumerate(lst):
            x = x0 + i * 2 * G
            y = h / 2 + L if s == "T" else -h / 2 - L
            ang = 270 if s == "T" else 90
            body += pin_sexpr(num, name, et, x, y, ang, L)
            pos[num] = (x, -y, 90 if s == "T" else 270)
    body += "    )\n"
    return body, pos, h, w


SYMS = {}   # kind -> dict(text, pins, h)


def build_symbols():
    lib = "(kicad_symbol_lib (version 20220914) (generator showfc_gen)\n"
    for kind, d in KINDS.items():
        name = NAMES.get(kind, kind)
        h = 10.16
        if kind == "PWR_FLAG":
            body = (f'    (symbol "{name}_0_0"\n'
                    '      (pin power_out line (at 0 0 90) (length 0) hide (name "pwr" (effects (font (size 1.27 1.27)))) (number "1" (effects (font (size 1.27 1.27)))))\n'
                    '    )\n'
                    f'    (symbol "{name}_0_1"\n'
                    '      (polyline (pts (xy 0 0) (xy 0 1.27) (xy -1.016 1.905) (xy 0 2.54) (xy 1.016 1.905) (xy 0 1.27)) (stroke (width 0) (type default)) (fill (type none)))\n'
                    '    )\n')
            pos = {"1": (0, 0, 270)}
            power = " (power)"
            extra = ""
        elif "two" in d:
            body, pos, _ = two_pin(name, d["two"])
            power = ""
            extra = " (pin_numbers hide) (pin_names (offset 0) hide)" if d["two"] not in ("led", "diode") else " (pin_numbers hide) (pin_names (offset 1.016) hide)"
        elif kind in ("TP", "HOLE"):
            body = (f'    (symbol "{name}_0_1"\n'
                    '      (circle (center 0 0) (radius 0.762) (stroke (width 0.254) (type default)) (fill (type none)))\n    )\n')
            pos = {}
            if kind == "TP":
                body += f'    (symbol "{name}_1_1"\n' + pin_sexpr("1", "1", "passive", 0, -3.81, 90, 2.54) + "    )\n"
                pos = {"1": (0, 3.81, 270)}
            power = ""
            extra = " (pin_numbers hide) (pin_names hide)"
        else:
            body, pos, h, w = box(name, d["pins"])
            power = ""
            extra = " (pin_names (offset 0.762))"
        ref = d["ref"]
        txt = f'  (symbol "{name}"{power}{extra} (in_bom {"no" if kind in ("PWR_FLAG", "HOLE") else "yes"}) (on_board {"no" if kind == "PWR_FLAG" else "yes"})\n'
        props = [("Reference", ref, 1.27 + h / 2), ("Value", name, -1.27 - h / 2), ("Footprint", "", 0), ("Datasheet", "", 0)]
        for i, (k, v, y) in enumerate(props):
            hide = " hide" if k in ("Footprint", "Datasheet") or kind == "PWR_FLAG" and k == "Reference" else ""
            txt += f'    (property "{k}" "{v}" (at 0 {fmt(y)} 0) (effects (font (size 1.27 1.27)){hide}))\n'
        txt += body + "  )\n"
        SYMS[kind] = dict(name=name, text=txt, pins=pos, h=h)
        lib += txt
    lib += ")\n"
    with open(os.path.join(ROOT, LIB + ".kicad_sym"), "w") as f:
        f.write(lib)


# ---------------------------------------------------------------------------
# Sheet layout. Each block gets a region; parts flow in rows inside it.

REGIONS = {   # group: (x, y, width) in mm on the A1 sheet
    "MCU": (30, 40, 380),
    "USB": (440, 40, 160),
    "Power": (620, 40, 200),
    "Sensors": (440, 230, 380),
    "Storage": (440, 400, 180),
    "IO": (30, 330, 400),
    "Mechanical": (650, 400, 150),
}


def snap(v):
    return round(v / G) * G


def label(net, x, y, ang):
    # global labels, so the schematic's net names match the board's ("GND", not "/GND")
    just = {0: "left", 90: "left", 180: "right", 270: "right"}[ang]
    return (f'  (global_label "{net}" (shape passive) (at {fmt(x)} {fmt(y)} {ang}) (fields_autoplaced)\n'
            f'    (effects (font (size 1.27 1.27)) (justify {just}))\n    (uuid {uid("lbl", net, x, y)})\n  )\n')


def build_schematic():
    root = uid("root")
    out = []
    out.append(f'(kicad_sch (version 20230121) (generator eeschema)\n\n  (uuid {root})\n\n  (paper "A1")\n\n')
    out.append('  (title_block\n    (title "Show FC (2-layer, RP2354B)")\n    (date "2026-10-24")\n    (rev "A")\n'
               '    (company "drone-show")\n    (comment 1 "Generated by tools/make_schematic.py from tools/design.py; do not edit by hand")\n'
               '    (comment 2 "Derived from OpenFC-Lite rev3.3 (CERN-OHL-S-2.0)")\n  )\n\n')
    out.append("  (lib_symbols\n")
    used = {p.kind for p in design.PARTS} | {"PWR_FLAG"}
    for kind in KINDS:
        if kind in used:
            t = SYMS[kind]["text"].replace(f'(symbol "{SYMS[kind]["name"]}"', f'(symbol "{LIB}:{SYMS[kind]["name"]}"', 1)
            out.append(t.replace("\n  ", "\n    ").replace("  (symbol", "    (symbol", 1))
    out.append("  )\n\n")

    body = []
    texts = []
    nets = design.nets()
    by_group = defaultdict(list)
    for p in design.PARTS:
        by_group[p.group].append(p)

    sym_uuid = {}
    for grp, parts in by_group.items():
        gx, gy, gw = REGIONS[grp]
        texts.append(f'  (text "{grp}" (at {fmt(gx)} {fmt(gy - 12)} 0)\n    (effects (font (size 5 5) (thickness 1) bold) (justify left bottom))\n    (uuid {uid("txt", grp)})\n  )\n')
        x, y = gx + 20, gy + 10
        row_h = 0
        big = [p for p in parts if p.kind not in ("R", "C", "L", "LED", "D", "TVS", "SW", "TP", "HOLE")]
        small = [p for p in parts if p not in big]
        for p in big + small:
            s = SYMS[p.kind]
            pw = 50 if p in big else 17.78
            ph = s["h"] + 30 if p in big else 33
            if p.kind == "RP2354B":
                pw, ph = 160, s["h"] + 30
            if x + pw > gx + gw:
                x = gx + 20
                y += row_h
                row_h = 0
            cx, cy = snap(x + (pw / 2 if p in big else 0)), snap(y + ph / 2)
            if p in small:
                cy = snap(y + 15)
            u = uid("sym", p.ref)
            sym_uuid[p.ref] = u
            props = [("Reference", p.ref), ("Value", p.value), ("Footprint", p.footprint), ("Datasheet", ""),
                     ("MPN", p.mpn), ("Manufacturer", p.mfr), ("LCSC", p.lcsc), ("Note", p.note)]
            sb = (f'  (symbol (lib_id "{LIB}:{s["name"]}") (at {fmt(cx)} {fmt(cy)} 0) (unit 1)\n'
                  f'    (in_bom {"no" if p.kind == "HOLE" else "yes"}) (on_board yes) (dnp no) (fields_autoplaced)\n    (uuid {u})\n')
            for i, (k, v) in enumerate(props):
                if k == "Reference":
                    at = (cx + 2.54, cy - s["h"] / 2 - 2.54) if p in big else (cx + 2.54, cy - 1.27)
                elif k == "Value":
                    at = (cx + 2.54, cy + s["h"] / 2 + 3.81) if p in big else (cx + 2.54, cy + 1.27)
                else:
                    at = (cx, cy)
                hide = "" if k in ("Reference", "Value") else " hide"
                v = v.replace('"', "'")
                sb += f'    (property "{k}" "{v}" (at {fmt(at[0])} {fmt(at[1])} 0)\n      (effects (font (size 1.27 1.27)) (justify left){hide})\n    )\n'
            for num in KINDS[p.kind].get("pins", {}) or {"1": 0, "2": 0}:
                if p.kind in ("HOLE",):
                    continue
                sb += f'    (pin "{num}" (uuid {uid("pin", p.ref, num)}))\n'
            sb += (f'    (instances\n      (project "{PROJECT}"\n        (path "/{root}" (reference "{p.ref}") (unit 1))\n      )\n    )\n  )\n')
            body.append(sb)
            for num, net in p.pins.items():
                px, py, lang = s["pins"][num]
                ax, ay = cx + px, cy + py
                if net == "NC":
                    body.append(f'  (no_connect (at {fmt(ax)} {fmt(ay)}) (uuid {uid("nc", p.ref, num)}))\n')
                else:
                    body.append(label(net, ax, ay, lang))
            x += pw
            row_h = max(row_h, ph)

    # Power flags on nets with a power input pin but no power output pin
    flags = []
    for net, pins in sorted(nets.items()):
        types = set()
        for ref, num in pins:
            p = next(q for q in design.PARTS if q.ref == ref)
            kp = KINDS[p.kind].get("pins")
            if kp and num in kp:
                types.add(kp[num][1])
        if PWR in types and "power_out" not in types:
            flags.append(net)
    fx, fy = 650, 470
    for i, net in enumerate(flags):
        x, y = snap(fx + (i % 6) * 25), snap(fy + (i // 6) * 20)
        u = uid("flag", net)
        body.append(f'  (symbol (lib_id "{LIB}:PWR_FLAG") (at {fmt(x)} {fmt(y)} 0) (unit 1)\n'
                    f'    (in_bom no) (on_board no) (dnp no)\n    (uuid {u})\n'
                    f'    (property "Reference" "#FLG{i + 1:02d}" (at {fmt(x)} {fmt(y - 4)} 0) (effects (font (size 1.27 1.27)) hide))\n'
                    f'    (property "Value" "PWR_FLAG" (at {fmt(x + 2)} {fmt(y - 3)} 0) (effects (font (size 1.27 1.27)) (justify left)))\n'
                    f'    (property "Footprint" "" (at {fmt(x)} {fmt(y)} 0) (effects (font (size 1.27 1.27)) hide))\n'
                    f'    (property "Datasheet" "" (at {fmt(x)} {fmt(y)} 0) (effects (font (size 1.27 1.27)) hide))\n'
                    f'    (pin "1" (uuid {uid("flagpin", net)}))\n'
                    f'    (instances\n      (project "{PROJECT}"\n        (path "/{root}" (reference "#FLG{i + 1:02d}") (unit 1))\n      )\n    )\n  )\n')
        body.append(label(net, x, y, 270))
    texts.append(f'  (text "Power flags" (at {fx} {fy - 12} 0)\n    (effects (font (size 3 3) (thickness 0.6) bold) (justify left bottom))\n    (uuid {uid("txt", "flags")})\n  )\n')

    out += texts + body
    out.append(f'\n  (sheet_instances\n    (path "/" (page "1"))\n  )\n)\n')
    with open(os.path.join(ROOT, PROJECT + ".kicad_sch"), "w") as f:
        f.write("".join(out))
    return root, sym_uuid


def write_tables():
    with open(os.path.join(ROOT, "sym-lib-table"), "w") as f:
        f.write('(sym_lib_table\n  (lib (name "showfc")(type "KiCad")(uri "${KIPRJMOD}/showfc.kicad_sym")(options "")(descr "Show FC project symbols, generated"))\n)\n')
    with open(os.path.join(ROOT, "fp-lib-table"), "w") as f:
        f.write('(fp_lib_table\n  (lib (name "showfc")(type "KiCad")(uri "${KIPRJMOD}/showfc.pretty")(options "")(descr "Show FC project footprints, generated"))\n)\n')


if __name__ == "__main__":
    build_symbols()
    root, _ = build_schematic()
    write_tables()
    print("schematic written, root", root)
