"""Write lightctl.kicad_sym and light-controller.kicad_sch from design.PARTS.

Every pin gets a short wire and a net label, and unused pins get a no-connect flag,
so the schematic is fully connected by label without hand-drawn wiring. Blocks
(power, buck, MCU, I/O, radio, the four LED channels) each get their own column group
with a heading.
"""
import math
import os
import sys
import uuid

sys.path.insert(0, os.path.dirname(__file__))
import design  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PROJECT = "light-controller"
G = 2.54
NS = uuid.UUID("6f1c3a52-7b1e-4f0e-9a51-1c2d3e4f5a6b")


def uid(*key):
    return str(uuid.uuid5(NS, "/".join(map(str, key))))


def q(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def font(hide=False, justify=None):
    j = f" (justify {justify})" if justify else ""
    h = " hide" if hide else ""
    return f"(effects (font (size 1.27 1.27)){j}{h})"


# ---------------------------------------------------------------------------------
# Symbol geometry
# ---------------------------------------------------------------------------------
class Sym:
    def __init__(self, name):
        self.name = name
        self.pins = []          # (num, name, x, y, angle)  symbol coords, y up
        self.gfx = []
        self.show_names = True
        if name in design.TWO:
            self._two(design.TWO[name])
        else:
            self._ic(*design.ICS[name])

    def _two(self, pins):
        self.show_names = False
        if len(pins) == 1:
            self.pins = [(pins[0][0], pins[0][1], -2 * G, 0, 0)]
            self.gfx = ["(circle (center 0 0) (radius 0.762) (stroke (width 0.254) (type default)) (fill (type none)))",
                        "(polyline (pts (xy -2.54 0) (xy -0.762 0)) (stroke (width 0.254) (type default)) (fill (type none)))"]
            self.box = (-2 * G, -G, G, G)
            return
        self.pins = [(pins[0][0], pins[0][1], -1.5 * G, 0, 0), (pins[1][0], pins[1][1], 1.5 * G, 0, 180)]
        n = self.name
        st = "(stroke (width 0.254) (type default))"
        if n in ("R", "Thermistor_NTC"):
            self.gfx = [f"(rectangle (start -2.032 -0.762) (end 2.032 0.762) {st} (fill (type none)))"]
            if n == "Thermistor_NTC":
                self.gfx.append(f"(polyline (pts (xy -2.032 -1.524) (xy -1.27 -1.524) (xy 1.524 1.524)) {st} (fill (type none)))")
        elif n == "C":
            self.gfx = [f"(polyline (pts (xy -0.508 -1.524) (xy -0.508 1.524)) {st} (fill (type none)))",
                        f"(polyline (pts (xy 0.508 -1.524) (xy 0.508 1.524)) {st} (fill (type none)))",
                        f"(polyline (pts (xy -2.54 0) (xy -0.508 0)) {st} (fill (type none)))",
                        f"(polyline (pts (xy 0.508 0) (xy 2.54 0)) {st} (fill (type none)))"]
        elif n == "L":
            self.gfx = [f"(arc (start {-2.54 + 1.27 * i:g} 0) (mid {-1.905 + 1.27 * i:g} 0.635) (end {-1.27 + 1.27 * i:g} 0) {st} (fill (type none)))"
                        for i in range(4)]
        elif n == "SW_Push":
            self.gfx = [f"(circle (center -2.032 0) (radius 0.508) {st} (fill (type none)))",
                        f"(circle (center 2.032 0) (radius 0.508) {st} (fill (type none)))",
                        f"(polyline (pts (xy -2.54 1.27) (xy 2.54 1.27)) {st} (fill (type none)))",
                        f"(polyline (pts (xy 0 1.27) (xy 0 2.54)) {st} (fill (type none)))"]
        else:   # diodes: pin 1 = K on the left, so the bar is on the left
            self.gfx = [f"(polyline (pts (xy 1.27 1.27) (xy 1.27 -1.27) (xy -1.27 0) (xy 1.27 1.27)) {st} (fill (type none)))",
                        f"(polyline (pts (xy -1.27 1.27) (xy -1.27 -1.27)) {st} (fill (type none)))",
                        f"(polyline (pts (xy -2.54 0) (xy 2.54 0)) {st} (fill (type none)))"]
            if n == "D_Zener":
                self.gfx[1] = f"(polyline (pts (xy -1.778 1.27) (xy -1.27 1.27) (xy -1.27 -1.27) (xy -0.762 -1.27)) {st} (fill (type none)))"
            if n == "D_Schottky":
                self.gfx[1] = f"(polyline (pts (xy -1.778 0.762) (xy -1.778 1.27) (xy -1.27 1.27) (xy -1.27 -1.27) (xy -0.762 -1.27) (xy -0.762 -0.762)) {st} (fill (type none)))"
            if n == "D_TVS":
                self.gfx[1] = f"(polyline (pts (xy -1.778 1.778) (xy -1.27 1.27) (xy -1.27 -1.27) (xy -0.762 -1.778)) {st} (fill (type none)))"
            if n == "LED":
                self.gfx += [f"(polyline (pts (xy -0.508 1.778) (xy 0.508 2.794)) {st} (fill (type none)))",
                             f"(polyline (pts (xy 0.508 1.778) (xy 1.524 2.794)) {st} (fill (type none)))"]
        self.box = (-1.5 * G, -G, 1.5 * G, G)

    def _ic(self, left, right):
        nl, nr = len(left), len(right)
        n = max(nl, nr, 2)
        top = math.ceil((n - 1) / 2) * G
        lw = max([len(p[1]) for p in left] + [0])
        rw = max([len(p[1]) for p in right] + [0])
        w = max(4, math.ceil(((lw + rw) * 1.27 * 0.75 + 4) / (2 * G))) * G   # half width, on grid
        for i, (num, name) in enumerate(left):
            self.pins.append((num, name, -w - G, top - i * G, 0))
        for i, (num, name) in enumerate(right):
            self.pins.append((num, name, w + G, top - i * G, 180))
        bot = top - (n - 1) * G
        self.gfx = [f"(rectangle (start {-w:g} {top + G:g}) (end {w:g} {bot - G:g}) "
                    "(stroke (width 0.254) (type default)) (fill (type background)))"]
        self.box = (-w - G, bot - G, w + G, top + G)

    def lib_entry(self, prefix=""):
        n = self.name
        ref = "U"
        if n in design.TWO:
            ref = {"R": "R", "C": "C", "L": "L", "LED": "D", "Thermistor_NTC": "TH", "SW_Push": "SW",
                   "TestPoint": "TP"}.get(n, "D")
        names = "" if self.show_names else " (pin_names hide)"
        nums = " (pin_numbers hide)" if n in design.TWO else ""
        out = [f"(symbol {q(prefix + n)}{nums}{names} (in_bom yes) (on_board yes)",
               f"  (property \"Reference\" {q(ref)} (at 0 {self.box[3] + 1.27:g} 0) {font()})",
               f"  (property \"Value\" {q(n)} (at 0 {self.box[1] - 1.27:g} 0) {font()})",
               f"  (property \"Footprint\" \"\" (at 0 0 0) {font(True)})",
               f"  (property \"Datasheet\" \"~\" (at 0 0 0) {font(True)})",
               f"  (symbol {q(n + '_0_1')}"] + ["    " + g for g in self.gfx] + ["  )", f"  (symbol {q(n + '_1_1')}"]
        for num, name, x, y, a in self.pins:
            ln = 1.27 if n in design.TWO else G
            out.append(f"    (pin passive line (at {x:g} {y:g} {a}) (length {ln:g}) "
                       f"(name {q(name)} {font()}) (number {q(num)} {font()}))")
        out += ["  )", ")"]
        return "\n".join(out)


def snap(v):
    return round(v / G) * G


def build():
    syms = {p["sym"]: Sym(p["sym"]) for p in design.PARTS}
    # library file
    lib = ["(kicad_symbol_lib (version 20220914) (generator gen_sch)"]
    for s in syms.values():
        lib.append(s.lib_entry())
    lib.append(")")
    open(os.path.join(ROOT, "lightctl.kicad_sym"), "w").write("\n".join(lib) + "\n")

    root = uid("root")
    items, placed = [], {}
    titles = {"power": "Power input, protection, sensing", "buck": "3.3 V buck", "mcu": "RP2350A",
              "io": "USB, debug, FC UART, test pads", "radio": "RM2 Wi-Fi",
              "ch_R": "Red channel", "ch_G": "Green channel", "ch_B": "Blue channel", "ch_W": "White channel"}
    order = ["power", "buck", "mcu", "io", "radio", "ch_R", "ch_G", "ch_B", "ch_W"]
    x0, y_top, y_max = 25.4, 38.1, 390
    x = x0
    for blk in order:
        parts = [p for p in design.PARTS if p["block"] == blk]
        items.append(f'(text {q(titles[blk])} (at {x:g} {y_top - 10.16:g} 0) (effects (font (size 2.54 2.54) bold) (justify left bottom)) (uuid {uid("t", blk)}))')
        col_w, y = 0, y_top
        for p in parts:
            s = syms[p["sym"]]
            bx0, by0, bx1, by1 = s.box
            labw = 4 + 1.6 * max(len(n or "") for n in p["pins"].values() if True) if any(p["pins"].values()) else 6
            w = (bx1 - bx0) + 2 * (labw + G)
            h = (by1 - by0) + 3 * G
            if y + h > y_max:
                x += col_w + 5.08
                col_w, y = 0, y_top
            cx = snap(x + labw + G - bx0)
            cy = snap(y + by1 + G)
            placed[p["ref"]] = (cx, cy)
            col_w = max(col_w, w)
            y += h
        x += col_w + 12.7
    width = x + 20
    paper = "A2" if width <= 594 else ("A1" if width <= 841 else "A0")

    for p in design.PARTS:
        s = syms[p["sym"]]
        cx, cy = placed[p["ref"]]
        fp = f"{design.LIB}:{p['fp']}"
        dnp = "yes" if p["dnp"] else "no"
        val = p["value"] + (" (DNP)" if p["dnp"] else "")
        e = [f"(symbol (lib_id {q(design.LIB + ':' + p['sym'])}) (at {cx:g} {cy:g} 0) (unit 1)",
             f"  (in_bom yes) (on_board yes) (dnp {dnp})",
             f"  (uuid {uid('sym', p['ref'])})",
             f"  (property \"Reference\" {q(p['ref'])} (at {cx:g} {cy - s.box[3] - 1.27:g} 0) {font()})",
             f"  (property \"Value\" {q(val)} (at {cx:g} {cy - s.box[1] + 1.27:g} 0) {font()})",
             f"  (property \"Footprint\" {q(fp)} (at {cx:g} {cy:g} 0) {font(True)})",
             f"  (property \"Datasheet\" \"~\" (at {cx:g} {cy:g} 0) {font(True)})",
             f"  (property \"Manufacturer\" {q(p['mfr'])} (at {cx:g} {cy:g} 0) {font(True)})",
             f"  (property \"MPN\" {q(p['mpn'])} (at {cx:g} {cy:g} 0) {font(True)})",
             f"  (property \"Side\" {q('bottom' if p.get('side') == 'B' else 'top')} (at {cx:g} {cy:g} 0) {font(True)})"]
        for num, *_ in s.pins:
            e.append(f"  (pin {q(num)} (uuid {uid('pin', p['ref'], num)}))")
        e.append(f"  (instances (project {q(PROJECT)} (path \"/{root}\" (reference {q(p['ref'])}) (unit 1))))")
        e.append(")")
        items.append("\n".join(e))
        # wires, labels, no-connects
        for num, name, px, py, a in s.pins:
            ax, ay = cx + px, cy - py
            net = p["pins"].get(num)
            if num not in p["pins"]:
                sys.exit(f"{p['ref']} pin {num} has no entry")
            if not net:
                items.append(f"(no_connect (at {ax:g} {ay:g}) (uuid {uid('nc', p['ref'], num)}))")
                continue
            dx = -1 if a == 0 else 1
            ex = ax + dx * G
            items.append(f"(wire (pts (xy {ax:g} {ay:g}) (xy {ex:g} {ay:g})) (stroke (width 0) (type default)) (uuid {uid('w', p['ref'], num)}))")
            ang, just = (180, "right bottom") if dx < 0 else (0, "left bottom")
            items.append(f"(label {q(net)} (at {ex:g} {ay:g} {ang}) (fields_autoplaced) {font(justify=just)} (uuid {uid('l', p['ref'], num)}))")
        missing = set(p["pins"]) - {pn[0] for pn in s.pins}
        if missing:
            sys.exit(f"{p['ref']}: pins {missing} not on symbol {p['sym']}")

    notes = [
        "Light controller for the drone-show quad: RP2350A + RM2 Wi-Fi, 12 Cree XP-E2 LEDs (3 per colour, R/G/B/W) on the bottom side,",
        "four AL8860 constant-current buck drivers, 4S-6S input. LED current = 0.1 V / Rsense = 0.67 A with 0.15 ohm (1.0 A max with 0.1 ohm).",
        "GPIO23/24/25/29 drive the RM2 exactly as on a Pico 2 W, so the pico2_w board definition and cyw43 driver work unchanged.",
        "CTRL pull-downs keep the LEDs dark while the RP2350 boots. Fit C51-C54 only for filtered (analog) dimming.",
    ]
    for i, t in enumerate(notes):
        items.append(f'(text {q(t)} (at 25.4 {395 + i * 5.08:g} 0) (effects (font (size 2 2)) (justify left bottom)) (uuid {uid("note", i)}))')

    sch = [f"(kicad_sch (version 20230121) (generator eeschema)",
           f"  (uuid {root})",
           f"  (paper {q(paper)})",
           "  (title_block (title \"Drone-show light controller\") (date \"2026-10-24\") (rev \"0.1\")",
           "    (comment 1 \"2-layer, 1.6 mm, PCBWay standard process\") (comment 2 \"Generated by tools/gen_sch.py from tools/design.py\"))",
           "  (lib_symbols"]
    for s in syms.values():
        sch.append("\n".join("    " + l for l in s.lib_entry(design.LIB + ":").split("\n")))
    sch.append("  )")
    sch += ["  " + it.replace("\n", "\n  ") for it in items]
    sch.append('  (sheet_instances (path "/" (page "1")))')
    sch.append(")")
    open(os.path.join(ROOT, PROJECT + ".kicad_sch"), "w").write("\n".join(sch) + "\n")
    print("schematic", paper, f"{len(design.PARTS)} parts")


def tables():
    open(os.path.join(ROOT, "sym-lib-table"), "w").write(
        '(sym_lib_table\n  (version 7)\n  (lib (name "lightctl")(type "KiCad")(uri "${KIPRJMOD}/lightctl.kicad_sym")(options "")(descr "Light controller local symbols"))\n)\n')
    open(os.path.join(ROOT, "fp-lib-table"), "w").write(
        '(fp_lib_table\n  (version 7)\n  (lib (name "lightctl")(type "KiCad")(uri "${KIPRJMOD}/lightctl.pretty")(options "")(descr "Light controller local footprints"))\n)\n')


if __name__ == "__main__":
    tables()
    build()
