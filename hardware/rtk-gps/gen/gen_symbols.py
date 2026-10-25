"""Write lib/rtk_gps.kicad_sym: NEO-F10N and MMC5983MA (not in the KiCad library)."""
import sys, uuid

def sym(name, ref, fp, ds, desc, left, right, w=15.24):
    n = max(len(left), len(right))
    h = (n + 1) * 2.54
    top = round(h / 2 / 2.54) * 2.54
    out = [f'\t(symbol "{name}"', '\t\t(exclude_from_sim no) (in_bom yes) (on_board yes)']
    for k, v, y, hide in (("Reference", ref, top + 1.27, False), ("Value", name, -top - 1.27, False),
                          ("Footprint", fp, -top - 3.81, True), ("Datasheet", ds, -top - 6.35, True),
                          ("Description", desc, -top - 8.89, True)):
        out.append(f'\t\t(property "{k}" "{v}" (at 0 {y:.2f} 0) (effects (font (size 1.27 1.27)){" (hide yes)" if hide else ""}))')
    out.append(f'\t\t(symbol "{name}_0_1" (rectangle (start {-w/2:.2f} {top:.2f}) (end {w/2:.2f} {-top:.2f}) '
               '(stroke (width 0.254) (type default)) (fill (type background))))')
    out.append(f'\t\t(symbol "{name}_1_1"')
    for side, pins in (("L", left), ("R", right)):
        for i, p in enumerate(pins):
            if p is None:
                continue
            num, pname, typ = p
            y = top - 2.54 * (i + 1)
            x = -w / 2 - 2.54 if side == "L" else w / 2 + 2.54
            ang = 0 if side == "L" else 180
            out.append(f'\t\t\t(pin {typ} line (at {x:.2f} {y:.2f} {ang}) (length 2.54) '
                       f'(name "{pname}" (effects (font (size 1.27 1.27)))) '
                       f'(number "{num}" (effects (font (size 1.27 1.27)))))')
    out.append('\t\t)\n\t\t(embedded_fonts no)\n\t)')
    return "\n".join(out)

neo = sym("NEO-F10N", "U", "RF_GPS:ublox_NEO",
          "https://content.u-blox.com/sites/default/files/documents/NEO-F10N_DataSheet_UBX-23002117.pdf",
          "u-blox F10 L1/L5 GNSS module, NEO LCC 12.2x16 mm, UART; -00B-20 adds carrier phase RAWX",
          [("23", "VCC", "power_in"), ("22", "V_BCKP", "power_in"), None,
           ("21", "RXD", "input"), ("20", "TXD", "output"), None,
           ("8", "~{RESET}", "input"), ("1", "~{SAFEBOOT}", "input"), ("4", "EXTINT", "input"),
           ("3", "TIMEPULSE", "output"), None,
           ("10", "GND", "power_in"), ("12", "GND", "passive"), ("13", "GND", "passive"), ("24", "GND", "passive")],
          [("11", "RF_IN", "input"), ("9", "VCC_RF", "power_out"), ("14", "LNA_EN", "output"), None,
           ("2", "RESERVED", "no_connect"), ("5", "RESERVED", "no_connect"), ("6", "RESERVED", "no_connect"),
           ("7", "RESERVED", "no_connect"), ("15", "RESERVED", "no_connect"), ("16", "RESERVED", "no_connect"),
           ("17", "RESERVED", "no_connect"), ("18", "RESERVED", "no_connect"), ("19", "RESERVED", "no_connect")],
          w=20.32)
mmc = sym("MMC5983MA", "U", "Package_LGA:LGA-16_3x3mm_P0.5mm",
          "https://www.memsic.com/Public/Uploads/uploadfile/files/20220119/MMC5983MADatasheetRevA.pdf",
          "MEMSIC 3-axis AMR magnetometer, I2C (0x30) / SPI, LGA-16 3x3 mm",
          [("2", "VDD", "power_in"), ("10", "CAP", "passive"), ("13", "VDDIO", "power_in"), None,
           ("9", "GND", "power_in"), ("11", "GND", "passive")],
          [("1", "SCL/SCK", "input"), ("16", "SDA/SDI", "bidirectional"), ("5", "SDO", "output"),
           ("4", "~{CS}", "input"), ("15", "INT", "output"), None,
           ("3", "NC", "no_connect"), ("6", "NC", "no_connect"), ("7", "NC", "no_connect"),
           ("8", "NC", "no_connect"), ("12", "NC", "no_connect"), ("14", "NC", "no_connect")],
          w=15.24)

with open(sys.argv[1], "w") as f:
    f.write('(kicad_symbol_lib\n\t(version 20241209)\n\t(generator "kicad_symbol_editor")\n\t(generator_version "9.0")\n')
    f.write(neo + "\n" + mmc + "\n)\n")
