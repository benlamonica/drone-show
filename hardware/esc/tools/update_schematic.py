"""One-shot conversion of the OpenESC-30x30 schematic to the 2-layer drone-esc.

Run from hardware/esc with kicad-skip installed. It swaps every passive to 0603
or larger, points every footprint at the local drone-esc library or the stock
KiCad libraries, and adds the bulk ceramic bank as real schematic symbols (the
prototype kept them board-only).
"""
import re
import skip

LOCAL = "drone-esc"

RES_MPN = {
    "10k": "RC0603FR-0710KL", "1k": "RC0603FR-071KL", "220R": "RC0603FR-07220RL",
    "15R": "RC0603FR-0715RL", "10R": "RC0603FR-0710RL", "115k": "RC0603FR-07115KL",
}
# value -> (footprint, MPN, manufacturer, note)
CAP = {
    "100nF": ("Capacitor_SMD:C_0603_1608Metric", "CL10B104KB8NNNC", "Samsung", "100nF 50V X7R 0603"),
    "22uF": ("Capacitor_SMD:C_0805_2012Metric", "CL21A226MAQNNNE", "Samsung", "22uF 25V X5R 0805"),
    "1uF": ("Capacitor_SMD:C_0603_1608Metric", "CL10B105KB8NNNC", "Samsung", "1uF 50V X7R 0603"),
    "4.7uF": ("Capacitor_SMD:C_1206_3216Metric", "GRM31CR71H475KA12L", "Murata", "4.7uF 50V X7R 1206"),
    "10uF": ("Capacitor_SMD:C_1206_3216Metric", "GRM31CR61H106MA12L", "Murata", "10uF 50V X5R 1206"),
}
IC_FP = {
    "AT32F421G8U7": f"{LOCAL}:QFN-28_L4.0-W4.0-P0.40-TL-EP2.4",
    "NSG2065Q": f"{LOCAL}:QFN-24_L4.0-W4.0-P0.50-TL-EP2.8",
    "SP40N01GHNK": f"{LOCAL}:PDFN-8L_L6.0-W5.0-P1.27",
    "INA186A3IDCKR": f"{LOCAL}:SOT-363_SC-70-6",
    "LMR54406DBVR": f"{LOCAL}:SOT-23-6_L2.9-W1.6-P0.95-LS2.8-BR",
    "TLV76733DRVR": f"{LOCAL}:WSON-6_L2.0-W2.0-P0.65-TL-EP",
    "FTC160808S4R7MBCA": f"{LOCAL}:IND-SMD_L1.6-W0.8_FTC160865SR47MBCA",
    "SM08B-SRSS-TB": f"{LOCAL}:JST_SH_BM08B-SRSS-TB_Vertical",
    "0.2mR": "Resistor_SMD:R_2512_6332Metric",
    "TestPoint": "TestPoint:TestPoint_Pad_D1.5mm",
}


def setprop(sym, name, value):
    if hasattr(sym.property, name):
        getattr(sym.property, name).value = value


def convert(path):
    s = skip.Schematic(path)
    for sym in s.symbol:
        lib = sym.lib_id.value
        if lib.startswith("power:"):
            continue
        ref = sym.property.Reference.value
        val = sym.property.Value.value
        if ref.startswith("U3"):
            setprop(sym, "Footprint", f"{LOCAL}:ESC_Pads_Octagon60")
        elif ref.startswith("R") and val in RES_MPN:
            setprop(sym, "Footprint", "Resistor_SMD:R_0603_1608Metric")
            setprop(sym, "MPN", RES_MPN[val])
            setprop(sym, "Manufacturer", "YAGEO")
            setprop(sym, "LCSC", "")
        elif ref.startswith("C") and val in CAP:
            fp, mpn, mfr, note = CAP[val]
            setprop(sym, "Footprint", fp)
            setprop(sym, "MPN", mpn)
            setprop(sym, "Manufacturer", mfr)
            setprop(sym, "Datasheet", "")
            setprop(sym, "Description", note)
            setprop(sym, "LCSC", "")
        elif val in IC_FP:
            setprop(sym, "Footprint", IC_FP[val])
            if val == "SM08B-SRSS-TB":
                # top-entry version: the board sits under the FC and the cable goes straight up
                sym.property.Value.value = "BM08B-SRSS-TB"
                setprop(sym, "MPN", "BM08B-SRSS-TB(LF)(SN)")
                setprop(sym, "Manufacturer", "JST")
                setprop(sym, "LCSC", "C160390")
        else:
            print("untouched", ref, val, lib)
    return s


def add_bulk_caps(s, count=24, first=100):
    """Bulk 10uF 50V 1206 bank across +BATT/GND, drawn in a grid on the root sheet."""
    proto = s.symbol.C2
    batt = next(x for x in s.symbol if x.lib_id.value == "power:+BATT" and x.at.value[2] == 0)
    gnd = next(x for x in s.symbol if x.lib_id.value == "power:GND" and x.at.value[2] == 0)
    pwr = 200
    for i in range(count):
        x = 30.48 + (i % 12) * 10.16
        y = 190.5 + (i // 12) * 25.4
        c = proto.clone()
        c.move(x, y, 0)
        c.property.Reference.value = f"C{first + i}"
        c.property.Value.value = "10uF"
        fp, mpn, mfr, note = CAP["10uF"]
        c.property.MPN.value = mpn
        c.property.Manufacturer.value = mfr
        c.property.Description.value = note
        c.instances.project.path.reference.value = f"C{first + i}"
        b = batt.clone()
        b.move(x, y - 3.81, 0)
        b.property.Reference.value = f"#PWR{pwr}"
        b.instances.project.path.reference.value = f"#PWR{pwr}"
        g = gnd.clone()
        g.move(x, y + 3.81, 0)
        g.property.Reference.value = f"#PWR{pwr + 1}"
        g.instances.project.path.reference.value = f"#PWR{pwr + 1}"
        pwr += 2


if __name__ == "__main__":
    import shutil
    import sys
    proto = sys.argv[1]  # path to OpenESC-30x30/hardware
    shutil.copy(f"{proto}/4in1.kicad_sch", "drone-esc.kicad_sch")
    shutil.copy(f"{proto}/ESC.kicad_sch", "ESC.kicad_sch")
    root = convert("drone-esc.kicad_sch")
    add_bulk_caps(root)
    root.write("drone-esc.kicad_sch")
    ch = convert("ESC.kicad_sch")
    ch.write("ESC.kicad_sch")
    for f in ("drone-esc.kicad_sch", "ESC.kicad_sch"):
        txt = open(f).read().replace('(project "4in1"', '(project "drone-esc"')
        open(f, "w").write(txt)
