"""Light controller netlist: the single source for the schematic, the board and the BOM.

Every part is one entry in PARTS. gen_sch.py draws the schematic from it, gen_pcb.py
places and connects the footprints from it, and gen_bom.py writes the BOM. Change a
connection here and regenerate all three, so the schematic and the board never drift.
"""

LIB = "lightctl"

# Board: 73 x 35 mm, origin at the board centre. -X is the end that slides in first
# (against the rail stop), +X is the open end where the wires leave.
BOARD_L, BOARD_W = 73.0, 35.0

# ---------------------------------------------------------------------------------
# Symbols: name -> (pins, layout). Pins are (number, name). Layout "2" is a vertical
# two-terminal part, otherwise (left pin numbers, right pin numbers).
# ---------------------------------------------------------------------------------
TWO = {
    "R": [("1", "~"), ("2", "~")],
    "C": [("1", "~"), ("2", "~")],
    "L": [("1", "~"), ("2", "~")],
    "D": [("1", "K"), ("2", "A")],
    "D_Schottky": [("1", "K"), ("2", "A")],
    "D_Zener": [("1", "K"), ("2", "A")],
    "D_TVS": [("1", "K"), ("2", "A")],
    "LED": [("1", "K"), ("2", "A")],
    "Thermistor_NTC": [("1", "~"), ("2", "~")],
    "SW_Push": [("1", "1"), ("2", "2")],
    "TestPoint": [("1", "1")],
}

ICS = {
    "LED_XPE2": ([("2", "A")], [("1", "K"), ("3", "PAD")]),
    "Q_PMOS_GDS": ([("1", "G")], [("2", "D"), ("3", "S")]),
    "LMR51420": ([("3", "VIN"), ("5", "EN"), ("1", "GND")],
                 [("6", "CB"), ("2", "SW"), ("4", "FB")]),
    "AL8860": ([("8", "VIN"), ("1", "SET"), ("4", "CTRL")],
               [("5", "SW"), ("6", "SW"), ("7", "NC"), ("2", "GND"), ("3", "GND"), ("9", "EP")]),
    "W25Q16": ([("1", "~CS"), ("6", "CLK"), ("5", "IO0"), ("2", "IO1")],
               [("8", "VCC"), ("3", "IO2"), ("7", "IO3"), ("4", "GND")]),
    "Crystal_GND24": ([("1", "1")], [("3", "3"), ("2", "GND"), ("4", "GND")]),
    "USBLC6-2SC6": ([("1", "IO1"), ("3", "IO2"), ("2", "GND")],
                    [("6", "IO1"), ("4", "IO2"), ("5", "VBUS")]),
    "Conn_1x02": ([("1", "1"), ("2", "2")], []),
    "Conn_1x03": ([("1", "1"), ("2", "2"), ("3", "3")], [("MP", "MP")]),
    "USB_C_16P": ([("A4", "VBUS"), ("A9", "VBUS"), ("B4", "VBUS"), ("B9", "VBUS"),
                   ("A5", "CC1"), ("B5", "CC2"), ("A6", "D+"), ("B6", "D+"),
                   ("A7", "D-"), ("B7", "D-"), ("A8", "SBU1"), ("B8", "SBU2")],
                  [("A1", "GND"), ("A12", "GND"), ("B1", "GND"), ("B12", "GND"), ("S1", "SHIELD")]),
    "RM2": ([("16", "VIN"), ("14", "VDDIO"), ("12", "WL_ON"), ("13", "BT_ON"), ("9", "CS"),
             ("3", "CLK"), ("5", "DIN"), ("6", "DOUT"), ("10", "IRQ")],
            [("8", "GPIO0"), ("18", "GPIO1"), ("17", "GPIO2"), ("2", "NC"), ("19", "NC"), ("20", "NC"),
             ("1", "GND"), ("4", "GND"), ("7", "GND"), ("11", "GND"), ("15", "GND"), ("21", "GND")]),
    "RP2350A": (
        [("1", "IOVDD"), ("11", "IOVDD"), ("20", "IOVDD"), ("30", "IOVDD"), ("38", "IOVDD"),
         ("45", "IOVDD"), ("6", "DVDD"), ("23", "DVDD"), ("39", "DVDD"), ("44", "ADC_AVDD"),
         ("53", "USB_OTP_VDD"), ("54", "QSPI_IOVDD"), ("49", "VREG_VIN"), ("46", "VREG_AVDD"),
         ("48", "VREG_LX"), ("50", "VREG_FB"), ("47", "VREG_PGND"), ("61", "GND"),
         ("21", "XIN"), ("22", "XOUT"), ("26", "RUN"), ("24", "SWCLK"), ("25", "SWDIO"),
         ("51", "USB_DM"), ("52", "USB_DP"), ("60", "QSPI_SS"), ("56", "QSPI_SCLK"),
         ("57", "QSPI_SD0"), ("59", "QSPI_SD1"), ("58", "QSPI_SD2"), ("55", "QSPI_SD3")],
        [("2", "GPIO0"), ("3", "GPIO1"), ("4", "GPIO2"), ("5", "GPIO3"), ("7", "GPIO4"),
         ("8", "GPIO5"), ("9", "GPIO6"), ("10", "GPIO7"), ("12", "GPIO8"), ("13", "GPIO9"),
         ("14", "GPIO10"), ("15", "GPIO11"), ("16", "GPIO12"), ("17", "GPIO13"), ("18", "GPIO14"),
         ("19", "GPIO15"), ("27", "GPIO16"), ("28", "GPIO17"), ("29", "GPIO18"), ("31", "GPIO19"),
         ("32", "GPIO20"), ("33", "GPIO21"), ("34", "GPIO22"), ("35", "GPIO23"), ("36", "GPIO24"),
         ("37", "GPIO25"), ("40", "GPIO26_ADC0"), ("41", "GPIO27_ADC1"), ("42", "GPIO28_ADC2"),
         ("43", "GPIO29_ADC3")]),
}

# ---------------------------------------------------------------------------------
# Footprints (all copied into the project-local lightctl.pretty)
# ---------------------------------------------------------------------------------
F = {
    "0603R": "R_0603_1608Metric", "0603C": "C_0603_1608Metric", "0805C": "C_0805_2012Metric",
    "1206C": "C_1206_3216Metric", "1206R": "R_1206_3216Metric", "0603LED": "LED_0603_1608Metric",
    "SMB": "D_SMB", "SMA": "D_SMA", "SOD123": "D_SOD-123", "SOT223": "SOT-223-3_TabPin2",
    "SOT236": "SOT-23-6", "MSOP8EP": "MSOP-8-1EP_3x3mm_P0.65mm_EP1.68x1.88mm",
    "SOIC8": "SOIC-8_3.9x4.9mm_P1.27mm", "XTAL": "Crystal_SMD_3225-4Pin_3.2x2.5mm",
    "XAL4020": "L_Coilcraft_XAL4020-XXX", "SRN6028": "L_Bourns-SRN6028",
    "L2016": "L_Abracon_AOTA-B201610", "QFN60": "RP2350A_QFN-60_7x7mm_P0.4mm",
    "RM2": "RPi_RM2", "XPE2": "LED_Cree-XP", "USBC": "USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal",
    "SH3": "JST_SH_SM03B-SRSS-TB_1x03-1MP_P1.00mm_Horizontal",
    "WIRE2": "SolderWire-0.5sqmm_1x02_P4.6mm_D0.9mm_OD2.1mm", "BTN": "SW_SPST_PTS810",
    "TP": "TestPoint_Pad_D1.5mm",
}

# Common parts: key -> (symbol, value, footprint, manufacturer, MPN)
def _r(v, mpn): return ("R", v, "0603R", "Yageo", mpn)
P = {
    "100n": ("C", "100nF 50V X7R", "0603C", "Murata", "GRM188R71H104KA93D"),
    "4u7_50": ("C", "4.7uF 50V X7R", "1206C", "Murata", "GRM31CR71H475KA12L"),
    "4u7_10": ("C", "4.7uF 10V X5R", "0603C", "Murata", "GRM188R61A475KE15D"),
    "10u": ("C", "10uF 10V X7R", "0805C", "Murata", "GRM21BR71A106KA73L"),
    "22u": ("C", "22uF 10V X5R", "0805C", "Murata", "GRM21BR61A226ME44L"),
    "15p": ("C", "15pF 50V C0G", "0603C", "Murata", "GRM1885C1H150JA01D"),
    "1u": ("C", "1uF 16V X7R", "0603C", "Murata", "GRM188R71C105KA12D"),
    "100k": _r("100k", "RC0603FR-07100KL"), "10k": _r("10k", "RC0603FR-0710KL"),
    "22k1": _r("22.1k", "RC0603FR-0722K1L"), "1k": _r("1k", "RC0603FR-071KL"),
    "5k1": _r("5.1k", "RC0603FR-075K1L"), "27R": _r("27", "RC0603FR-0727RL"),
    "33R": _r("33", "RC0603FR-0733RL"), "470R": _r("470", "RC0603FR-07470RL"),
    "100R": _r("100", "RC0603FR-07100RL"),
    "rsense": ("R", "0.15 1% 0.25W", "1206R", "Vishay Dale", "WSL1206R1500FEA"),
    "led_ind": ("LED", "green", "0603LED", "Wurth Elektronik", "150060GS75000"),
}

# Each colour channel: LED MPN, dimming GPIO name. Sense resistor 0.15 ohm -> 0.67 A.
CHANNELS = [
    ("R", "Cree LED", "XPEBRD-L1-0000-00801 (XP-E2 red, any in-stock flux bin)"),
    ("G", "Cree LED", "XPEBGR-L1-0000-00E01 (XP-E2 green)"),
    ("B", "Cree LED", "XPEBBL-L1-0000-00Y01 (XP-E2 blue, any in-stock flux bin)"),
    ("W", "Cree LED", "XPEBWT-L1-0000-00D50 (XP-E2 cool white)"),
]

PARTS = []      # dicts: ref, sym, value, fp, mfr, mpn, pins{num: net}, block, dnp


def add(ref, kind, pins, block, dnp=False, **over):
    if kind in P:
        sym, val, fp, mfr, mpn = P[kind]
    else:
        sym, val, fp, mfr, mpn = kind
    d = dict(ref=ref, sym=sym, value=val, fp=F.get(fp, fp), mfr=mfr, mpn=mpn,
             pins=pins, block=block, dnp=dnp)
    d.update(over)
    PARTS.append(d)


def two(a, b):
    return {"1": a, "2": b}


# --- Power input: wire pads, reverse-polarity P-FET, TVS, bulk -----------------------
B = "power"
add("J1", ("Conn_1x02", "BATT 4S-6S", "WIRE2", "-", "solder 20 AWG leads"),
    {"1": "VBAT_IN", "2": "GND"}, B)
add("Q1", ("Q_PMOS_GDS", "DMP6023LE", "SOT223", "Diodes Inc", "DMP6023LE-13"),
    {"1": "FET_GATE", "2": "VBAT_IN", "3": "VIN"}, B)
add("D1", ("D_Zener", "12V", "SOD123", "Diodes Inc", "BZT52C12-7-F"), {"1": "VIN", "2": "FET_GATE"}, B)
add("R1", "100k", two("FET_GATE", "GND"), B)
add("D2", ("D_TVS", "SMBJ26A", "SMB", "Littelfuse", "SMBJ26A"), {"1": "VIN", "2": "GND"}, B)
for i in (1, 2, 3):
    add(f"C{i}", "4u7_50", two("VIN", "GND"), B)
add("C4", "100n", two("VIN", "GND"), B)
add("D3", ("D_Schottky", "B5819W", "SOD123", "MCC", "B5819W-TP"), {"1": "VIN", "2": "VBUS"}, B)
add("R2", "100k", two("VIN", "VBAT_SENSE"), B)
add("R3", "10k", two("VBAT_SENSE", "GND"), B)
add("C5", "100n", two("VBAT_SENSE", "GND"), B)

# --- 3.3 V buck ------------------------------------------------------------------------
B = "buck"
add("U1", ("LMR51420", "LMR51420YDDCR", "SOT236", "Texas Instruments", "LMR51420YDDCR"),
    {"1": "GND", "2": "BUCK_SW", "3": "VIN", "4": "BUCK_FB", "5": "BUCK_EN", "6": "BUCK_CB"}, B)
add("C6", "4u7_50", two("VIN", "GND"), B)
add("C7", "100n", two("VIN", "GND"), B)
add("R4", "100k", two("VIN", "BUCK_EN"), B)
add("C8", "100n", two("BUCK_CB", "BUCK_SW"), B)
add("L1", ("L", "6.8uH 3.6A", "XAL4020", "Coilcraft", "XAL4020-682MEC"), two("BUCK_SW", "+3V3"), B)
add("R5", "100k", two("+3V3", "BUCK_FB"), B)
add("R6", "22k1", two("BUCK_FB", "GND"), B)
add("C9", "22u", two("+3V3", "GND"), B)
add("C10", "22u", two("+3V3", "GND"), B)

# --- RP2350A ---------------------------------------------------------------------------
B = "mcu"
gpio = {
    "2": "UART_TX", "3": "UART_RX", "4": "PWM_R", "5": "PWM_G", "7": "PWM_B", "8": "PWM_W",
    "9": "STAT_LED", "12": "GPIO8", "13": "GPIO9", "27": "GPIO16",
    "35": "WL_ON", "36": "WL_D", "37": "WL_CS", "40": "NTC_SENSE", "41": "VBAT_SENSE",
    "42": "GPIO28", "43": "WL_CLK",
}
mcu = {n: "+3V3" for n in ("1", "11", "20", "30", "38", "45", "44", "53", "54", "49")}
mcu.update({n: "+1V1" for n in ("6", "23", "39", "50")})
mcu.update({"46": "VREG_AVDD", "48": "VREG_LX", "47": "GND", "61": "GND", "21": "XIN", "22": "XOUT",
            "26": "RUN", "24": "SWCLK", "25": "SWDIO", "51": "USB_DM", "52": "USB_DP",
            "60": "QSPI_SS", "56": "QSPI_SCLK", "57": "QSPI_SD0", "59": "QSPI_SD1",
            "58": "QSPI_SD2", "55": "QSPI_SD3"})
mcu.update(gpio)
for n in ("10", "14", "15", "16", "17", "18", "19", "28", "29", "31", "32", "33", "34"):
    mcu[n] = None   # unused GPIO
add("U2", ("RP2350A", "RP2350A", "QFN60", "Raspberry Pi", "RP2350A (or RP2354A, then omit U3)"), mcu, B)
for i, _ in enumerate(("1", "11", "20", "30", "38", "45")):
    add(f"C{11 + i}", "100n", two("+3V3", "GND"), B)
for i in range(3):
    add(f"C{17 + i}", "100n", two("+1V1", "GND"), B)
add("C20", "4u7_10", two("+1V1", "GND"), B)
add("C21", "4u7_10", two("+3V3", "GND"), B)          # VREG_VIN
add("R7", "33R", two("+3V3", "VREG_AVDD"), B)
add("C22", "4u7_10", two("VREG_AVDD", "GND"), B)
add("C23", "100n", two("+3V3", "GND"), B)             # ADC_AVDD
add("C24", "100n", two("+3V3", "GND"), B)             # USB_OTP_VDD
add("C25", "100n", two("+3V3", "GND"), B)             # QSPI_IOVDD
add("C26", "10u", two("+3V3", "GND"), B)
add("L2", ("L", "3.3uH", "L2016", "Abracon", "AOTA-B201610S3R3-101-T"), two("VREG_LX", "+1V1"), B)
add("Y1", ("Crystal_GND24", "12MHz", "XTAL", "Abracon", "ABM8-272-T3"),
    {"1": "XIN", "3": "XOUT_X", "2": "GND", "4": "GND"}, B)
add("R8", "1k", two("XOUT", "XOUT_X"), B)
add("C27", "15p", two("XIN", "GND"), B)
add("C28", "15p", two("XOUT_X", "GND"), B)
add("U3", ("W25Q16", "W25Q16JVSNIQ", "SOIC8", "Winbond", "W25Q16JVSNIQ"),
    {"1": "QSPI_SS", "2": "QSPI_SD1", "3": "QSPI_SD2", "4": "GND", "5": "QSPI_SD0",
     "6": "QSPI_SCLK", "7": "QSPI_SD3", "8": "+3V3"}, B)
add("C29", "100n", two("+3V3", "GND"), B)
add("R9", "10k", two("+3V3", "QSPI_SS"), B)
add("R10", "1k", two("QSPI_SS", "BOOT_BTN"), B)
add("SW1", ("SW_Push", "BOOTSEL", "BTN", "C&K", "PTS810 SJM 250 SMTR LFS"), two("BOOT_BTN", "GND"), B)
add("R11", "10k", two("+3V3", "RUN"), B)
add("R21", "1k", two("STAT_LED", "STAT_LED_A"), B)
add("D5", "led_ind", {"1": "GND", "2": "STAT_LED_A"}, B)

# --- USB, debug, UART, test pads -------------------------------------------------------
B = "io"
add("J2", ("USB_C_16P", "USB-C", "USBC", "GCT", "USB4105-GF-A"),
    {"A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS", "A5": "CC1", "B5": "CC2",
     "A6": "USB_CONN_DP", "B6": "USB_CONN_DP", "A7": "USB_CONN_DM", "B7": "USB_CONN_DM",
     "A8": None, "B8": None, "A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "S1": "GND"}, B)
add("R12", "5k1", two("CC1", "GND"), B)
add("R13", "5k1", two("CC2", "GND"), B)
add("R14", "27R", two("USB_CONN_DP", "USB_DP"), B)
add("R15", "27R", two("USB_CONN_DM", "USB_DM"), B)
add("U4", ("USBLC6-2SC6", "USBLC6-2SC6", "SOT236", "STMicroelectronics", "USBLC6-2SC6"),
    {"1": "USB_CONN_DP", "6": "USB_CONN_DP", "3": "USB_CONN_DM", "4": "USB_CONN_DM",
     "2": "GND", "5": "VBUS"}, B)
add("J3", ("Conn_1x03", "SWD", "SH3", "JST", "SM03B-SRSS-TB(LF)(SN)"),
    {"1": "SWCLK", "2": "GND", "3": "SWDIO", "MP": "GND"}, B)
add("J4", ("Conn_1x03", "UART to FC", "SH3", "JST", "SM03B-SRSS-TB(LF)(SN)"),
    {"1": "UART_TX_C", "2": "UART_RX_C", "3": "GND", "MP": "GND"}, B)
add("R16", "100R", two("UART_TX", "UART_TX_C"), B)
add("R17", "100R", two("UART_RX_C", "UART_RX"), B)
for i, net in enumerate(("RUN", "GPIO8", "GPIO9", "GPIO16", "GPIO28")):
    add(f"TP{i + 1}", ("TestPoint", net, "TP", "-", "test pad"), {"1": net}, B)

# --- RM2 radio -------------------------------------------------------------------------
B = "radio"
add("M1", ("RM2", "RM2", "RM2", "Raspberry Pi", "SC1169 (Radio Module 2)"),
    {"16": "+3V3", "14": "+3V3", "12": "WL_ON", "13": "WL_ON", "9": "WL_CS", "3": "WL_CLK",
     "5": "WL_D", "6": "WL_DO_R", "10": "WL_IRQ_R", "8": "WL_GPIO0", "18": None, "17": None,
     "2": None, "19": None, "20": None,
     "1": "GND", "4": "GND", "7": "GND", "11": "GND", "15": "GND", "21": "GND"}, B)
add("R18", "470R", two("WL_DO_R", "WL_D"), B)
add("R20", "10k", two("WL_IRQ_R", "WL_D"), B)
add("R19", "1k", two("WL_GPIO0", "WL_LED_A"), B)
add("D4", "led_ind", {"1": "GND", "2": "WL_LED_A"}, B)
add("C31", "10u", two("+3V3", "GND"), B)
add("C32", "100n", two("+3V3", "GND"), B)
add("C33", "100n", two("+3V3", "GND"), B)

# --- Temperature -----------------------------------------------------------------------
B = "power"
add("R22", "10k", two("+3V3", "NTC_SENSE"), B)
add("TH1", ("Thermistor_NTC", "10k B3380", "0603R", "Murata", "NCP18XH103F03RB"),
    two("NTC_SENSE", "GND"), B)
add("C34", "100n", two("NTC_SENSE", "GND"), B)

# --- LED channels ----------------------------------------------------------------------
for ch, (c, mfr, mpn) in enumerate(CHANNELS):
    B = f"ch_{c}"
    n = ch + 1
    add(f"U{4 + n}", ("AL8860", "AL8860MP-13", "MSOP8EP", "Diodes Inc", "AL8860MP-13"),
        {"8": "VIN", "1": f"LEDA_{c}", "4": f"CTRL_{c}", "5": f"SW_{c}", "6": f"SW_{c}",
         "7": None, "2": "GND", "3": "GND", "9": "GND"}, B)
    add(f"R{30 + n}", "rsense", two("VIN", f"LEDA_{c}"), B)
    add(f"C{40 + n}", "4u7_50", two("VIN", "GND"), B)
    add(f"L{2 + n}", ("L", "47uH 0.92A", "SRN6028", "Bourns", "SRN6028-470M"), two(f"LEDK_{c}", f"SW_{c}"), B)
    add(f"D{5 + n}", ("D_Schottky", "B240A", "SMA", "Diodes Inc", "B240A-13-F"), {"1": "VIN", "2": f"SW_{c}"}, B)
    add(f"R{40 + n}", "1k", two(f"PWM_{c}", f"CTRL_{c}"), B)
    add(f"R{50 + n}", "10k", two(f"CTRL_{c}", "GND"), B)
    add(f"C{50 + n}", "1u", two(f"CTRL_{c}", "GND"), B, dnp=True)
    nodes = [f"LEDA_{c}", f"{c}1", f"{c}2", f"LEDK_{c}"]
    for k in range(3):
        add(f"LED{ch * 3 + k + 1}", ("LED_XPE2", f"XP-E2 {c}", "XPE2", mfr, mpn),
            {"2": nodes[k], "1": nodes[k + 1], "3": "GND"}, B, side="B")


def nets():
    out = {}
    for p in PARTS:
        for num, net in p["pins"].items():
            if net:
                out.setdefault(net, []).append((p["ref"], num))
    return out


if __name__ == "__main__":
    ns = nets()
    for n in sorted(ns):
        print(f"{n:14s} {len(ns[n]):3d}  {' '.join(r + '.' + p for r, p in ns[n][:8])}")
    singles = [n for n, v in ns.items() if len(v) < 2]
    print("parts", len(PARTS), "nets", len(ns), "single-pin nets:", singles)
