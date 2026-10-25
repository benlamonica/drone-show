"""Single source of truth for the RTK GPS board: parts, values, sourcing and nets.

Each part: ref, lib_id, value, footprint, (x, y) schematic position in mm,
pins {pin_number: net}, and BOM fields. Pins not listed get a no-connect flag.
"""

FP_R = "Resistor_SMD:R_0603_1608Metric"
FP_C = "Capacitor_SMD:C_0603_1608Metric"
FP_C8 = "Capacitor_SMD:C_0805_2012Metric"

PARTS = []


def part(ref, lib, value, fp, at, pins, mpn="", mfr="", mouser="", dnp=False, desc=""):
    PARTS.append(dict(ref=ref, lib=lib, value=value, fp=fp, at=at, pins=pins,
                      mpn=mpn, mfr=mfr, mouser=mouser, dnp=dnp, desc=desc))


def R(ref, val, a, b, at, mpn="", mouser=""):
    part(ref, "Device:R", val, FP_R, at, {"1": a, "2": b}, mpn=mpn, mfr="Yageo" if mpn else "",
         mouser=mouser)


def C(ref, val, a, b, at, fp=FP_C, mpn="", mouser="", mfr="Yageo"):
    part(ref, "Device:C", val, fp, at, {"1": a, "2": b}, mpn=mpn, mfr=mfr if mpn else "",
         mouser=mouser)


# ---------------------------------------------------------------- power
part("J1", "Connector_Generic:Conn_01x06", "FC GPS port (Pixhawk GH-6)",
     "Connector_JST:JST_GH_SM06B-GHS-TB_1x06-1MP_P1.25mm_Horizontal", (30, 40),
     {"1": "+5V", "2": "FC_TX", "3": "FC_RX", "4": "I2C_SCL", "5": "I2C_SDA", "6": "GND"},
     mpn="SM06B-GHS-TB(LF)(SN)", mfr="JST", mouser="306-SM06BGHSTBLFSN")
part("J2", "Connector_Generic:Conn_01x04", "AUX UART (corrections in)",
     "Connector_JST:JST_GH_SM04B-GHS-TB_1x04-1MP_P1.25mm_Horizontal", (30, 70),
     {"1": "+5V", "2": "AUX_TX", "3": "AUX_RX", "4": "GND"},
     mpn="SM04B-GHS-TB(LF)(SN)", mfr="JST", mouser="306-SM04BGHSTBLFSN")
part("J5", "Connector:USB_C_Receptacle_USB2.0_16P", "USB-C (flash + debug)",
     "Connector_USB:USB_C_Receptacle_GCT_USB4105-xx-A_16P_TopMnt_Horizontal", (30, 110),
     {"A4": "VBUS", "A9": "VBUS", "B4": "VBUS", "B9": "VBUS",
      "A1": "GND", "A12": "GND", "B1": "GND", "B12": "GND", "S1": "GND",
      "A5": "CC1", "B5": "CC2", "A6": "USB_CONN_DP", "B6": "USB_CONN_DP",
      "A7": "USB_CONN_DM", "B7": "USB_CONN_DM"},
     mpn="USB4105-GF-A", mfr="GCT", mouser="640-USB4105-GF-A")
R("R7", "5.1k", "CC1", "GND", (60, 100), mpn="RC0603FR-075K1L", mouser="603-RC0603FR-075K1L")
R("R8", "5.1k", "CC2", "GND", (70, 100), mpn="RC0603FR-075K1L", mouser="603-RC0603FR-075K1L")

part("D1", "Device:D_Schottky", "PMEG4010CEJ", "Diode_SMD:D_SOD-323F", (70, 40),
     {"1": "VIN", "2": "+5V"}, mpn="PMEG4010CEJ,115", mfr="Nexperia", mouser="771-PMEG4010CEJ-T/R")
part("D2", "Device:D_Schottky", "PMEG4010CEJ", "Diode_SMD:D_SOD-323F", (70, 55),
     {"1": "VIN", "2": "VBUS"}, mpn="PMEG4010CEJ,115", mfr="Nexperia", mouser="771-PMEG4010CEJ-T/R")
part("U5", "Regulator_Linear:AP2112K-3.3", "AP2112K-3.3", "Package_TO_SOT_SMD:SOT-23-5", (100, 45),
     {"1": "VIN", "2": "GND", "3": "VIN", "5": "+3V3"},
     mpn="AP2112K-3.3TRG1", mfr="Diodes Inc", mouser="621-AP2112K-3.3TRG1")
C("C20", "10uF", "VIN", "GND", (85, 60), fp=FP_C8, mpn="CC0805KKX5R7BB106", mouser="603-CC805KKX5R7BB106")
C("C21", "10uF", "+3V3", "GND", (115, 60), fp=FP_C8, mpn="CC0805KKX5R7BB106", mouser="603-CC805KKX5R7BB106")
R("R12", "1k", "+3V3", "LED_PWR", (130, 40), mpn="RC0603FR-071KL", mouser="603-RC0603FR-071KL")
part("D3", "Device:LED", "green", "LED_SMD:LED_0603_1608Metric", (130, 55),
     {"1": "GND", "2": "LED_PWR"}, mpn="LTST-C191KGKT", mfr="Lite-On", mouser="859-LTST-C191KGKT")

# ---------------------------------------------------------------- RP2350
RP = {
    "1": "+3V3", "11": "+3V3", "20": "+3V3", "30": "+3V3", "38": "+3V3", "45": "+3V3",
    "6": "+1V1", "23": "+1V1", "39": "+1V1",
    "2": "GNSS_RXD", "3": "GNSS_TXD", "4": "GNSS_PPS", "5": "GNSS_RESET_N",
    "7": "FC_RX", "8": "FC_TX", "9": "GNSS_EXTINT",
    "12": "AUX_TX", "13": "AUX_RX",
    "37": "LED_STAT_R",
    "21": "XIN", "22": "XOUT", "24": "SWCLK", "25": "SWDIO", "26": "RUN",
    "44": "+3V3", "46": "VREG_AVDD", "47": "GND", "48": "VREG_LX", "49": "+3V3", "50": "+1V1",
    "51": "USB_DM", "52": "USB_DP", "53": "+3V3", "54": "+3V3",
    "55": "QSPI_SD3", "56": "QSPI_SCLK", "57": "QSPI_SD0", "58": "QSPI_SD2", "59": "QSPI_SD1",
    "60": "QSPI_SS", "61": "GND",
}
part("U1", "MCU_RaspberryPi:RP2350A", "RP2350A", "Package_DFN_QFN:QFN-60-1EP_7x7mm_P0.4mm_EP3.4x3.4mm",
     (200, 110), RP, mpn="SC1509-A4 (RP2350A)", mfr="Raspberry Pi", mouser="358-SC1509",
     desc="RTKLIB engine (RP2354A also fits; then leave U2 unpopulated)")
part("U2", "Memory_Flash:W25Q32JVSS", "W25Q32JVSS", "Package_SO:SOIC-8_5.3x5.3mm_P1.27mm", (270, 60),
     {"1": "QSPI_SS", "2": "QSPI_SD1", "3": "QSPI_SD2", "4": "GND", "5": "QSPI_SD0",
      "6": "QSPI_SCLK", "7": "QSPI_SD3", "8": "+3V3"},
     mpn="W25Q32JVSSIQ", mfr="Winbond", mouser="454-W25Q32JVSSIQ")
C("C18", "100nF", "+3V3", "GND", (290, 45))
R("R5", "1k", "QSPI_SS", "BOOTSEL", (250, 40))
part("SW1", "Switch:SW_Push", "BOOTSEL", "Button_Switch_SMD:SW_SPST_B3U-1000P", (250, 25),
     {"1": "BOOTSEL", "2": "GND"}, mpn="B3U-1000P", mfr="Omron", mouser="653-B3U-1000P")
R("R6", "10k", "+3V3", "RUN", (150, 160), mpn="RC0603FR-0710KL", mouser="603-RC0603FR-0710KL")
part("SW2", "Switch:SW_Push", "RESET", "Button_Switch_SMD:SW_SPST_B3U-1000P", (150, 175),
     {"1": "RUN", "2": "GND"}, mpn="B3U-1000P", mfr="Omron", mouser="653-B3U-1000P")
part("J3", "Connector_Generic:Conn_01x03", "SWD (Pi debug probe)",
     "Connector_JST:JST_SH_SM03B-SRSS-TB_1x03-1MP_P1.00mm_Horizontal", (150, 195),
     {"1": "SWCLK", "2": "GND", "3": "SWDIO"},
     mpn="SM03B-SRSS-TB(LF)(SN)", mfr="JST", mouser="306-SM03BSRSSTBLFSNP")

# crystal
part("Y1", "Device:Crystal_GND24", "12MHz", "Crystal:Crystal_SMD_3225-4Pin_3.2x2.5mm", (150, 120),
     {"1": "XIN", "2": "GND", "3": "XOUT_X", "4": "GND"},
     mpn="ABM8-272-T3", mfr="Abracon", mouser="815-ABM8-272-T3")
R("R4", "1k", "XOUT", "XOUT_X", (150, 135))
C("C16", "15pF", "XIN", "GND", (135, 110))
C("C17", "15pF", "XOUT_X", "GND", (165, 135))

# core regulator
part("L1", "Device:L", "3.3uH", "Inductor_SMD:L_Murata_DFE201610P", (150, 75),
     {"1": "VREG_LX", "2": "+1V1"}, mpn="AOTA-B201610S3R3-101-T", mfr="Abracon",
     mouser="815-AB201610S3R3101T")
R("R1", "33R", "+3V3", "VREG_AVDD", (135, 90))
C("C1", "4.7uF", "+3V3", "GND", (165, 60))   # VREG_VIN
C("C2", "4.7uF", "VREG_AVDD", "GND", (120, 90))
C("C3", "4.7uF", "+1V1", "GND", (165, 75))
for i, x in enumerate((175, 185, 195)):
    C(f"C{4+i}", "100nF", "+1V1", "GND", (x, 60))
for i, x in enumerate((205, 215, 225, 235, 245, 255)):
    C(f"C{7+i}", "100nF", "+3V3", "GND", (x, 160))
C("C13", "100nF", "+3V3", "GND", (265, 160))  # USB_OTP_VDD
C("C14", "100nF", "+3V3", "GND", (275, 160))  # QSPI_IOVDD
C("C15", "100nF", "+3V3", "GND", (285, 160))  # ADC_AVDD

# USB
R("R2", "27R", "USB_CONN_DP", "USB_DP", (70, 120))
R("R3", "27R", "USB_CONN_DM", "USB_DM", (70, 135))

# status LED
R("R13", "1k", "LED_STAT_R", "LED_STAT", (260, 120), mpn="RC0603FR-071KL", mouser="603-RC0603FR-071KL")
part("D4", "Device:LED", "blue", "LED_SMD:LED_0603_1608Metric", (275, 120),
     {"1": "GND", "2": "LED_STAT"}, mpn="LTST-C191TBKT", mfr="Lite-On", mouser="859-LTST-C191TBKT")

# ---------------------------------------------------------------- GNSS
part("U3", "rtk_gps:NEO-F10N", "NEO-F10N-00B-20", "RF_GPS:ublox_NEO", (330, 110),
     {"1": "GNSS_SAFEBOOT_N", "3": "GNSS_PPS", "4": "GNSS_EXTINT", "8": "GNSS_RESET_N",
      "9": "VCC_RF", "10": "GND", "11": "RF_IN", "12": "GND", "13": "GND",
      "20": "GNSS_TXD", "21": "GNSS_RXD", "22": "+3V3", "23": "+3V3", "24": "GND"},
     mpn="NEO-F10N-00B-20", mfr="u-blox", mouser="377-NEO-F10N-00B-20",
     desc="L1/L5, carrier-phase RAWX (-20 variant only)")
C("C22", "100nF", "+3V3", "GND", (360, 60))
C("C23", "10uF", "+3V3", "GND", (370, 60), mpn="LMK107BBJ106MALT", mouser="963-LMK107BBJ106MALT", mfr="Taiyo Yuden")
C("C27", "100nF", "+3V3", "GND", (380, 60))   # V_BCKP
part("TP1", "Connector:TestPoint", "SAFEBOOT", "TestPoint:TestPoint_Pad_D1.0mm", (300, 80),
     {"1": "GNSS_SAFEBOOT_N"})
# active antenna feed: VCC_RF -> 10R -> 27nH -> RF_IN (RF_IN is DC-blocked inside the module)
R("R11", "10R", "VCC_RF", "ANT_BIAS", (370, 150))
C("C24", "10nF", "VCC_RF", "GND", (360, 165))
part("L2", "Device:L", "27nH", "Inductor_SMD:L_0603_1608Metric", (385, 150),
     {"1": "ANT_BIAS", "2": "RF_IN"}, mpn="LQG18HN27NJ00D", mfr="Murata", mouser="81-LQG18HN27NJ00D")
C("C28", "100nF", "ANT_BIAS", "GND", (380, 165))
part("J4", "Connector:Conn_Coaxial", "U.FL antenna", "Connector_Coaxial:U.FL_Hirose_U.FL-R-SMT-1_Vertical",
     (400, 110), {"1": "RF_IN", "2": "GND"}, mpn="U.FL-R-SMT-1(10)", mfr="Hirose",
     mouser="798-U.FL-R-SMT-1-10")

# ---------------------------------------------------------------- compass (FC I2C bus)
part("U4", "rtk_gps:MMC5983MA", "MMC5983MA", "Package_LGA:LGA-16_3x3mm_P0.5mm", (330, 200),
     {"1": "I2C_SCL", "2": "+3V3", "4": "+3V3", "9": "GND", "10": "+3V3", "11": "GND",
      "13": "+3V3", "16": "I2C_SDA"},
     mpn="MMC5983MA", mfr="MEMSIC", mouser="438-MMC5983MA", desc="I2C 0x30, ArduPilot MMC5XX3 driver")
C("C25", "1uF", "+3V3", "GND", (360, 200))
C("C26", "10uF", "+3V3", "GND", (370, 200), mpn="LMK107BBJ106MALT", mouser="963-LMK107BBJ106MALT", mfr="Taiyo Yuden")
R("R9", "2.2k", "+3V3", "I2C_SCL", (300, 190), mpn="RC0603FR-072K2L", mouser="603-RC0603FR-072K2L")
R("R10", "2.2k", "+3V3", "I2C_SDA", (300, 210), mpn="RC0603FR-072K2L", mouser="603-RC0603FR-072K2L")

# power flags so ERC knows where power comes from
FLAGS = ["+5V", "VBUS", "+1V1", "GND", "VIN", "VREG_AVDD"]

# generic sourcing for anonymous passives
GENERIC = {
    ("Device:C", "100nF"): ("CC0603KRX7R7BB104", "Yageo", "603-CC603KRX7R7BB104"),
    ("Device:C", "4.7uF"): ("CL10A475MO8NNNC", "Samsung", "187-CL10A475MO8NNNC"),
    ("Device:C", "1uF"): ("CC0603KRX7R7BB105", "Yageo", "603-CC603KRX7R7BB105"),
    ("Device:C", "10nF"): ("CC0603KRX7R9BB103", "Yageo", "603-CC603KRX7R9BB103"),
    ("Device:C", "15pF"): ("CC0603JRNPO9BN150", "Yageo", "603-CC603JRNPO9BN150"),
    ("Device:R", "1k"): ("RC0603FR-071KL", "Yageo", "603-RC0603FR-071KL"),
    ("Device:R", "33R"): ("RC0603FR-0733RL", "Yageo", "603-RC0603FR-0733RL"),
    ("Device:R", "27R"): ("RC0603FR-0727RL", "Yageo", "603-RC0603FR-0727RL"),
    ("Device:R", "10R"): ("RC0603FR-0710RL", "Yageo", "603-RC0603FR-0710RL"),
}
for p in PARTS:
    if not p["mpn"] and (p["lib"], p["value"]) in GENERIC:
        p["mpn"], p["mfr"], p["mouser"] = GENERIC[(p["lib"], p["value"])]
