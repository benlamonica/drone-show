# RTK GPS board, rev A

NEO-F10N-00B-20 (L1/L5 raw carrier phase) feeds an RP2350A running RTKLIB, which outputs the fix to the flight controller. An MMC5983MA compass sits on the FC's I2C bus.

- 2-layer board, 45 x 45 x 1.6 mm, corner radius 1.5 mm. 4x M2 holes (2.2 mm) on a 39 mm square, 3 mm in from each edge. All parts are on the top side and the smallest passive is 0603.
- PCBWay rules are in `rtk_gps.kicad_dru` and the project file: 0.15 mm (6 mil) track and space, 0.3/0.65 mm vias, 0.3 mm copper to edge.
- DRC: 0 errors and 0 unconnected; schematic parity is clean. One connection_width warning (a 0.13 mm neck at U1 pin 44, +3V3) and silk-clipped-by-mask warnings remain.
- ERC: clean.

## Connectors
| Ref | Part | Pins |
|---|---|---|
| J1 | JST GH-6, Pixhawk GPS port | 1 +5V, 2 FC_TX (into RP2350 GP5), 3 FC_RX (from GP4), 4 SCL, 5 SDA, 6 GND |
| J2 | JST GH-4, AUX UART (RTCM in) | 1 +5V, 2 AUX_TX (GP8), 3 AUX_RX (GP9), 4 GND |
| J3 | JST SH-3, SWD | Raspberry Pi Debug Probe pinout |
| J4 | U.FL | active antenna, 3.3 V bias through 10R + 27nH |
| J5 | USB-C | flashing/debug; diode-ORed with J1 5V |

## RP2350 pins
GP0 TX to GNSS RXD, GP1 RX from GNSS TXD, GP2 PPS, GP3 GNSS reset, GP4/GP5 UART1 to FC, GP6 EXTINT, GP8/GP9 AUX UART (PIO), GP25 status LED. SW1 is BOOTSEL and SW2 is RESET.

## Notes
- Compass: ArduPilot detects the MMC5983MA (I2C 0x30) as an external compass on the GPS I2C bus. The footprint is based on KiCad's MMC5883MA LGA-16 3x3 footprint, which has the same pin map. Check it against the MMC5983MA datasheet land pattern before ordering.
- Antenna: Taoglas AGVLB.25B.07.0060A (L1/L5 helical, U.FL, Mouser 960-AGVLB.25B070060A) is the suggested replacement for an L1-only helical.
- The firmware needs a GPS+Galileo+BeiDou, L1/L5 build (NFREQ=2); the NEO-F10N has no GLONASS.
- The J2 connector (SM04B-GHS-TB) is on backorder at Mouser until about Feb 2027. Any JST GH 4-pin horizontal SMT part fits, or leave J2 off if corrections come through the FC on J1.

## Regenerating
The scripts in `gen/` build everything from `gen/design.py`. They run in the kicad/kicad:9.0 docker image with Freerouting 1.9: `build.sh` does a full re-route, `post.sh` reuses the existing routes, and `gen/export.sh` writes `fab/`.
