# Light controller

A 73 x 35 mm, 2-layer board that slides into the frame's light rail
(`frame/out/light_pcb_outline.dxf`). It hosts the Wi-Fi link and drives twelve 1 W-class
Cree XP-E2 LEDs (3 red, 3 green, 3 blue, 3 white) straight from a 4S-6S pack.

| | |
|---|---|
| MCU | RP2350A (QFN-60) + 2 MB W25Q16 flash, 12 MHz crystal |
| Wi-Fi | Raspberry Pi RM2 module (CYW43439), wired exactly like the Pico 2 W |
| LED drive | 4 x AL8860 hysteretic constant-current bucks, one per colour, 0.67 A each |
| LEDs | 12 x XP-E2 on the bottom side, facing the diffuser |
| Input | 4S-6S (12-26 V), reverse-polarity P-FET, SMBJ26A TVS |
| Logic supply | LMR51420 buck to 3.3 V |
| Interfaces | JST-SH UART to the flight controller, JST-SH SWD, 4 test pads |
| Board | 2 layers, 1.6 mm, 1 oz, smallest passive 0603, PCBWay standard rules |

## Files

| Path | What |
|---|---|
| `light-controller.kicad_pro/.kicad_sch/.kicad_pcb` | KiCad project (opens in KiCad 7 and later) |
| `light-controller.kicad_dru` | PCBWay design rules (loaded by DRC automatically) |
| `lightctl.kicad_sym`, `lightctl.pretty/` | Project-local symbols and footprints, nothing from global libraries |
| `fab/light-controller-gerbers.zip` | Upload this to PCBWay |
| `fab/light-controller-bom.csv`, `fab/light-controller-cpl.csv` | PCBWay assembly BOM and placement (both sides) |
| `images/` | Renders of both sides |
| `tools/` | The generators. `design.py` is the single source of the netlist and BOM |

Regenerate everything: `python3 tools/build_lib.py && python3 tools/gen_sch.py &&
python3 tools/gen_pcb.py`, then route (`tools/route.py`, `tools/finish_route.py`,
`tools/stitch.py`), check (`tools/drc.py`) and export (`tools/gen_fab.py`,
`tools/render.py`). Re-running `gen_pcb.py` throws the routing away, so after the
first build edit the board in KiCad instead.

## Brightness and power

Each colour is a string of three XP-E2 in series on its own AL8860, with the sense
resistor setting I = 0.1 V / 0.15 ohm = 0.67 A. At that current, roughly:

| Colour | Per LED | 3 LEDs |
|---|---|---|
| White (cool) | ~230 lm | ~700 lm |
| Green | ~170 lm | ~500 lm |
| Red | ~100 lm | ~300 lm |
| Blue | ~450 mW radiant | ~1.3 W radiant |

All four channels at full is about 1500 lm and 25 W from the pack, which is far more
heat than a 73 x 35 mm board can shed in still air. It is fine in flight with prop
wash, and a show rarely holds full white. The firmware should still:

- cap the sum of the four duty cycles (for example 200 % total) during a show,
- derate on the NTC (`TH1`, on GPIO26) above about 80 C board temperature,
- never run full white on the bench without airflow.

To trade brightness for heat, change the four 0.15 ohm sense resistors
(`R31`-`R34`): 0.2 ohm gives 0.5 A. Don't go below 0.1 ohm (1 A, the XP-E2 maximum).

Dimming is PWM on each AL8860 CTRL pin through a 1k/10k divider. Keep the PWM under
500 Hz for linear dimming (1 kHz still works but the bottom of the range gets coarse).
CTRL below 0.2 V switches the channel fully off, so the LEDs are dark while the RP2350
is in reset. `C51`-`C54` are not fitted; fit them only to switch to analog dimming.

## Pin map

| GPIO | Function |
|---|---|
| 0 / 1 | UART TX / RX to the flight controller (MAVLink) |
| 2 | PWM green (PWM1 A) |
| 3 | PWM white (PWM1 B) |
| 8 | PWM red (PWM4 A) |
| 9 | PWM blue (PWM4 B) |
| 10, 11, 16 | Spare, on test pads TP2-TP4 |
| 23, 24, 25, 29 | RM2: WL_ON, data, CS, clock (same as Pico 2 W) |
| 26 (ADC0) | NTC board temperature |
| 27 (ADC1) | Battery voltage, 100k/10k divider (x11) |

Because the radio uses the Pico 2 W pins, the stock `pico2_w` board definition and
`pico_cyw43_arch` work unchanged. There is no separate status LED: use the CYW43
`WL_GPIO0` LED (`cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, ...)`) as on a Pico 2 W, or
the main LEDs themselves. GPIO 4-7, 12-15, 17-22 and 28 are unconnected.

## Programming: no USB

USB-C and the BOOTSEL button were left off to make room for a 2-layer layout. Program
the first time over SWD (JST-SH `J3`, the Raspberry Pi Debug Probe pinout and cable fit
directly: `openocd -f interface/cmsis-dap.cfg -f target/rp2350.cfg`). After that the
firmware can update itself over Wi-Fi. `RUN` is on TP1 if you need a hard reset.

## Firmware port from the existing `pico/` code

The current firmware drives a WS2812B/SK6812 strip through `port/led_strip.c`. On this
board the same `led_strip_fill(r, g, b, w)` call should instead set four PWM levels on
GPIO 8, 2, 9, 3 (red, green, blue, white). The `show_led` MAVLink parser
(`DEBUG_VECT` "rgb") stays as it is. Use about 400 Hz PWM with a gamma table, and add
the power cap and NTC derating described above.

## Things to check before ordering

1. **XP-E2 polarity.** The footprint uses KiCad's `LED_Cree-XP` (pad 1 cathode, pad 2
   anode, centre pad 3 thermal and electrically neutral, tied to GND for heat). Compare
   with the current Cree XP-E2 datasheet drawing.
2. **RM2 footprint.** `RPi_RM2` was drawn from the RM2 datasheet (pads and antenna
   keep-out). Check it against the latest Raspberry Pi drawing; nothing may be placed
   under or in front of the antenna, and the board edge past it is clear of copper.
3. **ERC.** The schematic is generated, so open it in KiCad and run ERC once. The
   netlist was verified to match the board pin for pin.
4. **DRC.** Run DRC in KiCad with the project rules; see the routing notes below.
5. **LED bins.** Any in-stock flux bin works; the MPNs in the BOM are typical ones.

## Routing notes

The board was routed by script (Freerouting, then a grid router with rip-up in
`tools/finish_route.py`, then the `gnd_*` tools to tie every ground pad and pour island
together with stitching vias). Current DRC with `light-controller.kicad_dru`:

- 0 errors: no clearance, hole, width or unconnected items. The netlist on the board was
  checked pad by pad against `tools/design.py`.
- 76 silkscreen warnings (reference text overlapping other silk or clipped by the mask).
  They are cosmetic; tidy the reference text in KiCad if you care about the labels.

The tracks are machine-made and look it: there are short zig-zags and a few long
detours. They are electrically fine. Ground pads use solid (not thermal) pour
connections, so hand soldering ground pins needs a hot iron; reflow is unaffected.
The 0.4 mm-pitch RP2350 fan-out uses 0.15 mm tracks and 0.3/0.6 mm vias, inside
PCBWay's standard 6/6 mil, 0.3 mm process.

## PCBWay order settings

2 layers, FR-4, 1.6 mm, 1 oz outer copper, HASL lead-free or ENIG, min track/space
6/6 mil, min hole 0.3 mm. Assembly on both sides (LEDs, flash, buck, SWD and a few
passives are on the bottom). Battery leads are soldered by hand to `J1`.

## Main parts

| Ref | Part | Mouser / maker number |
|---|---|---|
| U2 | RP2350A | Raspberry Pi RP2350A (or RP2354A with built-in flash, then omit U3) |
| M1 | RM2 radio module | Raspberry Pi SC1169 |
| U3 | 16 Mbit QSPI flash | Winbond W25Q16JVSNIQ |
| U1 | 3.3 V buck | TI LMR51420YDDCR |
| U5-U8 | LED driver | Diodes AL8860MP-13 |
| L3-L6 | 47 uH 0.92 A | Bourns SRN6028-470M |
| D6-D9 | Schottky | Diodes B240A-13-F |
| R31-R34 | 0.15 ohm sense | Vishay WSL1206R1500FEA |
| LED1-12 | XP-E2 R/G/B/W | Cree XPEBRD, XPEBGR, XPEBBL, XPEBWT |
| Q1 | Reverse-polarity P-FET | Diodes DMP6023LE-13 |
| D2 | TVS | Littelfuse SMBJ26A |

The full list is in `fab/light-controller-bom.csv`.
