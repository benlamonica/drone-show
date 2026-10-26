# drone-esc: 4-layer 4-in-1 ESC for the show drones

A 4-layer re-layout of [OpenESC-30x30](https://github.com/benlamonica/OpenESC-30x30)
for PCBWay's standard process. The circuit is the prototype's (four AT32F421
MCUs running AM32, four NSG2065Q gate drivers, 24 SP40N01GHNK FETs, INA186
current sense, LMR54406 10 V buck, TLV767 3.3 V LDO). What changed is the
board: 4 layers instead of 6, 0402 and bigger passives, and an 80 mm octagon
instead of a 30 x 30 mm square.

## Status

The board is placed and routed. KiCad DRC shows 0 errors and 0 unconnected
items. The remaining warnings are expected: schematic parity for U3 pins 3 to
11 (the FC wire pads were taken off the board, and J1 carries those signals),
the schematic's empty LCSC fields, and a few dangling stubs and thin pour
slivers. Look over the gate drive and current-sense routing before ordering a
big batch. Nobody has built one yet.

## Ratings

| | |
|---|---|
| Battery | 4S to 6S (25.2 V max). The FETs are 40 V parts, so leave the prototype's 6S ceiling alone. |
| Current | about 35 A continuous per motor with airflow, limited by 2 oz copper and the FET package. Show flying (hover plus gentle moves) needs a fraction of that. |
| Firmware | AM32 for AT32F421, same pinout as OpenESC-30x30, flashed over the motor signal lines through J1 (AM32 bootloader), or over SWD test points. |
| FC link | JST-SH 8-pin side entry, SM08B-SRSS-TB (Betaflight 4-in-1 order: VBAT, GND, CURR, TLM, M1 to M4; TLM is not connected). |
| Mounting | 30.5 mm M3 pattern, 3.3 mm unplated holes with a 6.9 mm copper-free ring, so it stacks with the FC on rubber standoffs. |

## Board

* 80 mm across the flats, octagonal, 1.6 mm FR4, 4 layers: 2 oz outer,
  1 oz inner. Layer 2 is a solid GND plane; layer 3 is a +BATT pour that a
  few signal tracks also cross.
* Two layers were tried first and did not work: at 66 mm the gate-drive lines
  had no room, and at 74 mm the routing cut the GND pour into islands, which
  is no good on a board switching 100+ A. The inner planes fix both and give a
  much shorter commutation loop.
* Each motor owns one edge. The high-side FET of each half-bridge is on top,
  its low-side FET directly underneath on the bottom, and the phase copper runs
  out to plated motor pads on the edge.
* The outer layers are +BATT pour on top and GND pour on the bottom, stitched
  to the inner planes.
* The motor channels own the four axis edges; the corner flats carry the
  connections:
  * **Battery** on the rear-right corner flat (marked BAT+ / BAT- on both
    sides). Each pad is a 7 x 6 mm plated pad with bare copper on both sides,
    a 2.2 mm hole (12 or 14 AWG goes straight through) and a ring of eight
    small plated holes so the current reaches both sides. Solder the battery
    lead on one side and the low-ESR electrolytic (35 V, 470 uF or more) on
    the other, legs on the matching pads, so the cap sits right at the power
    entry. The two current-sense shunts sit right beside BAT+, with the INA186
    and its input filter underneath on the bottom. Its output filter (R73,
    C94) is at the FC end, under J1.
  * **FC connector** J1 on the front-left corner flat: a side-entry JST SH
    with its opening facing out of the board edge, so the lead leaves sideways
    instead of standing up into the stack.
* Each channel's logic sits on the bottom, on the GND pour, in the wedge
  between the board centre and its FETs. The gate driver is just inboard of
  the FETs, with the gate resistors and bootstrap caps. The MCU is on the
  centre side of the driver, 6 mm from the middle of the board, so the six
  PWM lines are a few millimetres long and stay on one layer. The four MCUs
  form a ring round the centre, with their DShot resistors in the middle.
  The MCU's NRST, BOOT0 and VDDA parts, its 3V3 decoupling cap and the SWD
  test points are on top, right over it. The corners stay free for the
  battery, the FC connector and the 10 V / 3.3 V supplies, which are on the
  bottom of the rear-left corner.
* The MCU and gate-driver pins are 0.4 and 0.5 mm apart and boxed in on
  every side, which neither router gets out of reliably. So the MCU, the
  driver and the caps round them are placed exactly, and the escapes are
  drawn by `tools/route_prep.py`, the same in every channel: the PWM lines,
  the 3V3, NRST, VDDA and back-EMF escapes, and the driver's high-side row.
  The high-side gate lines drop to vias there and cross on top. 0.18 mm
  tracks keep every gap at 0.22 mm or more.
* 24 x 10 uF 1206 ceramics sit around the power path, next to each channel's
  high-side FETs.
* Parts: the MCU passives (decoupling, NRST, BOOT0, VDDA, back-EMF dividers,
  DShot resistor) are 0402, which gives the MCU room for a clear ring of
  escape tracks; everything else is 0603 or bigger. Nothing is smaller than
  0402. The ICs are the prototype's packages; the MCU QFN at 0.4 mm pitch is
  the finest pitch on the board.

### Changes from the prototype BOM

* The prototype's 0201 and 0402 resistors and capacitors became 0402 round
  each MCU (YAGEO RC0402, Samsung CL05B104KB5NNNC) and 0603 everywhere else
  (YAGEO RC0603, Samsung CL10). Same values throughout.
* J1 is the side-entry SM08B-SRSS-TB instead of a top-entry connector.
* The bulk ceramics are now real schematic symbols (C100 to C123) so they are
  in the BOM and the pick-and-place file.
* QFN-28 footprint: the prototype's footprint was missing pads 8, 23 and 24.
  They are back, so every MCU pin is soldered.

## Making it at PCBWay

Upload `fab/drone-esc-gerbers.zip` and choose:

| Option | Value |
|---|---|
| Layers | 4 |
| Material / thickness | FR-4 TG150, 1.6 mm |
| Copper | 2 oz outer, 1 oz inner |
| Min track / space | 7 / 8 mil (0.178 / 0.2 mm) |
| Min hole / via | 0.3 mm drill, 0.66 mm via pad (7 mil ring) |
| Surface finish | ENIG (flat pads for the QFNs) or lead-free HASL |
| Solder mask / silk | any colour |

For assembly upload `fab/drone-esc-bom.csv` and `fab/drone-esc-cpl.csv`.
Parts are on both sides. The design rules PCBWay checks against are in
`drone-esc.kicad_pro` and `drone-esc.kicad_dru`; open the board in KiCad 10 and
run DRC to see them.

## Frame changes this board needs

The board is bigger than the 30x30 prototype, so `frame/frame.py` needs:

* `BODY` 90 to about 110 mm, so the arm sockets clear the board's corner
  flats (37.8 mm from the centre).
* `GPS_MAST_R` 38 to about 53 mm, so the GPS mast clears the board edge
  (40 mm from the centre).
* Check both against the 3D model; they are worked out from the board outline,
  not test-fitted.
* Bottom-side parts stand about 1.2 mm proud of the board. The low ring and
  spokes at boss height in the hub touch them, so either raise the ESC on
  3 mm spacers/grommets or drop the ring by 3 mm.
* The battery leads leave from the rear-right corner flat, where motor 1's arm
  socket is. Leave a gap or notch in the hub wall there for the leads and the
  electrolytic (about 12 mm wide for two 12 AWG leads, more if the cap lies
  beside them), and route the leads back past the arm. The hub floor no longer
  needs a window under the board.
* J1's plug comes out of the front-left corner flat, sideways: leave about
  8 mm clear outside that flat (motor 4's arm socket) for the plug and its
  lead, which then runs up to the FC.

## Notes and limits

* Apart from the drawn escapes, the signal tracks are routed in three steps.
  First `tools/rescue_route.py`, a small grid maze router, routes the MCU's
  short local nets, the bootstrap nets and 3V3 on the near-empty board. Then
  Freerouting routes everything else round them. Finally the maze router
  finishes what Freerouting left open.
* RS2205-class motors are usually wound for 4S. On 6S pick a lower KV
  (about 1700 to 1900 KV) or limit throttle in the FC.
* The FETs and gate drivers sit on both sides, so the board can't be all
  parts-on-top like the FC. Bottom parts stand about 1.2 mm proud; the rubber
  standoffs between stacked boards need to be taller than that. Nothing on the
  board stands up into the stack any more: J1 is side entry and the
  electrolytic hangs off the battery pads.
* Four layers is a bit more per board than two at PCBWay, but still far below
  the prototype's six.

## Rebuilding

Everything is generated by scripts in `tools/` with KiCad 10:

```sh
FREEROUTING=/path/to/freerouting-1.9.0.jar PYTHON=/path/to/venv/bin/python sh tools/build.sh
sh tools/export.sh                                             # board -> fab/
```

A full build takes about 25 minutes on a laptop. `PYTHON` is a plain Python 3 with sexpdata, numpy, scipy and Pillow (the
netlist converter and the rescue router run there); the board scripts run in
KiCad's own Python (`KIPY`, found by itself on macOS). `sh tools/build.sh
place` stops after placement, which takes seconds, for trying placement
changes before a full routing run.

`tools/layout.py` holds the board geometry (size, FET positions, pads) that
the footprint and placement scripts share.
