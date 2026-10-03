# RTKLIB on the Raspberry Pi Pico 2 W (RP2350)

An embedded port of **RTKLIB 2.4.3 b34** that turns a **raw-capable but non-RTK**
u-blox receiver (NEO/LEA-M8T class) plus a Pico 2 W into an RTK GPS for
ArduPilot. The expensive part of RTK — carrier-phase ambiguity resolution —
runs on the RP2350 instead of inside a ZED-F9P.

```
   u-blox M8T ──UBX RXM-RAWX/SFRBX──▶ ┌──────────────┐
                                      │  Pico 2 W    │──NMEA──▶ ArduPilot
   base link ──RTCM3 or UBX─────────▶ │  RTKLIB RTK  │
                                      └──────────────┘
```

## What this needs that you may not have yet

RTK is **differential**. It needs a second receiver at a surveyed point and a
data link to the drone. One base serves an entire fleet, so N drones cost
N × (M8T + Pico) + 1 base, which is what makes this cheaper than N × F9P. There
is no configuration in which a single roving receiver produces an RTK fix.

## Status

| | |
|---|---|
| RTKLIB base | 2.4.3 b34 (`rtklib_2.4.3`) |
| Host test suite | 357 checks across GPS+Galileo and GPS+GLONASS builds, all passing |
| On-target suite | 108 checks passing on the RP2350 (GPS+GLONASS build) |
| RAM (`drone-show-controller`, incl. Wi-Fi + lwIP) | **263 KB of 520 KB** (51%) |
| Flash | **547 KB of 4 MB** (13%) |
| Accuracy (simulated, 1.2 km baseline) | 1.3 mm static / 8.0 mm kinematic, horizontal RMS over fixed epochs |
| Correction link | NTRIP over Wi-Fi, verified end to end on hardware |

## Layout

```
pico/
  config/rtkpico_config.h   compile-time trimming; the one place to change sizing
  port/                     platform shims, stubs, engine, NMEA output
  sim/                      GNSS simulator + UBX encoder + scenario driver
  test/                     host tests and the shared benchmark body
  app/                      firmware, on-target bench, on-target test runner
  openocd/, scripts/        debug configuration
```

## Building

```bash
export PICO_SDK_PATH=~/pico/pico-sdk
export PATH=/opt/gcc-arm-none-eabi-10-2020-q4-major/bin:$PATH

# firmware + on-target bench/tests
cmake -S pico -B build/pico -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release
cmake --build build/pico -j8

# host tests (no hardware needed): every test runs twice, GPS+Galileo and
# GPS+GLONASS (the M8P's pairing), as test_x and test_x_glo
cmake -S pico/test -B build/host && cmake --build build/host -j8
(cd build/host && ctest) && ./build/host/bench_host
```

Build options: `-DRTKPICO_ENABLE_GAL=ON|OFF`, `-DRTKPICO_ENABLE_GLO=ON|OFF`,
`-DRTKPICO_RTCM3=ON|OFF`, `-DRTKPICO_COPY_TO_RAM=ON|OFF`.

## Flashing and debugging

The Raspberry Pi OpenOCD fork is required — upstream 0.12.0 has no RP2350
target. Build it once:

```bash
git clone --depth 1 -b sdk-2.0.0 https://github.com/raspberrypi/openocd.git ~/pico/openocd
cd ~/pico/openocd && ./bootstrap
./configure --prefix=$HOME/pico/openocd-install --enable-cmsis-dap \
            --disable-werror --disable-capstone
make -j8 && make install
```

`--disable-capstone` avoids a header-path break against Homebrew's capstone 5;
nothing here needs disassembly.

```bash
./pico/scripts/flash.sh build/pico/drone-show-controller.elf     # program over SWD
./pico/scripts/debug.sh                            # GDB server on :3333
arm-none-eabi-gdb build/pico/drone-show-controller.elf -ex 'target extended-remote :3333'
```

## Wiring

| Pin | Direction | Peer |
|---|---|---|
| GP0 | UART0 TX | rover receiver (configuration at boot) |
| GP1 | UART0 RX | rover receiver — UBX RXM-RAWX + RXM-SFRBX |
| GP4 | UART1 TX | flight controller — NMEA |
| Wi-Fi | in | base corrections — RTCM3 over NTRIP |

Set your surveyed base position in `BASE_ECEF` in `app/main.c`, or leave it
zero and let the caster's RTCM 1005/1006 supply it.

## Base corrections over Wi-Fi (NTRIP)

Corrections come from `str2str` on a Raspberry Pi attached to the base
receiver. Configure the link at the top of `app/main.c`:

```c
#define WIFI_SSID      "drone-show"
#define WIFI_PASSWORD  "..."
#define NTRIP_HOST     "192.168.4.122"
#define NTRIP_PORT     2101
#define NTRIP_MOUNT    "RTCM3"     /* must match str2str's mountpoint */
```

The client is deliberately tolerant of what answers on the far end, so the
same firmware works against either of the usual `str2str` output modes:

```bash
# NTRIP caster (RTKLIB 2.4.3+) — mountpoint must match NTRIP_MOUNT
str2str -in serial://ttyACM0:115200 -out ntripc://:2101/RTCM3

# plain TCP server — mountpoint is ignored, RTCM starts immediately
str2str -in serial://ttyACM0:115200 -out tcpsvr://:2101
```

It handles `ICY 200 OK` (NTRIP v1), `HTTP/1.1 200 OK` (v2), and a bare RTCM3
stream. If the caster rejects the mountpoint it replies with a source table,
and the status line prints the mountpoint names the caster *does* offer —
that is nearly always the misconfiguration.

lwIP runs from an alarm IRQ (`pico_cyw43_arch_lwip_threadsafe_background`),
not polled. This matters: the main loop sits inside `rtkpos()` for ~100 ms per
epoch, and the polling variant would stall TCP for exactly that long, every
epoch. Corrections land in an 8 KB ring buffer from the lwIP callback.

Wi-Fi loss, caster restart and RTCM silence each trigger a reconnect with
exponential backoff (2 s to 30 s). Losing corrections is never fatal — the
engine keeps producing a single-point position, which is a better thing to
hand a flight controller than nothing.

The health line on USB reports the whole chain:

```
stat=1 ns=12 age=0.8 ratio=6.4 | sol=241 fix=238 flt=3 | rtkpos 104/119 us | ...
  ntrip: state=5 wifi=1 ip=192.168.4.168 rssi=-26 rtcm=18242 B rc=0 ... | caster accepted /RTCM3
```

### Testing without a base station

`scripts/test_caster.py` is a minimal NTRIP caster that serves valid RTCM3
1005 frames, so the network path can be exercised on the bench:

```bash
./pico/scripts/test_caster.py --port 2101 --mount RTCM3
```

It carries a station position but no observations, so the firmware will show
corrections arriving and a base position adopted while still reporting a
single-point solution.

### If corrections never arrive

The firmware prints its own DHCP address for exactly this reason. If it is on
the caster's subnet but nothing connects, suspect **client isolation** on the
access point (also called AP isolation or guest mode): clients can reach the
gateway but not each other, so ARP resolves while TCP never completes. The
drones and the Pi must be on a network that permits client-to-client traffic.

On the ArduPilot side: `SERIALn_PROTOCOL=5` (GPS), `SERIALn_BAUD=57`,
`GPS_TYPE=5` (NMEA). GGA carries the RTK status (4 = fixed, 5 = float) and GST
carries the per-axis standard deviations the EKF uses for weighting.

## How the port was made to fit

Stock RTKLIB needs ~550 KB of RAM for a rover+base session — more than the
RP2350 has. Almost all of it is features this port never uses:

| Removed from `nav_t` | Saved |
|---|---|
| SBAS ionospheric grid (`sbsion[11]`) | 71 KB |
| SSR / PPP-RTK corrections | 37 KB |
| Satellite antenna PCV tables | 29 KB |
| QZSS LEX ephemerides | 19 KB |
| SBAS satellite corrections | 12 KB |
| RTCM2 DGPS corrections | 4 KB |

`nav_t` collapses from roughly 200 KB to a few KB. Dropping the unused SBAS
satellite slots (an empty PRN range) also removes 39 entries from `MAXSAT`
under 2.4.3, which shrinks the EKF covariance substantially. Run
`build/host/memreport` and `memreport_stock` to see the current numbers both
ways; a stock rover+base session does not fit in 520 KB, and the trimmed one
uses about half.

2.4.3 makes the trimming matter *more*, not less: `MAXOBS` rose to 96, the
SBAS range to PRN 158, Galileo to PRN 36, and IRNSS was added — so untrimmed
`MAXSAT` is larger than in 2.4.2.

Every change to `src/` is behind `#ifdef RTKLIB_EMBEDDED` (or a
feature-specific `RTKLIB_NO_*`) and is inert in a normal build — all 33
upstream sources still compile unchanged with every constellation enabled.

### Note on XIP

RP2350 already executes from flash through the XIP cache, so the ~251 KB of
code was never the constraint; the 520 KB limit applies to *data*, which XIP
cannot relocate. Pico 2 W does not populate the external QSPI PSRAM that
RP2350 can otherwise address. `-DRTKPICO_COPY_TO_RAM=ON` does the opposite —
spends SRAM to remove XIP cache misses from the filter — and is available as a
performance option now that there is RAM to spare.

### NFREQ must be 2, not 1

2.4.2's `prange()` opened with `if (NFREQ<2 || ...) return 0.0`, so an
`NFREQ==1` build rejected every satellite and could never get a single-point
fix. 2.4.3 removed that guard, but `prange()` still reads `obs->P[1]` and
`obs->code[1]` unconditionally before the `ionoopt` test, which at `NFREQ=1`
indexes one past `double P[NFREQ+NEXOBS]`. So `NFREQ=1` remains undefined
behaviour, just a quieter kind.

The count that drives processing and the EKF state size is `prcopt_t.nf`,
which the engine pins to 1. `NFREQ=2` sizes the arrays; `nf=1` keeps the
filter single-frequency — the standard RTKLIB configuration for an L1-only
receiver.

## Testing

The simulator generates a Walker constellation, computes true observables for
a base and a rover, encodes them as **byte-exact UBX**, and feeds them to the
engine one byte at a time. Ephemerides go out as real GPS LNAV subframes
through `RXM-SFRBX`, so `save_subfrm()` → `decode_frame()` → `decode_ephem()`
are all exercised — nothing is injected behind the decoder's back.

GLONASS is simulated the same way, and it is the GPS+GLONASS build (`_glo`)
that matches the firmware. The 24 satellites fly the real 3-plane layout with
the real slot→channel table. Each observes on its own FDMA carrier
(1602 MHz + 0.5625 MHz × channel, ionosphere scaled to match) and reports its
channel in RAWX `freqId`. The ephemerides are PZ-90 state vectors, integrated
by RTKLIB's own `geph2pos()` and broadcast as real navigation strings 1–4
(Hamming code included) through `RXM-SFRBX`. They sit on the broadcast
resolution grid, so what the rover decodes is bit-identical to the orbit the
observations came from. A station can also carry per-channel GLONASS code
biases (`sim_station_t.glo_code_bias`), and the base can speak RTCM3 — 1005
plus MSM7 1077/1087 from RTKLIB's own encoder — instead of UBX
(`SIM_BASE_RTCM3`).

`test_ubx` (53 checks; 76 in the GLONASS build) covers RAWX round-tripping,
LNAV bit packing (decoded ephemeris agrees with truth to 4 mm of satellite
position), GLONASS string encode → decode (bit-exact state vector, adjacent
slots, channel recovery), cycle-slip flagging, and framing robustness against
junk, bad checksums and truncation.

`test_rtk` (24 checks; 36 in the GLONASS build) covers static and kinematic
convergence, recurring cycle slips, half-cycle flags, base-link dropout and
recovery, and accuracy against baseline length. The GLONASS build adds
GPS+GLONASS fixing, an RTCM3 MSM7 base, and the 2026-09-21 field fault:
per-channel GLONASS code biases plus a receiver-wide lock loss every 25 s.
That last one fixes on wrong integers (~1.5 m off) with RTKLIB's stock GLONASS
code weight and must never produce a wrong fix with `EFACT_GLO_CODE`.

Two bugs the tests caught, both worth knowing about:

- **Receiver clock bias in the transmit time.** RTKLIB derives transmit time
  as `t_obs − P/c − dts`, which because `P` contains `c·dtr` equals
  `t − r/c − dtr`. The simulator originally ignored `dtr`. Since rover and base
  clocks differ by ~10⁻⁴ s, each station's satellite position was displaced by
  up to 0.4 m of orbital motion — a per-satellite bias of ~1 cycle that does
  *not* cancel in the double difference. Ambiguity resolution never fixed.
  With it corrected, all single-differenced ambiguity errors collapse to a
  single constant (the unobservable common offset), exactly as theory says.
- **Cold-start filter poisoning.** Observations arrive before ephemerides do.
  With `dynamics=1`, `rtkpos()` coasts the state forward even when `pntpos()`
  fails, so those pre-ephemeris epochs drove the filter to 20 km errors it
  never recovered from. The engine now gates `rtkpos()` on having ephemerides
  for at least 4 satellites, which also stops a garbage position reaching the
  flight controller.

Both were in the *test rig*, not RTKLIB — but each would have appeared
identically as "RTK never fixes" on real hardware.

### On the target

`test_target` runs `test_ubx` and `test_rtk` on the RP2350 itself and prints
its own heap high-water mark and deepest stack use at the end (last run:
414 of 446 KB heap, 12,988 of 24,576 bytes stack). Two things make it fit:

- **One engine, on the heap.** The RTK tests borrow a single `rtkeng_t` and
  `sim_scenario_t` that the runner allocates once (`test_rtk_suite()`); the
  host `main()` does the same. Statically there were two of each, 340 KB of
  bss, which left the engine too little heap to start.
- **A real stack.** RTKLIB needs ~13 KB of stack; the SDK gives core 0 the two
  4 KB scratch banks with the heap free to grow up to them. `rtkpico_stack_guard()`
  (`pico/CMakeLists.txt`) reserves `RTKPICO_STACK_RESERVE` (16 KB) below the
  banks by overriding one SDK linker include, `pico/linker/section_end.incl`,
  so that `__StackLimit` -- where the SDK's own `_sbrk` stops the heap -- sits
  lower. It also enables `PICO_USE_STACK_GUARDS`, which on the RP2350 arms the
  stack-limit register `MSPLIM` at the new bottom, so an overflow faults
  instead of corrupting the heap. No SDK function is replaced. Re-check
  `section_end.incl` against the SDK's copy when upgrading the SDK.

The firmware apps do not use `rtkpico_stack_guard()` yet, though they run the
same `rtkpos()` path and so need the same stack.

### Replaying a field capture

The simulator cannot reproduce what a real sky and real receivers do, so field
problems are diagnosed from recordings instead. Build `rtk_bench` with the raw
dump enabled, record it, and replay the bytes on the host:

```bash
# firmware: every rover and correction byte also goes out as X,U / X,R lines.
# CMAKE_C_FLAGS replaces the SDK's CPU flags rather than adding to them, which
# is why they are spelled out.
cmake -S pico -B build/dump -DPICO_BOARD=pico2_w -DCMAKE_BUILD_TYPE=Release \
      -DRTKPICO_ENABLE_GLO=ON -DRTKPICO_ENABLE_GAL=OFF \
      -DCMAKE_C_FLAGS="-mcpu=cortex-m33 -mthumb -march=armv8-m.main+fp+dsp -mcmse -mfloat-abi=softfp -DRTK_BENCH_RAWDUMP=1"
cmake --build build/dump --target rtk_bench && ./pico/scripts/flash.sh build/dump/rtk_bench.elf

# record with the rover static and undisturbed
./pico/scripts/capture.py -o field.log --minutes 15

# replay is built against the GPS+GLONASS library, the pairing the firmware
# flies; a capture only reproduces with the same constellations compiled in
cmake -S pico/test -B build/host && cmake --build build/host --target replay
./build/host/replay field.log
```

The replay is bit-identical to what the board computed, so every engine
experiment runs at the desk: `--gps`, `--static`, `--thres`, `--minlock`,
`--elmin` and so on. The receiver's own `NAV-RELPOSNED` rides in the same UBX
stream and is the reference. With the rover static at a known baseline (take
the receiver's fixed result), two options answer most questions:

- `--ref E N U` scores every one of our solutions against it, so a *wrong*
  fix is counted as wrong rather than as a fix.
- `--dd E N U` prints per-satellite double-difference residuals of the raw
  observations. Code DDs should average zero; phase DDs should stay flat
  within an unbroken arc. A satellite that fails either feeds the filter a
  bias that no tuning will remove. `--ddtrace G26` prints one satellite's series.

Every replay also reports **receiver-wide carrier-lock losses**: epochs where
nearly every satellite's lock time resets at once while C/N0 holds steady.
That is the receiver, not the sky, and each one resets every ambiguity in our
engine and in the receiver's own RTK. `rtk_bench` shows the same count live on
its `lock:` line (rule in `port/lockloss.h`). To find the trigger, build it
with `-DRTK_BENCH_WIFI=0` (radio never powered, so no corrections either) or
`-DRTK_BENCH_TEE=0` (corrections still flow to our engine but are no longer
written to the receiver's UART), and compare rates with the antenna in the
same spot.

`--split PREFIX` writes the raw `.ubx` and `.rtcm3` streams for stock RTKLIB
(`convbin`, then `rnx2rtkp`), which settles whether a problem is ours or
RTKLIB's. Stock 2.4.3 b34 `rnx2rtkp -sys` is broken -- its switch has no
`break`s, so any letter enables every system -- so select systems with a config
file line such as `pos1-navsys=1` instead.

## Performance

Measured on the RP2350 at the stock 150 MHz, DWT cycle counter, `rtkpos()`
only (decode and NMEA are negligible by comparison):

| Configuration | nx | sats | mean | worst |
|---|---:|---:|---:|---:|
| GPS+GAL, AR fix+hold | 65 | 12 | 107 ms | 119 ms |
| GPS+GAL, AR off | 65 | 12 | 71 ms | 89 ms |
| GPS only, AR fix+hold | 35 | 7 | 43 ms | 51 ms |
| GPS only, AR off | 35 | 7 | 35 ms | 44 ms |
| GPS only, AR + `dynamics=1` | 41 | 7 | 110 ms | 117 ms |

**The state dimension is the lever, not the satellite count.**
`nx = NP + MAXSAT`, and `resamb_LAMBDA()` and `udpos()` form matrix products
over the *full* state dimension rather than over the satellites in view. Going
from GPS+GAL to GPS-only takes `MAXSAT` from 62 to 32 and cuts the epoch cost
by 2.5×, even though the visible-satellite count only drops from 12 to 7.

**`dynamics=1` is expensive.** It adds six states but, more importantly, a
full `F·P·Fᵀ` propagation — two `nx³` products per epoch. At `nx=41` that
costs more than everything else combined (110 ms vs 43 ms at `nx=35`). It is
off by default purely for that reason: it works correctly, but at 5 Hz with
GPS+Galileo it would not fit the epoch budget.

Leaving it off costs nothing, because the velocity ArduPilot receives does not
come from those states — `pntpos()` runs `estvel()` on the Doppler every epoch
and `relpos()` only overwrites `sol.rr[0..2]`, so `sol.rr[3..5]` carries a
Doppler velocity regardless of `dynamics`.

### Choosing a solution rate

At 150 MHz, GPS+GAL with ambiguity resolution leaves **5 Hz** as the practical
ceiling (60% of budget). If you want 10 Hz:

- build GPS-only (`-DRTKPICO_ENABLE_GAL=OFF`) → 51 ms worst case, comfortable;
- and/or raise `clk_sys` — RP2350 runs reliably well above 150 MHz, and the
  cost here is very nearly linear in clock;
- and/or set `dynamics=0`;
- `-DRTKPICO_COPY_TO_RAM=ON` removes XIP cache misses from the filter, now
  that there is SRAM to spare.

ArduPilot does not need a high GPS rate — its EKF fuses GPS at 5 Hz happily —
so 5 Hz GPS+GAL is usually the better trade than 10 Hz GPS-only.

### Double precision

Doubles use the RP2350 DCP coprocessor
(`pico_set_double_implementation(target pico)`), which is the single most
important switch in the build — RTKLIB is double precision end to end.
Confirm it is linked with:

```bash
grep -A2 "__wrap___aeabi_dmul" build/pico/drone-show-controller.elf.map   # -> double_aeabi_dcp.S.o
```

Measured primitive costs (`pico/app/perf_probe.c`): `atan2` 301 cycles,
`eph2pos` 13.3 k cycles (89 µs) per satellite, a 20×20×20 `matmul` 490 k
cycles — about 61 cycles per double multiply-accumulate including loads and
index arithmetic. GCC 10's objdump mis-decodes the DCP encodings as legacy
Maverick coprocessor mnemonics (`cfstrd`/`cfldrd`); they are cp4 accesses and
are correct.

## MAVLink bridge

The same Wi-Fi link carries MAVLink between a ground station / show controller
and the flight controller, and the RTK position reaches ArduPilot as
`GPS_INPUT` rather than NMEA.

```
 ground MAVLink server  <--UDP/Wi-Fi-->  Pico  <--UART, MAVLink2-->  ArduPilot
```

On the flight controller:

```
SERIAL2_PROTOCOL 2      # MAVLink2 on the port the Pico is wired to
SERIAL2_BAUD     115
GPS_TYPE         14     # MAV: position arrives as GPS_INPUT
```

This suits the Pico 2 W hwdef, where `UART0` is committed to the ELRS receiver
at 420 kbaud and PIO UARTs are restricted to GP16+, leaving exactly one wire
pair. `GPS_INPUT` also expresses what NMEA cannot: true RTK fixed/float fix
type, per-axis accuracy from the filter covariance, and GPS week/ms.

Traffic in both directions is forwarded **verbatim**. The bridge never
interprets mission or command messages — the mission protocol is stateful with
its own retries, and a middlebox that second-guesses it causes the failures it
is meant to prevent. It only frames traffic, so each datagram carries whole
messages, and learns the FC's system id from its heartbeats so injected
`GPS_INPUT` shares it.

### Not done yet

- **Mission verification** is by readback: have the ground station issue
  `MISSION_REQUEST_LIST` per drone and compare. That proves what the FC
  *stored*, not what crossed the wire. `MISSION_CHECKSUM` (id 53) is still in
  `development.xml`, not `common.xml`, so it is unavailable.
- **Synchronised start.** The intended mechanism is to send "start at GPS
  week/ms T" ahead of time and have each Pico fire `MAV_CMD_MISSION_START` at
  that instant from a hardware alarm — every unit already has RTK GPS time, so
  the fleet syncs to well under a millisecond regardless of Wi-Fi jitter. It
  must not be driven from the main loop, which `rtkpos()` blocks for ~107 ms.

### Safety notes

- The published velocity is gated. `estvel()` can leave `sol.rr[3..5]` stale if
  it fails to converge, and feeding the EKF a wrong velocity is far worse than
  feeding it none, so an implausible magnitude clears
  `GPS_INPUT_IGNORE_FLAG_VEL_*` rather than guessing.
- HDOP comes from real geometry via `dops()` over the satellites actually used,
  not a flattering constant — ArduPilot's arming checks read it.
- `alt` must be MSL but RTK gives ellipsoidal height. `MAV_GEOID_SEP` is a
  per-site constant, accurate to well under a centimetre over a show-sized
  area; a full geoid grid would cost ~60 KB of flash for no benefit.
- RTL and other failsafes travelling through the box that does the RTK compute
  is a coupling worth knowing about, substantially mitigated by ELRS on
  `SERIAL1` giving the FC an independent RC/GCS failsafe path.

### Testing

`test_mavlink` drives a simulated constellation through UBX, RTKLIB and the
bridge; `tools/check_mavlink.py` decodes the resulting `GPS_INPUT` with
pymavlink and compares it against the simulator's own truth, so a field-order,
unit or scaling error cannot pass silently. Current result: position within
**0.6 cm** and velocity within **0.000 m/s** of truth, `fix_type` 6 (RTK fixed).

`tools/gcs_probe.py` stands in for the ground station over UDP.
`tools/mavlink_wire.py` reprints the CRC_EXTRA values and field offsets from
pymavlink, which is where the constants in `mavlink_min.c` came from.

## Show lighting (WS2812B / SK6812 RGBW)

Skybrush authors light programs in Blender as **RGB**; the firmware converts to
RGB or RGBW at the driver. So receiving RGB and synthesising white loses
nothing, and a WS2812B strip is a faithful demo of an RGBW board.

Colour arrives as MAVLink `DEBUG_VECT` with `name = "rgb"` and x/y/z in 0..1
(show firmware `SHOW_LED0_TYPE 1`, `SHOW_LED0_CHAN` = the port the Pico is on).
It rides the MAVLink link that already exists, so lighting costs **no extra
wires**: 35 bytes per update, 875 B/s at 25 Hz, 7.6% of a 115200 link.

If the light stream stops for `SHOW_LED_TIMEOUT_MS`, the strip goes dark rather
than freezing on a stale colour.

### One timing set for both chips

WS2812B is 24-bit GRB, SK6812 RGBW is 32-bit GRBW, but their pulse windows
overlap. At a 20 MHz state-machine clock with 25 cycles per bit
(`T1=8 T2=6 T3=11`):

| | measured | WS2812B | SK6812 |
|---|---|---|---|
| T0H | 0.400 µs | 0.25–0.55 | 0.15–0.45 |
| T0L | 0.850 µs | 0.70–1.00 | 0.60–1.05 |
| T1H | 0.700 µs | 0.65–0.95 | 0.45–0.75 |
| T1L | 0.550 µs | 0.30–0.60 | 0.45–1.05 |

So changing chips is a configuration change, not a rewrite. Writing is
DMA-driven — a 30-LED frame is ~0.9 ms on the wire, which must not block a loop
that also runs `rtkpos()`.

### Bring-up without parts

`led_demo` runs two self-tests that need no LEDs and no instruments:

- **Wire format.** A second PIO state machine samples the pin the first one is
  driving (the pad's input buffer stays live), captures at 40 MHz via DMA, and
  checks pulse widths against the datasheet, then decodes the colour back.
- **Light path.** The PL011's internal loopback bit (`UARTCR.LBE`) ties TX to
  RX inside the peripheral, so a real `DEBUG_VECT` goes out a real UART and
  comes back through the frame parser and light decoder.

```
=== WS2812B / SK6812 driver bring-up ===
  T0H 0.400 us  T0L 0.850 us  T1H 0.700 us  T1L 0.550 us
  pixel 0 decoded 0x000FF0A5, expected 0x000FF0A5      RESULT: PASS
--- UART loopback: DEBUG_VECT "rgb" -> light command ---
  sent R32 G128 B192 ... decoded R 32 G128 B192        RESULT: PASS
```

### Breadboard wiring

```
GP16 --470R--> 74AHCT125 --> DIN     3.3 V direct is marginal: WS2812B wants
5V  --------------------- V+        ~0.7*VDD for a logic 1. It will work on the
GND --------------------- GND       bench and fail in the air.
                                     1000 uF across V+/GND at the strip.
```

Ground must be common with the Pico. At ~60 mA per LED at full white a few run
off USB; the driver defaults to brightness 64 for that reason.

## Known limitations

- Galileo I/NAV page encoding is not implemented in the simulator, so Galileo
  ephemerides are injected directly; the GPS satellites keep the full
  byte-level path under test.
- Simulated accuracy is idealised (no multipath correlation, no antenna phase
  centre, truth and estimator share an orbit model). The tests validate the
  pipeline and the numerics, not absolute field accuracy.
- GLONASS is off by default: FDMA needs per-receiver inter-channel bias
  calibration for single-frequency AR, and costs 24 more states.
