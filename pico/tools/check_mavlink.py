#!/usr/bin/env python3
"""
Cross-validate the GPS_INPUT the bridge produces against pymavlink and against
the simulator's own truth position.

    ./build/host/test_mavlink | ./pico/tools/check_mavlink.py

A field-order, unit or scaling error in mav_bridge.c shows up here as a
position that disagrees with truth, which a self-consistent C test could not
catch on its own.
"""
import sys, math
from pymavlink.dialects.v20 import common as mav

hexs = truth = stat = None
tvel = None
for line in sys.stdin:
    if line.startswith("GPS_INPUT_HEX"): hexs = line.split()[1]
    elif line.startswith("TRUTH_LLH"):   truth = [float(x) for x in line.split()[1:]]
    elif line.startswith("TRUTH_VEL_NED"): tvel = [float(x) for x in line.split()[1:]]
    elif line.startswith("SOL_STAT"):    stat = line.split()

if not hexs or not truth:
    print("no GPS_INPUT/TRUTH in input"); sys.exit(1)

m = mav.MAVLink(None); m.robust_parsing = True
msgs = m.parse_buffer(bytes.fromhex(hexs))
if not msgs:
    print("FAIL: pymavlink rejected the frame (CRC or framing)"); sys.exit(1)
g = msgs[0]
if g.get_type() != "GPS_INPUT":
    print("FAIL: got", g.get_type()); sys.exit(1)

lat, lon, alt = g.lat / 1e7, g.lon / 1e7, g.alt
tlat, tlon, talt = truth

# metres of error, using a local flat approximation
dn = (lat - tlat) * 111320.0
de = (lon - tlon) * 111320.0 * math.cos(math.radians(tlat))
dh = alt - talt
horiz = math.hypot(dn, de)

print(f"  pymavlink decoded GPS_INPUT from sys={g.get_srcSystem()} comp={g.get_srcComponent()}")
print(f"  fix_type            {g.fix_type}  (6 = RTK fixed)")
print(f"  satellites_visible  {g.satellites_visible}")
print(f"  gps_id              {g.gps_id}")
print(f"  time_week/ms        {g.time_week} / {g.time_week_ms}")
print(f"  hdop / vdop         {g.hdop:.2f} / {g.vdop:.2f}")
print(f"  reported accuracy   horiz {g.horiz_accuracy*100:.1f} cm, vert {g.vert_accuracy*100:.1f} cm")
print(f"  velocity NED        {g.vn:+.3f} {g.ve:+.3f} {g.vd:+.3f} m/s")
print(f"  ignore_flags        0x{g.ignore_flags:02x}")
print()
print(f"  position vs truth   horizontal {horiz*100:.2f} cm, vertical {dh*100:.2f} cm")

ok = True
def chk(cond, msg):
    global ok
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond: ok = False

chk(g.fix_type == 6,              "fix_type reports RTK FIXED")
chk(horiz < 0.10,                 f"horizontal position within 10 cm of truth ({horiz*100:.2f} cm)")
chk(abs(dh) < 0.20,               f"vertical position within 20 cm of truth ({dh*100:.2f} cm)")
chk(0 < g.hdop < 5,               f"HDOP plausible ({g.hdop:.2f}) - ArduPilot arming checks this")
chk(g.horiz_accuracy < 0.5,       "reported horizontal accuracy is centimetric")
chk(g.time_week > 2000,           "GPS week is set (ArduPilot needs it to set time)")
chk(g.satellites_visible > 4,     "satellite count is sane")
chk((g.ignore_flags & 0x02) == 0, "HDOP is not marked ignored")

if tvel:
    vn_t, ve_t, vd_t = tvel
    speed_t = math.hypot(vn_t, ve_t)
    verr = math.sqrt((g.vn-vn_t)**2 + (g.ve-ve_t)**2 + (g.vd-vd_t)**2)
    vel_claimed = (g.ignore_flags & 0x08) == 0
    print(f"\n  truth velocity NED  {vn_t:+.3f} {ve_t:+.3f} {vd_t:+.3f} m/s (speed {speed_t:.2f})")
    print(f"  velocity error      {verr:.3f} m/s   claimed_valid={vel_claimed}")
    if vel_claimed:
        chk(verr < 1.0, f"published velocity within 1 m/s of truth ({verr:.3f})")
    else:
        chk(True, "velocity correctly marked as unavailable")
print("\n  RESULT:", "PASS" if ok else "FAIL")
sys.exit(0 if ok else 1)
