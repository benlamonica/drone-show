#!/usr/bin/env python3
"""
Regenerate the wire constants baked into mavlink_min.[ch].

MAVLink orders payload fields by descending type size (extensions appended in
declaration order) and mixes a per-message CRC_EXTRA into the checksum. Both
are easy to get wrong by hand, so print them from pymavlink and compare.

    ./pico/tools/mavlink_wire.py
"""
import struct
from pymavlink.dialects.v20 import common

for name in ("HEARTBEAT", "GPS_INPUT"):
    mid = getattr(common, "MAVLINK_MSG_ID_" + name)
    cls = common.mavlink_map[mid]
    print(f"\n{name}: id={mid} CRC_EXTRA={cls.crc_extra} payload_len={cls.unpacker.size}")
    off = 0
    for c, fn in zip([c for c in cls.unpacker.format if c != '<'],
                     cls.ordered_fieldnames):
        sz = struct.calcsize(c)
        print(f"   off={off:3d} size={sz}  {fn}")
        off += sz
