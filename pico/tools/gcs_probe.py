#!/usr/bin/env python3
"""
Stand in for the ground station / show controller while testing the bridge.

Sends real MAVLink 2 to the Pico's UDP port and prints anything it sends back.
With no flight controller attached there is nothing to reply, so this verifies
the ground->vehicle direction and the socket; with an FC on the UART it becomes
a usable one-drone GCS.

    ./pico/tools/gcs_probe.py --host 192.168.4.168 --port 14555 --count 20
"""
import argparse, socket, time
from pymavlink.dialects.v20 import common as mav

ap = argparse.ArgumentParser()
ap.add_argument("--host", default="192.168.4.168")
ap.add_argument("--port", type=int, default=14555)
ap.add_argument("--count", type=int, default=20)
ap.add_argument("--rtl", action="store_true", help="also send a RETURN_TO_LAUNCH")
ap.add_argument("--bind", type=int, default=0,
                help="local port; 0 = ephemeral. Use 14550 only if no GCS holds it.")
a = ap.parse_args()

s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
s.settimeout(0.2)
s.bind(("0.0.0.0", a.bind))

m = mav.MAVLink(None, srcSystem=255, srcComponent=190)   # 255 = GCS
sent = rx = 0
for i in range(a.count):
    pkt = m.heartbeat_encode(6, 8, 0, 0, 4).pack(m)      # MAV_TYPE_GCS
    if a.rtl and i == 3:
        pkt += m.command_long_encode(1, 1, 20, 0, 0,0,0,0,0,0,0).pack(m)  # RTL
        print("  -> sent MAV_CMD_NAV_RETURN_TO_LAUNCH")
    s.sendto(pkt, (a.host, a.port))
    sent += len(pkt)
    try:
        data, addr = s.recvfrom(2048)
        rx += len(data)
        for msg in mav.MAVLink(None).parse_buffer(data) or []:
            print(f"  <- {msg.get_type()} from sys={msg.get_srcSystem()} "
                  f"comp={msg.get_srcComponent()}")
    except socket.timeout:
        pass
    time.sleep(0.1)

print(f"\n  sent {sent} bytes to {a.host}:{a.port}, received {rx} bytes")
