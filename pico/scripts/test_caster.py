#!/usr/bin/env python3
"""
Minimal NTRIP caster for bench-testing the Pico RTK firmware.

Serves valid RTCM3 1005 (Stationary RTK Reference Station ARP) frames after an
NTRIP handshake, so the firmware's network path can be exercised without the
real base station: Wi-Fi -> TCP -> NTRIP header -> ring buffer -> input_rtcm3()
-> base position adopted by the engine.

    ./pico/scripts/test_caster.py [--port 2101] [--mount RTCM3]

This is a test fixture, not a substitute for str2str: it carries a station
position but no observations, so the firmware will show corrections arriving
and a base position adopted, and will still report a single-point solution.
"""
import argparse, socket, struct, threading, time, sys

def crc24q(data: bytes) -> int:
    """CRC-24Q (Qualcomm), the RTCM3 frame check."""
    crc = 0
    for b in data:
        crc ^= b << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
    return crc & 0xFFFFFF

class Bits:
    def __init__(self): self.v, self.n = 0, 0
    def put(self, val, nbits):
        self.v = (self.v << nbits) | (int(val) & ((1 << nbits) - 1)); self.n += nbits
    def puts(self, val, nbits):                      # signed
        self.put(int(val) & ((1 << nbits) - 1), nbits)
    def bytes(self):
        pad = (-self.n) % 8
        return (self.v << pad).to_bytes((self.n + pad) // 8, "big")

def rtcm1005(staid, x, y, z):
    """Stationary RTK Reference Station ARP; ECEF metres."""
    b = Bits()
    b.put(1005, 12); b.put(staid, 12); b.put(0, 6)   # msg, station id, ITRF year
    b.put(1, 1); b.put(0, 1); b.put(0, 1)            # GPS / GLONASS / Galileo
    b.put(0, 1)                                      # reference-station indicator
    b.puts(round(x * 10000), 38)                     # ECEF-X, 0.1 mm
    b.put(0, 1); b.put(0, 1)                         # osc indicator, reserved
    b.puts(round(y * 10000), 38)
    b.put(0, 2)                                      # quarter cycle indicator
    b.puts(round(z * 10000), 38)
    return frame(b.bytes())

def frame(payload: bytes) -> bytes:
    hdr = b"\xD3" + struct.pack(">H", len(payload) & 0x3FF)
    return hdr + payload + crc24q(hdr + payload).to_bytes(3, "big")

def serve(conn, addr, mount, pos):
    print(f"[caster] connection from {addr[0]}:{addr[1]}", flush=True)
    conn.settimeout(5.0)
    try:
        req = conn.recv(1024).decode("latin-1", "replace")
    except socket.timeout:
        req = ""
    first = req.split("\r\n")[0] if req else "(no request line)"
    print(f"[caster] request: {first}", flush=True)

    if req.startswith("GET"):
        want = req.split()[1].lstrip("/") if len(req.split()) > 1 else ""
        if want and want != mount:
            print(f"[caster] mountpoint '{want}' != '{mount}', sending SOURCETABLE",
                  flush=True)
            body = f"STR;{mount};;RTCM 3;1005;;;;;;0.00;0.00;0;0;;;N;N;0;\r\nENDSOURCETABLE\r\n"
            conn.sendall(("SOURCETABLE 200 OK\r\nContent-Length: %d\r\n\r\n%s"
                          % (len(body), body)).encode())
            conn.close(); return
        conn.sendall(b"ICY 200 OK\r\n\r\n")
        print(f"[caster] accepted /{mount}, streaming RTCM3 1005 at 1 Hz", flush=True)
    else:
        print("[caster] no NTRIP request; streaming anyway (plain TCP mode)", flush=True)

    msg, n = rtcm1005(1234, *pos), 0
    try:
        while True:
            conn.sendall(msg); n += len(msg)
            if n % (len(msg) * 5) == 0:
                print(f"[caster] sent {n} bytes", flush=True)
            time.sleep(1.0)
    except (BrokenPipeError, ConnectionResetError, OSError):
        print(f"[caster] client gone after {n} bytes", flush=True)
    finally:
        conn.close()

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=2101)
    ap.add_argument("--mount", default="RTCM3")
    ap.add_argument("--pos", default="-3961905.0,3348994.0,3698212.0",
                    help="base ECEF x,y,z in metres")
    a = ap.parse_args()
    pos = tuple(float(v) for v in a.pos.split(","))

    s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind(("0.0.0.0", a.port)); s.listen(4)
    print(f"[caster] listening on 0.0.0.0:{a.port} mount=/{a.mount}", flush=True)
    while True:
        conn, addr = s.accept()
        threading.Thread(target=serve, args=(conn, addr, a.mount, pos),
                         daemon=True).start()

if __name__ == "__main__":
    main()
