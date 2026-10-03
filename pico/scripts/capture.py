#!/usr/bin/env python3
"""
Record the bench firmware's USB output to a file, for offline analysis.

    ./pico/scripts/capture.py -o field.log --minutes 15
    ./pico/scripts/capture.py -o field.log --port /dev/cu.usbmodem12101

With rtk_bench built with -DRTK_BENCH_RAWDUMP=1 the file can be replayed
through the host engine (pico/test/replay.c); without it, it is still the
ordinary D-line and status log.

The port is found by USB product name ("Pico", never the "Debugprobe"), or by
--serial; macOS renumbers /dev/cu.usbmodem* on every replug, so hard-coding a
path goes stale.

Watch the gap warnings. The Pico drops USB output it cannot deliver rather
than stall its main loop, so a host that stops reading -- a busy laptop, a
sleeping lid -- leaves a hole in the capture that nothing else will report.
Replay tolerates a hole (it just restarts the filter), but the seconds are
gone. Keep heavy work off the machine while it records.
"""
import argparse, re, sys, time

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    sys.exit("needs pyserial:  python3 -m pip install pyserial")


def find_port(serial_number=None):
    ports = []
    for p in serial.tools.list_ports.comports():
        if serial_number:
            if p.serial_number == serial_number:
                return p.device
        elif p.product == "Pico":
            ports.append(p.device)
    if len(ports) == 1:
        return ports[0]
    if not ports:
        sys.exit("no Pico found on USB; pass --port or --serial")
    sys.exit("several Picos found (%s); pass --port or --serial" % ", ".join(ports))


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("-o", "--output", required=True, help="file to write")
    ap.add_argument("--minutes", type=float, default=15.0, help="how long to record")
    ap.add_argument("--port", help="serial device (default: find the Pico)")
    ap.add_argument("--serial", help="USB serial number of the board to use")
    args = ap.parse_args()

    port = args.port or find_port(args.serial)
    dev = serial.Serial(port, 115200, timeout=0.5)   # USB CDC: baud is ignored
    end = time.time() + args.minutes * 60.0
    last_report = time.time()
    total = xlines = dlines = 0
    last_uptime = None
    tail = b""
    uptime_re = re.compile(rb"^D,([0-9]+\.[0-9]+),")
    print("recording %s -> %s for %.1f min" % (port, args.output, args.minutes), file=sys.stderr)

    with open(args.output, "wb") as out:
        while time.time() < end:
            chunk = dev.read(65536)
            if not chunk:
                continue
            out.write(chunk)
            total += len(chunk)
            # Line bookkeeping only for the progress report and gap check.
            lines = (tail + chunk).split(b"\n")
            tail = lines.pop()
            for ln in lines:
                if ln.startswith(b"X,"):
                    xlines += 1
                    continue
                m = uptime_re.match(ln)
                if not m:
                    continue
                dlines += 1
                up = float(m.group(1))
                if last_uptime is not None and up - last_uptime > 2.0:
                    print("  GAP: device uptime %.1f -> %.1f s, %.0f s of output lost"
                          % (last_uptime, up, up - last_uptime), file=sys.stderr)
                last_uptime = up
            if time.time() - last_report >= 30.0:
                last_report = time.time()
                left = max(0.0, end - time.time())
                print("  %6.1f MB  %7d raw lines  %6d solutions  %4.1f min left"
                      % (total / 1e6, xlines, dlines, left / 60.0), file=sys.stderr)
    print("done: %.1f MB, %d raw lines, %d solutions" % (total / 1e6, xlines, dlines),
          file=sys.stderr)
    if xlines == 0:
        print("note: no X, lines -- this firmware was not built with -DRTK_BENCH_RAWDUMP=1,"
              " so the capture cannot be replayed", file=sys.stderr)


if __name__ == "__main__":
    main()
