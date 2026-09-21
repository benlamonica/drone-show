#!/usr/bin/env python3
"""rtk_scatter.py : what spacing can two aircraft actually hold?

Reads a log captured from app/rtk_bench.c and reports the *scatter* of a
stationary rover's baseline, not its mean. The mean is the survey error and is
common to every aircraft, so it cancels in the formation; the scatter does not,
and it is the floor on how tightly two aircraft can be flown.

    python3 rtk_scatter.py rtk.csv
    python3 rtk_scatter.py rtk.csv --settle 120     # drop the first 2 minutes

Only lines beginning "D," are used, so a raw screen/picocom capture with the
human-readable summary interleaved can be passed straight in.
"""
import sys, math, argparse

COLS = ("t_s stat ns age_s ratio E_m N_m U_m sdE sdN sdU rtkpos_us "
        "link reconn rssi "
        "u_carr u_E u_N u_U u_accE u_accN u_accD u_sv").split()
QNAME = {1: "FIX", 2: "FLOAT", 3: "SBAS", 4: "DGPS", 5: "SINGLE", 6: "PPP"}

def load(path):
    rows = []
    for line in open(path, errors="replace"):
        line = line.strip()
        if not line.startswith("D,"):
            continue
        f = line[2:].split(",")
        if len(f) < 12 or f[0] == "t_s":       # skip the header line
            continue
        try:
            r = dict(zip(COLS, f))
            for k in ("t_s","age_s","ratio","E_m","N_m","U_m","sdE","sdN","sdU"):
                r[k] = float(r[k])
            for k in ("stat","ns"):
                r[k] = int(r[k])
            # u-blox columns are absent in logs from before the dual-engine build
            for k in ("u_E","u_N","u_U","u_accE","u_accN","u_accD"):
                r[k] = float(r.get(k, 0.0) or 0.0)
            for k in ("u_carr","u_sv"):
                r[k] = int(r.get(k, 0) or 0)
            rows.append(r)
        except ValueError:
            continue                            # truncated line from the capture
    return rows

def stats(vals):
    n = len(vals)
    if n < 2:
        return (0.0, 0.0, 0.0, 0.0)
    m = sum(vals) / n
    sd = math.sqrt(sum((v - m) ** 2 for v in vals) / (n - 1))
    return (m, sd, min(vals) - m, max(vals) - m)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--settle", type=float, default=0.0,
                    help="ignore the first N seconds while the filter converges")
    a = ap.parse_args()

    rows = load(a.log)
    if not rows:
        sys.exit("no 'D,' data lines found in %s" % a.log)

    t0 = rows[0]["t_s"]
    rows = [r for r in rows if r["t_s"] - t0 >= a.settle]
    span = rows[-1]["t_s"] - rows[0]["t_s"] if len(rows) > 1 else 0.0

    print("samples %d over %.0f s (%.1f Hz)" % (len(rows), span,
          len(rows) / span if span else 0))

    counts = {}
    for r in rows:
        counts[r["stat"]] = counts.get(r["stat"], 0) + 1
    print("\nsolution mix:")
    for q, c in sorted(counts.items(), key=lambda kv: -kv[1]):
        print("  %-7s %5d  %5.1f%%" % (QNAME.get(q, str(q)), c, 100.0 * c / len(rows)))

    # Correction health. A dropout inflates age_s, which is the honest
    # explanation for an epoch that went bad -- separate that from RTK noise.
    ages = [r["age_s"] for r in rows]
    stale = sum(1 for x in ages if x > 5.0)
    reconn = 0
    try:
        reconn = int(rows[-1]["reconn"]) - int(rows[0]["reconn"])
    except (KeyError, ValueError):
        pass
    print("\ncorrection link:")
    print("  differential age  mean %.1f s  max %.1f s" % (sum(ages)/len(ages), max(ages)))
    print("  epochs with age > 5 s : %d (%.1f%%)" % (stale, 100.0*stale/len(rows)))
    print("  reconnects during run : %d" % reconn)
    if reconn:
        print("  -> corrections were interrupted; scatter below is pessimistic.")

    for want, label in ((1, "FIX"), (2, "FLOAT")):
        sub = [r for r in rows if r["stat"] == want]
        if len(sub) < 2:
            continue
        print("\n%s epochs: %d" % (label, len(sub)))
        tot = 0.0
        for axis in ("E_m", "N_m", "U_m"):
            m, sd, lo, hi = stats([r[axis] for r in sub])
            if axis != "U_m":
                tot += sd * sd
            print("  %s  mean %+9.4f m   sd %6.4f m   range %+.3f/%+.3f m"
                  % (axis[0], m, sd, lo, hi))
        hor = math.sqrt(tot)
        print("  horizontal sd (2D) : %.4f m" % hor)
        # Two independent rovers each wander by this much, so their separation
        # error is the quadrature sum -- sqrt(2) larger than one rover's sd.
        print("  => spacing error between two aircraft, 1-sigma: %.3f m" % (hor*math.sqrt(2)))
        print("     keep at least %.2f m between aircraft for 3-sigma clearance"
              % (3.0 * hor * math.sqrt(2)))

    # --- the receiver's own RTK engine, on identical corrections -------------
    # Both solutions come from one antenna and one correction stream, so the
    # only difference between them is the engine. Their *disagreement* is the
    # most informative number here: two independent engines converging on the
    # same millimetres is strong evidence both are right, while a decimetre
    # apart means at least one is resolving the wrong integers.
    ub = [r for r in rows if r["u_carr"] == 2]
    if ub:
        print("\n%s" % ("-" * 58))
        print("u-blox hardware RTK (same antenna, same corrections)")
        ucounts = {}
        for r in rows:
            ucounts[r["u_carr"]] = ucounts.get(r["u_carr"], 0) + 1
        for q, label in ((2, "FIX"), (1, "FLOAT"), (0, "none")):
            if q in ucounts:
                print("  %-7s %5d  %5.1f%%" % (label, ucounts[q],
                                               100.0 * ucounts[q] / len(rows)))
        print("  satellites used: mean %.1f" %
              (sum(r["u_sv"] for r in rows) / len(rows)))
        print("\n  FIX epochs: %d" % len(ub))
        utot = 0.0
        for axis, label in (("u_E", "E"), ("u_N", "N"), ("u_U", "U")):
            m, sd, lo, hi = stats([r[axis] for r in ub])
            if axis != "u_U":
                utot += sd * sd
            print("    %s  mean %+9.4f m   sd %6.4f m   range %+.3f/%+.3f m"
                  % (label, m, sd, lo, hi))
        print("    horizontal sd (2D) : %.4f m" % math.sqrt(utot))

        # agreement, over epochs where BOTH engines fixed independently
        both = [r for r in rows if r["stat"] == 1 and r["u_carr"] == 2]
        if len(both) > 1:
            print("\n  agreement with RTKLIB over %d epochs where both fixed:" % len(both))
            dtot = 0.0
            for a, b, label in (("E_m","u_E","E"), ("N_m","u_N","N"), ("U_m","u_U","U")):
                d = [r[a] - r[b] for r in both]
                m, sd, lo, hi = stats(d)
                if label != "U":
                    dtot += m * m
                print("    d%s  mean %+8.4f m   sd %6.4f m" % (label, m, sd))
            print("    horizontal offset between engines: %.4f m" % math.sqrt(dtot))
            print("    -> a constant offset is a frame or base-position difference;")
            print("       scatter in the difference is one engine being noisier.")
        else:
            print("\n  (no epochs where both engines fixed simultaneously)")
    elif any(r["u_carr"] for r in rows):
        print("\nu-blox hardware RTK: reached FLOAT but never FIX.")
    else:
        print("\nu-blox hardware RTK: no fixed epochs logged "
              "(older capture, or corrections never reached the receiver).")

    t = [r["rtkpos_us"] for r in rows]
    try:
        t = [float(x) for x in t]
        print("\nrtkpos: mean %.0f us  max %.0f us  (budget %.0f us at 5 Hz)"
              % (sum(t)/len(t), max(t), 200000.0))
    except ValueError:
        pass

if __name__ == "__main__":
    main()
