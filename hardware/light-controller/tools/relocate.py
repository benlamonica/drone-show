"""Move a small part whose ground pad can't be reached: try nearby spots and
rotations on both sides where its pads clear all copper, reconnect its other pad
with the grid router and its ground pad with gnd_hop, keep the best that lowers
the unconnected count without new clearance errors.

    python3 tools/relocate.py C19 [C20 ...]
"""
import math
import os
import re
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import escape as es  # noqa: E402
import finish_route as fr  # noqa: E402

mm = pcbnew.FromMM
HERE = os.path.dirname(os.path.abspath(__file__))


def count():
    b = pcbnew.LoadBoard(fr.PCB)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.WriteDRCReport(b, "/tmp/lc_rl.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    t = open("/tmp/lc_rl.rpt").read()
    return (len(re.findall(r"^\[unconnected_items\]", t, re.M)),
            len(re.findall(r"^\[(clearance|hole_clearance|hole_near_hole|courtyards_overlap)\]", t, re.M)))


def clear(b, fp):
    for p in fp.Pads():
        L = pcbnew.B_Cu if fp.IsFlipped() else pcbnew.F_Cu
        sh = p.GetEffectiveShape(L)
        for t in b.GetTracks():
            if t.IsOnLayer(L) and t.GetEffectiveShape(L).Collide(sh, mm(0.16)):
                return False
        for f2 in b.GetFootprints():
            if f2.GetReference() == fp.GetReference():
                continue
            for q in f2.Pads():
                if q.IsOnLayer(L) and q.GetEffectiveShape(L).Collide(sh, mm(0.25)):
                    return False
    # courtyard: keep 0.25 mm from other parts' pads on the same side (approximation)
    return True


def main():
    for ref in sys.argv[1:]:
        base = "/tmp/rl_base.kicad_pcb"
        subprocess.run(["cp", fr.PCB, base])
        n0, c0 = count()
        b = pcbnew.LoadBoard(fr.PCB)
        fp = b.FindFootprintByReference(ref)
        x0, y0 = fp.GetPosition().x, fp.GetPosition().y
        cands = []
        for r in (0.0, 1.0, 1.6, 2.2, 3.0, 4.0):
            for k in range(1 if r == 0 else 12):
                a = 2 * math.pi * k / 12
                for side in (fp.IsFlipped(), not fp.IsFlipped()):
                    for rot in (0, 90, 180, 270):
                        cands.append((r, x0 + mm(r) * math.cos(a), y0 + mm(r) * math.sin(a), side, rot))
        best = None
        tried = 0
        for r, x, y, side, rot in cands:
            if tried >= 12:
                break
            b = pcbnew.LoadBoard(base)
            fp = b.FindFootprintByReference(ref)
            nets = {p.GetNetname() for p in fp.Pads()}
            for t in list(b.GetTracks()):
                if t.GetClass() == "PCB_TRACK" and t.GetNetname() in nets and any(
                        t.IsOnLayer(L) and p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(t.GetEffectiveShape(L), 0)
                        for p in fp.Pads() for L in (pcbnew.F_Cu, pcbnew.B_Cu)):
                    b.Delete(t)
            if fp.IsFlipped() != side:
                fp.Flip(fp.GetPosition(), False)
            fp.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
            fp.SetOrientationDegrees(rot)
            if not clear(b, fp):
                continue
            tried += 1
            pcbnew.SaveBoard(fr.PCB, b)
            for n in sorted(nets - {"/GND"}):
                subprocess.run(["python3", os.path.join(HERE, "finish_route.py"), n], capture_output=True)
            subprocess.run(["python3", os.path.join(HERE, "gnd_hop.py")], capture_output=True)
            n1, c1 = count()
            print(f"{ref} r={r} side={'B' if side else 'F'} rot={rot}: open {n0}->{n1} clr {c0}->{c1}", flush=True)
            if n1 < n0 and c1 <= c0 and (best is None or n1 < best[0]):
                best = (n1, open(fr.PCB).read())
                if n1 <= n0 - 2:
                    break
        if best:
            open(fr.PCB, "w").write(best[1])
            print(f"{ref}: moved, open {n0}->{best[0]}")
        else:
            subprocess.run(["cp", base, fr.PCB])
            print(f"{ref}: left in place")


if __name__ == "__main__":
    main()
