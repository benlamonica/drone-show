"""Stitch a stranded ground pad's pour island to the other layer: search a fine grid
near the pad for a spot inside the pour on both layers where a via clears everything,
and keep the first via that lowers the unconnected count."""
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import escape as es  # noqa: E402
from gen_pcb import PCB  # noqa: E402

mm = pcbnew.FromMM


def count(b):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.WriteDRCReport(b, "/tmp/lc_gi.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    t = open("/tmp/lc_gi.rpt").read()
    pads = set()
    for blk in re.split(r"\n(?=\[)", t):
        if blk.startswith("[unconnected_items]"):
            pads |= {(ref, num) for num, ref in re.findall(r"Pad (\S+) \[/GND\] of (\S+) on", blk)}
    return (len(re.findall(r"^\[unconnected_items\]", t, re.M)),
            len(re.findall(r"^\[(clearance|hole_clearance|hole_near_hole)\]", t, re.M)), sorted(pads))


def fills(b):
    out = {L: pcbnew.SHAPE_POLY_SET() for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
    for z in b.Zones():
        if z.GetNetname() == "/GND":
            for L in out:
                if z.IsOnLayer(L):
                    out[L].Append(z.GetFilledPolysList(L))
    return out


def main():
    b = pcbnew.LoadBoard(PCB)
    n0, c0, stranded = count(b)
    for ref, num in stranded:
        b = pcbnew.LoadBoard(PCB)
        n0, c0, _ = count(b)
        fl = fills(b)
        c = b.FindFootprintByReference(ref).FindPadByNumber(num).GetPosition()
        pts = []
        for i in range(-15, 16):
            for j in range(-15, 16):
                p = pcbnew.VECTOR2I(c.x + mm(0.2 * i), c.y + mm(0.2 * j))
                if all(fl[L].Collide(p, -mm(0.3)) for L in fl):      # 0.3 mm inside both pours
                    pts.append(p)
        pts.sort(key=lambda p: (p - c).EuclideanNorm())
        tried = 0
        for p in pts:
            if tried >= 25:
                break
            if es.collides(b, "/GND", [(None, pcbnew.SHAPE_CIRCLE(p, mm(0.3)))]) or not es.holes_ok(b, p):
                continue
            tried += 1
            v = pcbnew.PCB_VIA(b)
            v.SetPosition(p)
            v.SetWidth(mm(0.6))
            v.SetDrill(mm(0.3))
            v.SetNet(b.FindNet("/GND"))
            b.Add(v)
            n1, c1, _ = count(b)
            if n1 < n0 and c1 <= c0:
                pcbnew.SaveBoard(PCB, b)
                print(f"{ref}.{num}: via at {pcbnew.ToMM(p)}, open {n0}->{n1}", flush=True)
                break
            b.Delete(v)
        else:
            print(f"{ref}.{num}: no spot", flush=True)
            continue
        if tried >= 25:
            print(f"{ref}.{num}: no spot", flush=True)


if __name__ == "__main__":
    main()
