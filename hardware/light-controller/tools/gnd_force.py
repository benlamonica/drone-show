"""Give a stranded ground pad its own via even where signal tracks are in the way:
place the via and stub clear of every pad and hole, take up the signal tracks they
hit, and reroute those nets. Kept only when DRC reports fewer unconnected items."""
import math
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import escape as es  # noqa: E402
import finish_route as fr  # noqa: E402

mm = pcbnew.FromMM


def report(b):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.WriteDRCReport(b, "/tmp/lc_gf.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    t = open("/tmp/lc_gf.rpt").read()
    n_open = len(re.findall(r"^\[unconnected_items\]", t, re.M))
    n_clr = len(re.findall(r"^\[(clearance|hole_clearance|hole_near_hole)\]", t, re.M))
    pads = set()
    for blk in re.split(r"\n(?=\[)", t):
        if blk.startswith("[unconnected_items]"):
            pads |= {(ref, num) for num, ref in re.findall(r"Pad (\S+) \[/GND\] of (\S+) on", blk)}
    return n_open, n_clr, sorted(pads)


def pads_only_collide(b, shapes):
    for fp in b.GetFootprints():
        for p in fp.Pads():
            if p.GetNetname() == "/GND":
                continue
            for ly, sh in shapes:
                for L in ((ly,) if ly is not None else (pcbnew.F_Cu, pcbnew.B_Cu)):
                    if p.IsOnLayer(L) and p.GetEffectiveShape(L).Collide(sh, mm(0.16)):
                        return True
    return False


def candidates(pad):
    c = pad.GetPosition()
    for r in (0.95, 1.15, 1.4, 1.7):
        for k in range(16):
            a = 2 * math.pi * k / 16
            yield pcbnew.VECTOR2I(int(c.x + mm(r) * math.cos(a)), int(c.y + mm(r) * math.sin(a)))


def attempt(ref, num):
    b = pcbnew.LoadBoard(fr.PCB)
    n0, c0, _ = report(b)
    pad = b.FindFootprintByReference(ref).FindPadByNumber(num)
    layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
    for pos in candidates(pad):
        if pcbnew.ToMM(pos.x) < -29.0 or abs(pcbnew.ToMM(pos.y)) > 15.0:
            continue
        shapes = [(None, pcbnew.SHAPE_CIRCLE(pos, mm(0.3))),
                  (layer, pcbnew.SHAPE_SEGMENT(pad.GetPosition(), pos, mm(0.25)))]
        if pads_only_collide(b, shapes) or not es.holes_ok(b, pos):
            continue
        for z in b.Zones():
            z.UnFill()
        v = pcbnew.PCB_VIA(b)
        v.SetPosition(pos)
        v.SetWidth(mm(0.6))
        v.SetDrill(mm(0.3))
        v.SetNet(pad.GetNet())
        b.Add(v)
        t = pcbnew.PCB_TRACK(b)
        t.SetStart(pad.GetPosition())
        t.SetEnd(pos)
        t.SetWidth(mm(0.25))
        t.SetLayer(layer)
        t.SetNet(pad.GetNet())
        b.Add(t)
        ripped = fr.rip_up(b, "/GND", [v, t])
        ok = all(fr.route_net(b, n, fr.items_by_net(b)) for n in sorted(ripped))
        if not ok:
            b = pcbnew.LoadBoard(fr.PCB)
            pad = b.FindFootprintByReference(ref).FindPadByNumber(num)
            continue
        n1, c1, left = report(b)
        print(f"{ref}.{num}: via at {pcbnew.ToMM(pos)}, rerouted {sorted(ripped)}; open {n0}->{n1}, clearance {c0}->{c1}", flush=True)
        if n1 < n0 and c1 <= c0:
            pcbnew.SaveBoard(fr.PCB, b)
            return True
        b = pcbnew.LoadBoard(fr.PCB)
        pad = b.FindFootprintByReference(ref).FindPadByNumber(num)
    print(f"{ref}.{num}: nothing worked", flush=True)
    return False


if __name__ == "__main__":
    b = pcbnew.LoadBoard(fr.PCB)
    for ref, num in report(b)[2]:
        attempt(ref, num)
