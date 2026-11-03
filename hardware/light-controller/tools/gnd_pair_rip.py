"""Connect a stray ground cluster to the main ground: a pad of the cluster and a
nearby main-ground via go on a temporary net, the grid router joins them pushing
signal tracks aside, the displaced nets are rerouted, and the change is kept only
when DRC shows fewer unconnected items and no new clearance errors.

    python3 tools/gnd_pair_rip.py REF.PAD [REF.PAD ...]
"""
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import finish_route as fr  # noqa: E402
import gnd_join as gj  # noqa: E402

mm = pcbnew.FromMM


def report(b):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.WriteDRCReport(b, "/tmp/lc_gpr.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    t = open("/tmp/lc_gpr.rpt").read()
    return (len(re.findall(r"^\[unconnected_items\]", t, re.M)),
            len(re.findall(r"^\[(clearance|hole_clearance|hole_near_hole|short_item|tracks_crossing)\]", t, re.M)))


def attempt(ref, num):
    b0 = pcbnew.LoadBoard(fr.PCB)
    n0, c0 = report(b0)
    comp, main_id, _ = gj.clusters(b0)
    pad = b0.FindFootprintByReference(ref).FindPadByNumber(num)
    c = pad.GetPosition()
    vias = []
    for v in b0.GetTracks():
        if v.GetClass() == "PCB_VIA" and v.GetNetname() == "/GND":
            p = v.GetPosition()
            x, y = int((pcbnew.ToMM(p.x) + fr.HX) / fr.RES), int((pcbnew.ToMM(p.y) + fr.HY) / fr.RES)
            if comp[0][y, x] == main_id or comp[1][y, x] == main_id:
                vias.append(p)
    vias.sort(key=lambda p: (p - c).EuclideanNorm())
    for target in vias[:6]:
        b = pcbnew.LoadBoard(fr.PCB)
        for z in b.Zones():
            z.UnFill()
        tmp = pcbnew.NETINFO_ITEM(b, "/GND_PAIR")
        b.Add(tmp)
        gnd = b.FindNet("/GND")
        p = b.FindFootprintByReference(ref).FindPadByNumber(num)
        v = next(t for t in b.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetPosition() == target)
        p.SetNet(tmp)
        v.SetNet(tmp)
        ripped = fr.route_net(b, "/GND_PAIR", fr.items_by_net(b), rip=True)
        for it in [p, v] + [t for t in b.GetTracks() if t.GetNetname() == "/GND_PAIR"]:
            it.SetNet(gnd)
        if ripped is None:
            continue
        if ripped is True:
            ripped = set()
        ok = True
        for n in sorted(ripped - {"/GND"}):
            if not fr.route_net(b, n, fr.items_by_net(b)):
                ok = False
                break
        if not ok:
            continue
        n1, c1 = report(b)
        print(f"{ref}.{num} -> via {pcbnew.ToMM(target)}: rerouted {sorted(ripped)}; open {n0}->{n1}, clearance {c0}->{c1}", flush=True)
        if n1 < n0 and c1 <= c0 + int(os.environ.get("CLR_SLACK", "0")):
            pcbnew.SaveBoard(fr.PCB, b)
            return True
    print(f"{ref}.{num}: no luck", flush=True)
    return False


if __name__ == "__main__":
    for a in sys.argv[1:]:
        attempt(*a.split("."))
