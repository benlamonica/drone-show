"""Delete the track stubs and vias DRC flags as dangling (left over from rip-up),
one at a time, keeping each deletion only if the unconnected count doesn't grow."""
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from gen_pcb import PCB  # noqa: E402


def drc(b):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.WriteDRCReport(b, "/tmp/lc_cd.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    return open("/tmp/lc_cd.rpt").read()


def dangling(t):
    hits = []
    for blk in re.split(r"\n(?=\[)", t):
        if blk.startswith(("[track_dangling]", "[via_dangling]")):
            m = re.search(r"@\((\S+) mm, (\S+) mm\): (Track|Via) \[(\S+)\]", blk)
            if m:
                hits.append((float(m.group(1)), float(m.group(2)), m.group(3), m.group(4)))
    return hits


def unconn(t):
    return len(re.findall(r"^\[unconnected_items\]", t, re.M))


def main():
    skipped = set()
    while True:
        b = pcbnew.LoadBoard(PCB)
        t = drc(b)
        n0 = unconn(t)
        todo = [h for h in dangling(t) if h not in skipped]
        if not todo:
            break
        x, y, kind, net = h = todo[0]
        for it in b.GetTracks():
            p = it.GetPosition()
            if it.GetNetname() == net and abs(pcbnew.ToMM(p.x) - x) < 0.001 and abs(pcbnew.ToMM(p.y) - y) < 0.001 \
                    and (kind == "Via") == (it.GetClass() == "PCB_VIA"):
                b.Delete(it)
                break
        n1 = unconn(drc(b))
        if n1 > n0:
            print(f"keep {h}: needed")
            skipped.add(h)
        else:
            print(f"removed {h}")
            pcbnew.SaveBoard(PCB, b)


if __name__ == "__main__":
    main()
