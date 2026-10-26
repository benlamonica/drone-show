"""Run KiCad DRC (pcbnew API, KiCad 7) and summarise the report."""
import collections
import os
import re
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
PCB = os.path.normpath(os.path.join(HERE, "..", "light-controller.kicad_pcb"))


def main(fill=True):
    b = pcbnew.LoadBoard(PCB)
    if fill:
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    rpt = "/tmp/lc_drc.rpt"
    pcbnew.WriteDRCReport(b, rpt, pcbnew.EDA_UNITS_MILLIMETRES, True)
    t = open(rpt).read()
    kinds = collections.Counter(re.findall(r"^\[(\w+)\]", t, re.M))
    print(dict(kinds))
    return t


if __name__ == "__main__":
    t = main("--nofill" not in sys.argv)
    pat = sys.argv[1] if len(sys.argv) > 1 and not sys.argv[1].startswith("--") else None
    blocks = re.split(r"\n(?=\[)", t)
    for blk in blocks:
        if blk.startswith("[") and (pat is None or re.search(pat, blk)):
            print(blk.strip()[:400])
