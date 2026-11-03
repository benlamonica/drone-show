"""Join ground pieces DRC still reports apart, pad to pad: the two pads go on a
temporary net so the grid router treats the pair as its own two-pin net (and the
rest of ground as an obstacle it must clear), then go back to GND."""
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import finish_route as fr  # noqa: E402


def pairs(board):
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.WriteDRCReport(board, "/tmp/lc_gp.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    out = []
    for blk in re.split(r"\n(?=\[)", open("/tmp/lc_gp.rpt").read()):
        if blk.startswith("[unconnected_items]"):
            p = re.findall(r"Pad (\S+) \[/GND\] of (\S+) on", blk)
            if len(p) == 2:
                out.append(tuple((ref, num) for num, ref in p))
    return out


def main():
    b = pcbnew.LoadBoard(fr.PCB)
    for (r1, n1), (r2, n2) in pairs(b):
        for z in b.Zones():
            z.UnFill()
        tmp = pcbnew.NETINFO_ITEM(b, "/GND_PAIR")
        b.Add(tmp)
        p1 = b.FindFootprintByReference(r1).FindPadByNumber(n1)
        p2 = b.FindFootprintByReference(r2).FindPadByNumber(n2)
        gnd = p1.GetNet()
        p1.SetNet(tmp)
        p2.SetNet(tmp)
        ok = fr.route_net(b, "/GND_PAIR", fr.items_by_net(b))
        for it in [p1, p2] + [t for t in b.GetTracks() if t.GetNetname() == "/GND_PAIR"]:
            it.SetNet(gnd)
        print(f"{r1}.{n1} - {r2}.{n2}: {'joined' if ok else 'no path'}", flush=True)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(fr.PCB, b)


if __name__ == "__main__":
    main()
