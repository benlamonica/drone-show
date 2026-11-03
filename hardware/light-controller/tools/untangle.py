"""Free a pin boxed in by its neighbours: take up the routes of the nets on the
adjacent MCU pins, route the boxed net first, then route the neighbours again.
Keeps the result only if fewer nets are left open.

    python3 tools/untangle.py /NET [/NET ...]
"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import finish_route as fr  # noqa: E402

SKIP = {"/GND", "/+3V3", "/+1V1", ""}


def neighbours(board, net, reach):
    u2 = board.FindFootprintByReference("U2")
    nums = [int(p.GetNumber()) for p in u2.Pads() if p.GetNetname() == net and p.GetNumber().isdigit()]
    out = []
    for n in nums:
        for k in range(1, reach + 1):
            for m in (n - k, n + k):
                p = u2.FindPadByNumber(str(m))
                if p and p.GetNetname() not in SKIP and p.GetNetname() != net and p.GetNetname() not in out:
                    out.append(p.GetNetname())
    return out


def attempt(net, reach):
    b = pcbnew.LoadBoard(fr.PCB)
    for z in b.Zones():
        z.UnFill()
    before = fr.unconnected_nets(b)
    nb = neighbours(b, net, reach)
    for t in [t for t in b.GetTracks() if t.GetNetname() in nb]:
        b.Delete(t)
    order = [net] + nb
    for n in order:
        fr.route_net(b, n, fr.items_by_net(b))
    after = fr.unconnected_nets(b)
    print(f"{net} reach {reach}: neighbours {nb}; open {len(before)} -> {len(after)}", flush=True)
    if len(after) < len(before):
        pcbnew.ZONE_FILLER(b).Fill(b.Zones())
        pcbnew.SaveBoard(fr.PCB, b)
        return True
    return False


if __name__ == "__main__":
    for net in sys.argv[1:]:
        for reach in (1, 2, 3):
            if attempt(net, reach):
                break
