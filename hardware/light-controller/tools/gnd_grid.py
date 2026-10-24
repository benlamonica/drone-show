"""Ground stitching: a via every 2 mm wherever one fits clear of every other net, so
the top and bottom pours tie together and pour islands between tracks get a path to
ground. Islands that still touch nothing are then dropped from the fill.
"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import escape as es  # noqa: E402
from gen_pcb import PCB  # noqa: E402

mm = pcbnew.FromMM
PITCH = 2.0
ANT_X = -29.0          # antenna keep-out edge


def main():
    b = pcbnew.LoadBoard(PCB)
    gnd = b.FindNet("/GND")
    for z in b.Zones():
        if not z.GetIsRuleArea():
            z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_NEVER)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    fills = {L: pcbnew.SHAPE_POLY_SET() for L in (pcbnew.F_Cu, pcbnew.B_Cu)}
    for z in b.Zones():
        if z.GetNetname() == "/GND":
            for L in fills:
                if z.IsOnLayer(L):
                    fills[L].Append(z.GetFilledPolysList(L))
    n = 0
    x = -36.5 + 1.0
    while x < 36.5 - 0.8:
        y = -17.5 + 2.6
        while y < 17.5 - 2.5:
            pos = pcbnew.VECTOR2I(mm(x), mm(y))
            # inside both pours with room for the via, clear of other nets and holes
            if x > ANT_X + 0.6 and all(fills[L].Contains(pos, -1, mm(0.0)) and
                                       fills[L].Collide(pos, 0) for L in fills):
                circ = pcbnew.SHAPE_CIRCLE(pos, mm(0.3))
                if not es.collides(b, "/GND", [(None, circ)]) and es.holes_ok(b, pos):
                    v = pcbnew.PCB_VIA(b)
                    v.SetPosition(pos)
                    v.SetWidth(mm(0.6))
                    v.SetDrill(mm(0.3))
                    v.SetNet(gnd)
                    b.Add(v)
                    n += 1
            y += PITCH / 2
        x += PITCH / 2
    for z in b.Zones():
        if not z.GetIsRuleArea():
            z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)
    print("stitching vias added:", n)


if __name__ == "__main__":
    main()
