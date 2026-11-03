"""Move one via (and the track ends on it) by a small offset: python3 tools/nudge.py X Y DX DY"""
import os
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from gen_pcb import PCB  # noqa: E402

mm = pcbnew.FromMM
x, y, dx, dy = map(float, sys.argv[1:5])
b = pcbnew.LoadBoard(PCB)
at = pcbnew.VECTOR2I(mm(x), mm(y))
v = min((t for t in b.GetTracks() if t.GetClass() == "PCB_VIA"), key=lambda t: (t.GetPosition() - at).EuclideanNorm())
old = v.GetPosition()
new = pcbnew.VECTOR2I(old.x + mm(dx), old.y + mm(dy))
for t in b.GetTracks():
    if t.GetClass() == "PCB_TRACK" and t.GetNetCode() == v.GetNetCode():
        if (t.GetStart() - old).EuclideanNorm() < mm(0.01):
            t.SetStart(new)
        if (t.GetEnd() - old).EuclideanNorm() < mm(0.01):
            t.SetEnd(new)
v.SetPosition(new)
pcbnew.ZONE_FILLER(b).Fill(b.Zones())
pcbnew.SaveBoard(PCB, b)
print("moved", v.GetNetname(), "via to", pcbnew.ToMM(new))
