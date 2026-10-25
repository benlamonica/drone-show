"""Pre-route the RF feed by hand (short, wide, straight) and export the Specctra DSN for freerouting."""
import sys, pcbnew
pcb = sys.argv[1]
b = pcbnew.LoadBoard(pcb)
fp = {f.GetReference(): f for f in b.GetFootprints()}
def pad(ref, num):
    return [p for p in fp[ref].Pads() if p.GetNumber() == num][0]
def track(a, c, w, net):
    t = pcbnew.PCB_TRACK(b); t.SetStart(a); t.SetEnd(c); t.SetWidth(pcbnew.FromMM(w))
    t.SetLayer(pcbnew.F_Cu); t.SetNet(net); t.SetLocked(True); b.Add(t)
rf = pad("U3", "11"); ufl = pad("J4", "1"); l2 = pad("L2", "2")
net = rf.GetNet()
p1, p2, p3 = rf.GetPosition(), ufl.GetPosition(), l2.GetPosition()
mid = pcbnew.VECTOR2I(p1.x, p2.y)
track(p1, pcbnew.VECTOR2I(p1.x, p3.y), 0.6, net)
track(pcbnew.VECTOR2I(p1.x, p3.y), mid, 0.6, net)
track(mid, p2, 0.6, net)
track(p3, pcbnew.VECTOR2I(p1.x, p3.y), 0.4, net)
print("RF pads", [(pcbnew.ToMM(p.x), pcbnew.ToMM(p.y)) for p in (p1, p2, p3)])
# RP2350 exposed pad: four 0.3 mm vias straight into the bottom plane (PCBWay standard drill)
ep = pad("U1", "61")
for dx in (-0.85, 0.85):
    for dy in (-0.85, 0.85):
        v = pcbnew.PCB_VIA(b); v.SetPosition(ep.GetPosition() + pcbnew.VECTOR2I(pcbnew.FromMM(dx), pcbnew.FromMM(dy)))
        v.SetWidth(pcbnew.FromMM(0.6)); v.SetDrill(pcbnew.FromMM(0.3)); v.SetNet(ep.GetNet()); v.SetLocked(True); b.Add(v)
# keep bottom-layer tracks out from under the RP2350 so the plane under it stays whole
c = fp["U1"].GetPosition()
ka = pcbnew.ZONE(b); ka.SetIsRuleArea(True); ka.SetLayer(pcbnew.B_Cu)
ka.SetDoNotAllowTracks(True); ka.SetDoNotAllowVias(False); ka.SetDoNotAllowPads(False)
ka.SetDoNotAllowFootprints(False); ka.SetZoneName("U1_bottom_plane_keepout")
o = ka.Outline(); o.NewOutline()
h = pcbnew.FromMM(4.8)
for dx, dy in ((-h, -h), (h, -h), (h, h), (-h, h)):
    o.Append(c.x + dx, c.y + dy)
b.Add(ka)
ns = b.GetDesignSettings().m_NetSettings
print("Default clearance", pcbnew.ToMM(ns.GetDefaultNetclass().GetClearance()))
b.Save(pcb)
pcbnew.ExportSpecctraDSN(b, sys.argv[2])
# PCBWay wants 0.5 mm between drilled holes; with 0.6/0.3 vias that is 0.2 mm copper-to-copper between vias
d = open(sys.argv[2]).read()
d = d.replace("(clearance 50 (type smd_smd))", "(clearance 50 (type smd_smd))\n      (clearance 250 (type via_via))", 1)
open(sys.argv[2], "w").write(d)
