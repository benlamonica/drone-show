import pcbnew
b=pcbnew.LoadBoard('/w/rtk-gps/rtk_gps.kicad_pcb')
for z in b.Zones():
    fl=z.GetFilledPolysList(z.GetLayer())
    g=[t.GetPosition() for t in b.GetTracks() if t.GetClass()=="PCB_VIA" and t.GetNetname()=="/GND"]
    for i in range(fl.OutlineCount()):
        ol=fl.Outline(i); bb=ol.BBox()
        print(z.GetZoneName(), i, round(pcbnew.ToMM(bb.GetLeft()),1),round(pcbnew.ToMM(bb.GetTop()),1),round(pcbnew.ToMM(bb.GetRight()),1),round(pcbnew.ToMM(bb.GetBottom()),1), sum(ol.PointInside(h) for h in g), fl.HoleCount(i))
