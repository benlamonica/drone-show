"""Import the freerouting session, add the top GND pour, fill planes, save."""
import sys, pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
# the RF feed is hand-placed: drop whatever the router left on RF_IN and lay it again
def pad(ref, num):
    return b.FindFootprintByReference(ref).FindPadByNumber(num)
RFPOS = None
def track(a, c, w, net):
    t = pcbnew.PCB_TRACK(b); t.SetStart(a); t.SetEnd(c); t.SetWidth(pcbnew.FromMM(w))
    t.SetLayer(pcbnew.F_Cu); t.SetNet(net); t.SetLocked(True); b.Add(t)
rf, ufl, l2 = pad("U3", "11"), pad("J4", "1"), pad("L2", "2")
p1, p2, p3 = pcbnew.VECTOR2I(rf.GetPosition()), pcbnew.VECTOR2I(ufl.GetPosition()), pcbnew.VECTOR2I(l2.GetPosition())
rfnet = rf.GetNetCode()
rfn = b.FindNet(rfnet)
k1, k2 = pcbnew.VECTOR2I(p1.x, p3.y), pcbnew.VECTOR2I(p1.x, p2.y)
track(p1, k1, 0.6, rfn); track(k1, k2, 0.6, rfn); track(k2, p2, 0.6, rfn); track(p3, k1, 0.4, rfn)
bot = [z for z in b.Zones() if z.GetLayer() == pcbnew.B_Cu and not z.GetIsRuleArea()][0]
top = pcbnew.ZONE(bot)
top.SetLayer(pcbnew.F_Cu)
top.SetZoneName("GND_top")
b.Add(top)
f = pcbnew.ZONE_FILLER(b); f.Fill(b.Zones())

# stitch the two GND pours together on a 2.5 mm grid wherever both are solid around the via
gnd = top.GetNet()
fills = [top.GetFilledPolysList(pcbnew.F_Cu), bot.GetFilledPolysList(pcbnew.B_Cu)]
holes = [t.GetPosition() for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]
holes += [p.GetPosition() for fp in b.GetFootprints() for p in fp.Pads() if p.HasHole()]
import math
def solid(x, y, r):
    pts = [(x, y)] + [(x + r * math.cos(a), y + r * math.sin(a)) for a in [i * math.pi / 4 for i in range(8)]]
    return all(fl.Contains(pcbnew.VECTOR2I(int(px), int(py))) for fl in fills for px, py in pts)
n = 0
step = pcbnew.FromMM(2.5)
bb = b.GetBoardEdgesBoundingBox()
y = bb.GetTop() + pcbnew.FromMM(1.5)
while y < bb.GetBottom():
    x = bb.GetLeft() + pcbnew.FromMM(1.5)
    while x < bb.GetRight():
        if solid(x, y, pcbnew.FromMM(0.6)) and all(math.hypot(h.x - x, h.y - y) > pcbnew.FromMM(1.0) for h in holes):
            v = pcbnew.PCB_VIA(b); v.SetPosition(pcbnew.VECTOR2I(int(x), int(y)))
            v.SetWidth(pcbnew.FromMM(0.65)); v.SetDrill(pcbnew.FromMM(0.3)); v.SetNet(gnd); b.Add(v)
            holes.append(pcbnew.VECTOR2I(int(x), int(y))); n += 1
        x += step
    y += step
# tie every pour island into the main ground: islands are linked through GND vias, and any island
# not reachable from the main bottom pour gets a via down/up into a pour that is
def area(ol):
    bb = ol.BBox(); return bb.GetWidth() * bb.GetHeight()
def via_at(pt):
    v = pcbnew.PCB_VIA(b); v.SetPosition(pt)
    v.SetWidth(pcbnew.FromMM(0.65)); v.SetDrill(pcbnew.FromMM(0.3)); v.SetNet(gnd); b.Add(v)
    holes.append(pt)
def inside(ol, x, y, r):
    pts = [(x, y)] + [(x + r * math.cos(a), y + r * math.sin(a)) for a in [i * math.pi / 4 for i in range(8)]]
    return all(ol.PointInside(pcbnew.VECTOR2I(int(px), int(py))) for px, py in pts)
for _ in range(6):
    f.Fill(b.Zones())
    T = [top.GetFilledPolysList(pcbnew.F_Cu).Outline(i) for i in range(top.GetFilledPolysList(pcbnew.F_Cu).OutlineCount())]
    B = [bot.GetFilledPolysList(pcbnew.B_Cu).Outline(i) for i in range(bot.GetFilledPolysList(pcbnew.B_Cu).OutlineCount())]
    nodes = [("T", i) for i in range(len(T))] + [("B", j) for j in range(len(B))]
    adj = {k: set() for k in nodes}
    links = [t.GetPosition() for t in b.GetTracks() if t.GetClass() == "PCB_VIA" and t.GetNetname() == gnd.GetNetname()]
    links += [p.GetPosition() for fp in b.GetFootprints() for p in fp.Pads() if p.HasHole() and p.GetNetname() == gnd.GetNetname()]
    for pt in links:
        ti = [("T", i) for i, ol in enumerate(T) if ol.PointInside(pt)]
        bj = [("B", j) for j, ol in enumerate(B) if ol.PointInside(pt)]
        for x in ti:
            for y in bj:
                adj[x].add(y); adj[y].add(x)
    start = ("B", max(range(len(B)), key=lambda j: area(B[j])))
    seen, todo = {start}, [start]
    while todo:
        k = todo.pop()
        for m in adj[k]:
            if m not in seen:
                seen.add(m); todo.append(m)
    lost = [k for k in nodes if k not in seen]
    if not lost:
        break
    added = 0
    for lay, i in lost:
        ol = (T if lay == "T" else B)[i]
        others = [(B if lay == "T" else T)[k[1]] for k in seen if k[0] != lay]
        obb = ol.BBox(); done = False
        yy = obb.GetTop()
        while yy < obb.GetBottom() and not done:
            xx = obb.GetLeft()
            while xx < obb.GetRight() and not done:
                if inside(ol, xx, yy, pcbnew.FromMM(0.34)) and any(inside(o2, xx, yy, pcbnew.FromMM(0.34)) for o2 in others) and \
                   all(math.hypot(h.x - xx, h.y - yy) > pcbnew.FromMM(0.95) for h in holes):
                    via_at(pcbnew.VECTOR2I(int(xx), int(yy))); n += 1; added += 1; done = True
                xx += pcbnew.FromMM(0.1)
            yy += pcbnew.FromMM(0.1)
    print("islands not tied:", len(lost), "vias added:", added)
    if not added:
        break
print("stitching vias", n)
f.Fill(b.Zones())
b.Save(sys.argv[1])
print("tracks", len([t for t in b.GetTracks() if t.GetClass() == "PCB_TRACK"]), "vias", len([t for t in b.GetTracks() if t.GetClass() == "PCB_VIA"]))
