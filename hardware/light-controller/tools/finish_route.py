"""Route the connections the autorouter left open, with a 2-layer grid maze router.

For every net DRC still reports as unconnected (GND excluded: stitch.py handles it),
the board is rasterised at 0.05 mm. Copper of other nets is grown by clearance plus
half the track width, so any free cell can carry the track centre line. A Dijkstra
search runs from one connected piece of the net to the nearest other piece, changing
layer through a via where a via fits. The path becomes tracks and vias, and the
search repeats until the net is whole.
"""
import heapq
import math
import os
import re
import sys

import numpy as np
import pcbnew
from PIL import Image, ImageDraw
from scipy import ndimage

sys.path.insert(0, os.path.dirname(__file__))
from gen_pcb import PCB  # noqa: E402

RES = 0.025                        # mm per cell
HX, HY = 73.0 / 2, 35.0 / 2
W, H = int(73.0 / RES), int(35.0 / RES)
CLR = 0.152                        # 0.15 rule + a hair for the grid
VIA_D, VIA_DRILL = 0.6, 0.3
ANT_X = -29.3
LAYERS = (pcbnew.F_Cu, pcbnew.B_Cu)
WIDTH = {"Power": 0.5, "Supply": 0.2}
nm = pcbnew.FromMM
HIST = [np.zeros((H, W), np.float32), np.zeros((H, W), np.float32)]   # rip-up history cost


def width_for(board, net):
    nc = board.FindNet(net).GetNetClassName()
    return WIDTH.get(nc, 0.15)


def raster(items, layer, grow):
    """Rasterise (item, layer) copper grown by `grow` mm into a bool array."""
    im = Image.new("1", (W, H), 0)
    d = ImageDraw.Draw(im)
    for it in items:
        if not it.IsOnLayer(layer):
            continue
        cls = it.GetClass()
        if cls in ("PCB_TRACK", "PCB_VIA"):
            r = (pcbnew.ToMM(it.GetWidth()) / 2 + grow + 0.01) / RES
            a, b = (it.GetPosition(), it.GetPosition()) if cls == "PCB_VIA" else (it.GetStart(), it.GetEnd())
            ax, ay = (pcbnew.ToMM(a.x) + HX) / RES, (pcbnew.ToMM(a.y) + HY) / RES
            bx, by_ = (pcbnew.ToMM(b.x) + HX) / RES, (pcbnew.ToMM(b.y) + HY) / RES
            d.ellipse((ax - r, ay - r, ax + r, ay + r), fill=1)
            d.ellipse((bx - r, by_ - r, bx + r, by_ + r), fill=1)
            if cls == "PCB_TRACK":
                d.line((ax, ay, bx, by_), fill=1, width=max(1, int(round(2 * r))))
            continue
        ps = pcbnew.SHAPE_POLY_SET()
        it.TransformShapeToPolygon(ps, layer, nm(grow), nm(0.005))
        for k in range(ps.OutlineCount()):
            ol = ps.Outline(k)
            pts = [((pcbnew.ToMM(ol.CPoint(i).x) + HX) / RES, (pcbnew.ToMM(ol.CPoint(i).y) + HY) / RES)
                   for i in range(ol.PointCount())]
            if len(pts) > 2:
                d.polygon(pts, fill=1)
    return np.array(im, dtype=bool)


def items_by_net(board):
    by = {}
    for fp in board.GetFootprints():
        for p in fp.Pads():
            by.setdefault(p.GetNetname(), []).append(p)
    for t in board.GetTracks():
        by.setdefault(t.GetNetname(), []).append(t)
    return by


def unconnected_nets(board):
    rpt = "/tmp/lc_fin.rpt"
    pcbnew.WriteDRCReport(board, rpt, pcbnew.EDA_UNITS_MILLIMETRES, False)
    nets = []
    for blk in re.split(r"\n(?=\[)", open(rpt).read()):
        if blk.startswith("[unconnected_items]"):
            for n in re.findall(r"\[(/[^\]]+)\]", blk):
                if n != "/GND" and n not in nets:
                    nets.append(n)
    return nets


def edge_mask(grow):
    m = np.zeros((H, W), dtype=bool)
    g = int(math.ceil(grow / RES))
    m[:g, :] = m[-g:, :] = True
    m[:, :g] = m[:, -g:] = True
    m[:, :int((ANT_X + HX) / RES)] = True
    return m


def route_net(board, net, by, rip=False):
    """Connect `net`. With rip=True other nets' tracks cost extra instead of blocking;
    returns the set of nets whose tracks the new path pushed out (they are removed)."""
    w = width_for(board, net)
    grow = CLR + w / 2
    others = [it for n, its in by.items() if n != net for it in its]
    pads = [it for it in others if it.GetClass() == "PAD"]
    tracks = [it for it in others if it.GetClass() != "PAD"]
    hard = [raster(pads, ly, grow) | edge_mask(0.3 + w / 2) for ly in LAYERS]
    soft = [raster(tracks, ly, grow) for ly in LAYERS]
    block = hard if rip else [hard[i] | soft[i] for i in range(2)]
    vgrow = CLR + VIA_D / 2
    vblock = raster(others, LAYERS[0], vgrow) | raster(others, LAYERS[1], vgrow) | edge_mask(0.3 + VIA_D / 2)
    # hole-to-hole: keep 0.5 mm between drill edges for every drilled item
    holes = Image.new("1", (W, H), 0)
    dh = ImageDraw.Draw(holes)
    for its in by.values():
        for it in its:
            if it.GetClass() == "PCB_VIA" or (it.GetClass() == "PAD" and it.HasHole()):
                c = it.GetPosition()
                r = (0.5 + VIA_DRILL / 2 + pcbnew.ToMM(it.GetDrillValue() if it.GetClass() == "PCB_VIA"
                                                       else it.GetDrillSize().x) / 2) / RES
                x, y = (pcbnew.ToMM(c.x) + HX) / RES, (pcbnew.ToMM(c.y) + HY) / RES
                dh.ellipse((x - r, y - r, x + r, y + r), fill=1)
    vblock |= np.array(holes, dtype=bool)

    for _ in range(20):
        own = items_by_net(board).get(net, [])
        cu = [raster(own, ly, 0.0) for ly in LAYERS]
        lab = [ndimage.label(cu[i], structure=np.ones((3, 3)))[0] for i in range(2)]
        # union components across layers where a via or through-hole pad joins them
        parent = {}

        def find(a):
            while parent.get(a, a) != a:
                a = parent[a]
            return a
        n0 = lab[0].max()
        for it in own:
            if it.GetClass() == "PCB_VIA" or (it.GetClass() == "PAD" and it.HasHole()):
                c = it.GetPosition()
                x, y = int((pcbnew.ToMM(c.x) + HX) / RES), int((pcbnew.ToMM(c.y) + HY) / RES)
                a, b = lab[0][y, x], lab[1][y, x]
                if a and b:
                    parent[find(a)] = find(b + n0)
        comp = [np.where(lab[0] > 0, np.vectorize(lambda v: find(v))(lab[0]) if lab[0].max() else 0, 0),
                np.where(lab[1] > 0, np.vectorize(lambda v: find(v + n0))(lab[1]) if lab[1].max() else 0, 0)]
        ids = set(np.unique(comp[0])) | set(np.unique(comp[1]))
        ids.discard(0)
        if len(ids) <= 1:
            return True
        # source: the smallest piece; targets: everything else of this net
        sizes = {i: int((comp[0] == i).sum() + (comp[1] == i).sum()) for i in ids}
        src = min(sizes, key=sizes.get)
        path = search(comp, src, block, vblock, soft if rip else None)
        if path is None:
            print(f"  {net}: no path")
            return False if not rip else None
        if rip:
            # contested cells get dearer each time, so swapping pairs look for another way
            for ly, y, x in path:
                if soft[ly][y, x]:
                    HIST[ly][max(0, y - 3):y + 4, max(0, x - 3):x + 4] += 15
        new = add_path(board, net, path, w)
        if rip:
            return rip_up(board, net, new)
    return False


def rip_up(board, net, new):
    """Remove other nets' tracks and vias that the new items crowd; return their nets."""
    hit = set()
    for t in list(board.GetTracks()):
        if t.GetNetname() in (net, ""):
            continue
        for n in new:
            ly = n.GetLayer() if n.GetClass() == "PCB_TRACK" else None
            for L in ([ly] if ly is not None else list(LAYERS)):
                if not t.IsOnLayer(L):
                    continue
                if t.GetEffectiveShape(L).Collide(n.GetEffectiveShape(L), nm(0.15)):
                    hit.add(t)
                    break
    nets = set()
    for t in hit:
        nets.add(t.GetNetname())
        board.Remove(t)
    return nets


def prune_dangling(board):
    """Drop track ends that touch nothing of their net (left behind by rip-up)."""
    while True:
        removed = 0
        tracks = [t for t in board.GetTracks() if t.GetClass() == "PCB_TRACK"]
        for t in tracks:
            for end in (t.GetStart(), t.GetEnd()):
                if not touches(board, t, end):
                    board.Remove(t)
                    removed += 1
                    break
        if not removed:
            return


def touches(board, t, end):
    net = t.GetNetCode()
    for o in board.GetTracks():
        if o is t or o.GetNetCode() != net:
            continue
        if o.GetClass() == "PCB_VIA":
            if (o.GetPosition() - end).EuclideanNorm() <= o.GetWidth() // 2:
                return True
        elif o.IsOnLayer(t.GetLayer()) and o.GetEffectiveShape(t.GetLayer()).Collide(pcbnew.SHAPE_CIRCLE(end, 1), 0):
            return True
    for fp in board.GetFootprints():
        for p in fp.Pads():
            if p.GetNetCode() == net and p.IsOnLayer(t.GetLayer()) and \
                    p.GetEffectiveShape(t.GetLayer()).Collide(pcbnew.SHAPE_CIRCLE(end, 1), 0):
                return True
    for z in board.Zones():
        if z.GetNetCode() == net and z.IsOnLayer(t.GetLayer()):
            return True
    return False


def search(comp, src, block, vblock, soft=None):
    start = [(0, ly, y, x) for ly in (0, 1) for y, x in zip(*np.nonzero(comp[ly] == src))]
    target = [comp[0] > 0, comp[1] > 0]
    target[0] &= comp[0] != src
    target[1] &= comp[1] != src
    tpts = np.argwhere(target[0] | target[1])
    if len(tpts) > 60:
        tpts = tpts[np.linspace(0, len(tpts) - 1, 60).astype(int)]

    def h(y, x):
        return float(np.min(np.hypot(tpts[:, 0] - y, tpts[:, 1] - x)))
    dist = {}
    prev = {}
    pq = []
    for _, ly, y, x in start:
        dist[(ly, y, x)] = 0.0
        heapq.heappush(pq, (h(y, x), 0.0, ly, y, x))
    VIA_COST = 30.0
    RIP_COST = 25.0
    steps = [(-1, 0, 1), (1, 0, 1), (0, -1, 1), (0, 1, 1), (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    n = 0
    while pq:
        f, g, ly, y, x = heapq.heappop(pq)
        if g > dist.get((ly, y, x), 1e18):
            continue
        n += 1
        if n > (800_000 if soft is not None else 3_000_000):
            return None
        if target[ly][y, x]:
            path = [(ly, y, x)]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            return path[::-1]
        own_here = comp[ly][y, x] == src
        for dy, dx, c in steps:
            yy, xx = y + dy, x + dx
            if not (0 <= yy < H and 0 <= xx < W):
                continue
            if block[ly][yy, xx] and not (comp[ly][yy, xx] > 0):
                continue
            ng = g + c + (RIP_COST + HIST[ly][yy, xx] if soft is not None and soft[ly][yy, xx] else 0)
            k = (ly, yy, xx)
            if ng < dist.get(k, 1e18):
                dist[k] = ng
                prev[k] = (ly, y, x)
                heapq.heappush(pq, (ng + h(yy, xx), ng, ly, yy, xx))
        ol = 1 - ly
        if not vblock[y, x] and not block[ol][y, x] and not (soft is not None and (soft[0][y, x] or soft[1][y, x])):
            ng = g + VIA_COST
            k = (ol, y, x)
            if ng < dist.get(k, 1e18):
                dist[k] = ng
                prev[k] = (ly, y, x)
                heapq.heappush(pq, (ng + h(y, x), ng, ol, y, x))
    return None


def add_path(board, net, path, w):
    ni = board.FindNet(net)

    def pt(y, x):
        return pcbnew.VECTOR2I(nm(float(x * RES - HX + RES / 2)), nm(float(y * RES - HY + RES / 2)))
    added = []
    # split by layer, simplify straight runs
    runs, cur = [], [path[0]]
    for p in path[1:]:
        if p[0] != cur[-1][0]:
            runs.append(cur)
            v = pcbnew.PCB_VIA(board)
            v.SetPosition(pt(p[1], p[2]))
            v.SetWidth(nm(VIA_D))
            v.SetDrill(nm(VIA_DRILL))
            v.SetNet(ni)
            board.Add(v)
            added.append(v)
            cur = [p]
        else:
            cur.append(p)
    runs.append(cur)
    for run in runs:
        if len(run) < 2:
            continue
        pts = [run[0]]
        for a, b, c in zip(run, run[1:], run[2:]):
            if (b[1] - a[1], b[2] - a[2]) != (c[1] - b[1], c[2] - b[2]):
                pts.append(b)
        pts.append(run[-1])
        for a, b in zip(pts, pts[1:]):
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pt(a[1], a[2]))
            t.SetEnd(pt(b[1], b[2]))
            t.SetWidth(nm(w))
            t.SetLayer(LAYERS[a[0]])
            t.SetNet(ni)
            board.Add(t)
            added.append(t)
    return added


def main():
    board = pcbnew.LoadBoard(PCB)
    for z in board.Zones():
        z.UnFill()
    nets = [a for a in sys.argv[1:] if a.startswith("/")] or unconnected_nets(board)
    print("open nets:", " ".join(nets))
    by = items_by_net(board)
    for net in nets:
        if board.FindNet(net) is None:
            continue
        by = items_by_net(board)
        ok = route_net(board, net, by)
        print(f"{net}: {'routed' if ok else 'FAILED'}", flush=True)
    # careful rip-up: push other tracks aside only if every displaced net reroutes
    if "--try" in sys.argv:
        import time
        for net in unconnected_nets(board):
            t0 = time.time()
            pcbnew.SaveBoard("/tmp/fr_snap.kicad_pcb", board)
            before = len(unconnected_nets(board))
            ripped = route_net(board, net, items_by_net(board), rip=True)
            ok = ripped is not None
            todo, depth = sorted(ripped or []), 0
            while ok and todo and depth < 6:
                nxt = []
                for v in todo:
                    if v == "/GND" or route_net(board, v, items_by_net(board)):
                        continue
                    r2 = route_net(board, v, items_by_net(board), rip=True)
                    if r2 is None:
                        ok = False
                        break
                    nxt += [x for x in r2 if x not in nxt]
                todo, depth = nxt, depth + 1
            ok = ok and not todo
            after = len(unconnected_nets(board)) if ok else before
            if not ok or after >= before:
                board = pcbnew.LoadBoard("/tmp/fr_snap.kicad_pcb")
                print(f"{net}: kept as is ({time.time() - t0:.0f}s)", flush=True)
            else:
                print(f"{net}: routed, rerouted {sorted(ripped)} ({time.time() - t0:.0f}s)", flush=True)
    # rip-up and reroute: whatever is still open may push other nets' tracks aside
    if "--rip" in sys.argv:
        queue = unconnected_nets(board)
        tries = {}
        while queue and sum(tries.values()) < 400:
            net = queue.pop(0)
            tries[net] = tries.get(net, 0) + 1
            if route_net(board, net, items_by_net(board)):
                continue
            ripped = route_net(board, net, items_by_net(board), rip=True)
            if ripped is None:
                print(f"{net}: unroutable even with rip-up")
                continue
            print(f"{net}: rip-up pass, displaced {sorted(ripped)}", flush=True)
            for n in sorted(ripped) + [net]:
                if n != "/GND" and n not in queue:
                    queue.append(n)
        prune_dangling(board)
        print("still open:", unconnected_nets(board))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    pcbnew.SaveBoard(PCB, board)


if __name__ == "__main__":
    main()
