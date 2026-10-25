"""Finish the last few connections the autorouter leaves open.

A small two-layer grid router: it reads the unconnected pairs from the last DRC
report, rasterises every other-net pad, track, via, hole, keep-out and the board
edge (grown by the clearance a new track or via needs), and A*-searches a path
between the two copper islands. Ground pours are ignored here; they are
re-poured afterwards.

    python3 handroute.py            # routes what build/drc.rpt lists as unconnected
"""

import heapq
import math
import os
import re
import sys

import numpy as np
import pcbnew
from PIL import Image, ImageDraw

import make_board as mb
import route as rt

GRID = float(os.environ.get("GRID", "0.05"))   # mm per cell
# extra margin added to every obstacle for rasterising error (default one cell)
SLACK = float(os.environ.get("SLACK", GRID))
VIA_D, VIA_DRILL = 0.6, 0.3
CLEAR = 0.15
VIA_COST = 2 / GRID               # in cells (2 mm of track)
# ground is poured on both layers, so its tracks can be ripped up freely; other
# nets only when RIP_ANY is set
RIP_ANY = bool(os.environ.get("RIP_ANY"))
MM = pcbnew.FromMM


# signal width; SIGNAL_W=0.15 (the fab minimum) lets a track escape between
# 0.4 mm pitch QFN pins
SIGNAL_W = float(os.environ.get("SIGNAL_W", "0.2"))


def width_for(net):
    return 0.4 if net in ("VBAT", "+5V", "BUCK_SW") else SIGNAL_W


def clearance_for(net):
    return 0.2 if net in ("VBAT", "+5V", "BUCK_SW") else CLEAR


class Grid:
    def __init__(self, board):
        bb = board.GetBoardEdgesBoundingBox()
        self.x0, self.y0 = pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop())
        self.nx = int(pcbnew.ToMM(bb.GetWidth()) / GRID) + 1
        self.ny = int(pcbnew.ToMM(bb.GetHeight()) / GRID) + 1

    def cell(self, x, y):
        return int(round((x - self.x0) / GRID)), int(round((y - self.y0) / GRID))

    def pos(self, i, j):
        return self.x0 + i * GRID, self.y0 + j * GRID

    def image(self):
        return Image.new("1", (self.nx, self.ny), 0)

    def poly(self, draw, shape_poly, grow=0.0):
        for k in range(shape_poly.OutlineCount()):
            ol = shape_poly.Outline(k)
            pts = []
            for m in range(ol.PointCount()):
                p = ol.CPoint(m)
                i = (pcbnew.ToMM(p.x) - self.x0) / GRID
                j = (pcbnew.ToMM(p.y) - self.y0) / GRID
                pts.append((i, j))
            if len(pts) >= 3:
                draw.polygon(pts, fill=1)
                if grow > 0:
                    draw.line(pts + [pts[0]], fill=1, width=int(2 * grow / GRID) + 2, joint="curve")


def draw_item(grid, draw, item, layer, grow):
    """Paint an item's copper on a layer, grown by grow mm. Pads go through
    KiCad's polygon call; tracks and vias are painted as round-ended strokes and
    discs (KiCad 7 does not export the ERROR_LOC enum their polygon call needs)."""
    if isinstance(item, pcbnew.PAD):
        sp = pcbnew.SHAPE_POLY_SET()
        item.TransformShapeToPolygon(sp, layer, MM(grow + 0.01), MM(0.005))
        grid.poly(draw, sp)
        return
    r = (pcbnew.ToMM(item.GetWidth()) / 2 + grow) / GRID
    if item.GetClass() == "PCB_VIA":
        ends = [item.GetPosition()]
    else:
        ends = [item.GetStart(), item.GetEnd()]
    pts = [((pcbnew.ToMM(p.x) - grid.x0) / GRID, (pcbnew.ToMM(p.y) - grid.y0) / GRID) for p in ends]
    for i, j in pts:
        draw.ellipse((i - r, j - r, i + r, j + r), fill=1)
    if len(pts) == 2:
        (i0, j0), (i1, j1) = pts
        L = math.hypot(i1 - i0, j1 - j0)
        if L > 0:
            nx_, ny_ = -(j1 - j0) / L * r, (i1 - i0) / L * r
            draw.polygon([(i0 + nx_, j0 + ny_), (i1 + nx_, j1 + ny_), (i1 - nx_, j1 - ny_), (i0 - nx_, j0 - ny_)], fill=1)


def obstacle_maps(board, grid, net, grow_track, grow_via, rip=False):
    """Per layer: cells a track centre may not enter, and cells a via centre may not enter."""
    layers = (pcbnew.F_Cu, pcbnew.B_Cu)
    track = {l: grid.image() for l in layers}
    via = {l: grid.image() for l in layers}
    dt = {l: ImageDraw.Draw(track[l]) for l in layers}
    dv = {l: ImageDraw.Draw(via[l]) for l in layers}
    own = {l: grid.image() for l in layers}
    do = {l: ImageDraw.Draw(own[l]) for l in layers}
    # other-net tracks and vias are soft: the router may cross them at a cost and
    # rip them up, and their nets get re-routed on the next round
    strack = {l: grid.image() for l in layers}
    svia = {l: grid.image() for l in layers}
    dst = {l: ImageDraw.Draw(strack[l]) for l in layers}
    dsv = {l: ImageDraw.Draw(svia[l]) for l in layers}

    def add(item, l, other, soft=False):
        if other:
            draw_item(grid, (dst if soft else dt)[l], item, l, grow_track + clearance_extra(item))
            draw_item(grid, (dsv if soft else dv)[l], item, l, grow_via + clearance_extra(item))
        else:
            draw_item(grid, do[l], item, l, 0)

    def clearance_extra(item):
        n = item.GetNetname() if hasattr(item, "GetNetname") else ""
        return 0.05 if n in ("VBAT", "+5V", "BUCK_SW") else 0.0

    for f in board.GetFootprints():
        for p in f.Pads():
            other = p.GetNetname() != net
            for l in layers:
                if p.IsOnLayer(l):
                    add(p, l, other)
            if p.GetDrillSize().x > 0:          # holes block both layers for vias
                for l in layers:
                    sp = pcbnew.SHAPE_POLY_SET()
                    p.TransformHoleToPolygon(sp, MM(grow_via + 0.25), MM(0.005))
                    grid.poly(dv[l], sp)
    for t in board.GetTracks():
        other = t.GetNetname() != net
        soft = rip and not t.IsLocked() and (RIP_ANY or t.GetNetname() == "GND")
        for l in layers:
            if t.IsOnLayer(l):
                add(t, l, other, soft)
    # keep-out rule areas
    for z in [board.GetArea(i) for i in range(board.GetAreaCount())]:
        if not z.GetIsRuleArea():
            continue
        sp = z.Outline()
        for l in layers:
            if z.IsOnLayer(l):
                if z.GetDoNotAllowTracks():
                    grid.poly(dt[l], sp, grow_track)
                if z.GetDoNotAllowVias():
                    grid.poly(dv[l], sp, grow_via)
    arr = {}
    for l in layers:
        t = np.array(track[l], dtype=bool)
        v = np.array(via[l], dtype=bool)
        arr[l] = (t, v, np.array(own[l], dtype=bool),
                  np.array(strack[l], dtype=bool), np.array(svia[l], dtype=bool))
    # board edge: track centre keeps 0.3 mm + half width from the outline
    ii, jj = np.meshgrid(np.arange(grid.nx), np.arange(grid.ny))
    xs = grid.x0 + ii * GRID - mb.CX
    ys = grid.y0 + jj * GRID - mb.CY
    def outside(m):
        return (np.abs(xs) > mb.HALF - m) | (np.abs(ys) > mb.HALF - m) | \
               (np.abs(xs) + np.abs(ys) > mb.CUT - m * math.sqrt(2))
    edge_t = outside(0.3 + grow_track - CLEAR)
    edge_v = outside(0.3 + VIA_D / 2)
    for l in layers:
        t, v, o, st, sv = arr[l]
        arr[l] = (t | edge_t, v | edge_v, o, st, sv)
    return arr


def island_cells(board, grid, net, pos, layer_name):
    """Cells of the copper item at pos (pad or track) on its layer(s)."""
    x, y = pos
    pt = pcbnew.VECTOR2I(MM(x), MM(y))
    best = None
    for f in board.GetFootprints():
        for p in f.Pads():
            if p.GetNetname() == net and p.HitTest(pt):
                best = p
    if best is None:
        for t in board.GetTracks():
            if t.GetNetname() == net and t.HitTest(pt, MM(0.05)):
                best = t
                break
    if best is None:
        return []
    out = []
    for l, lname in ((pcbnew.F_Cu, "F"), (pcbnew.B_Cu, "B")):
        if not best.IsOnLayer(l):
            continue
        img = grid.image()
        draw_item(grid, ImageDraw.Draw(img), best, l, 0)
        a = np.array(img, dtype=bool)
        js, is_ = np.nonzero(a)
        out += [(int(i), int(j), 0 if l == pcbnew.F_Cu else 1) for i, j in zip(is_, js)]
    return out


def astar(maps, starts, goals, rip_cost=0):
    tF, vF, oF, sF, svF = maps[pcbnew.F_Cu]
    tB, vB, oB, sB, svB = maps[pcbnew.B_Cu]
    blocked = (tF & ~oF, tB & ~oB)      # own-net copper is always passable
    soft = (sF & ~oF, sB & ~oB)
    vblock = vF | vB
    vsoft = svF | svB
    goal = set(goals)
    gx = np.mean([g[0] for g in goals]); gy = np.mean([g[1] for g in goals])
    def h(i, j):
        return math.hypot(i - gx, j - gy) * 0.5
    openq = []
    came = {}
    cost = {}
    for st in starts:
        cost[st] = 0
        heapq.heappush(openq, (h(st[0], st[1]), 0, st))
    steps = [(1, 0, 1), (-1, 0, 1), (0, 1, 1), (0, -1, 1),
             (1, 1, 1.4142), (1, -1, 1.4142), (-1, 1, 1.4142), (-1, -1, 1.4142)]
    ny, nx = tF.shape
    n = 0
    while openq:
        _, c, cur = heapq.heappop(openq)
        if cur in goal:
            path = [cur]
            while path[-1] in came:
                path.append(came[path[-1]])
            return path[::-1]
        if c > cost.get(cur, 1e18):
            continue
        n += 1
        if n > 20_000_000:
            return None
        i, j, l = cur
        for di, dj, w in steps:
            a, b = i + di, j + dj
            if not (0 <= a < nx and 0 <= b < ny):
                continue
            nxt = (a, b, l)
            if nxt not in goal:
                if blocked[l][b, a]:
                    continue
                if soft[l][b, a]:
                    if not rip_cost:
                        continue
                    w += rip_cost
            nc = c + w
            if nc < cost.get(nxt, 1e18):
                cost[nxt] = nc
                came[nxt] = cur
                heapq.heappush(openq, (nc + h(a, b), nc, nxt))
        if not vblock[j, i] and (rip_cost or not vsoft[j, i]):
            nxt = (i, j, 1 - l)
            nc = c + VIA_COST + (rip_cost * 5 if vsoft[j, i] else 0)
            if nc < cost.get(nxt, 1e18):
                cost[nxt] = nc
                came[nxt] = cur
                heapq.heappush(openq, (nc + h(i, j), nc, nxt))
    return None


def simplify(path):
    """Collapse the cell path into straight runs per layer, with vias at layer changes."""
    out = [path[0]]
    for k in range(1, len(path) - 1):
        a, b, c = out[-1], path[k], path[k + 1]
        if b[2] != a[2] or c[2] != b[2]:
            out.append(b)
            continue
        d1 = (b[0] - a[0], b[1] - a[1])
        d2 = (c[0] - b[0], c[1] - b[1])
        if d1[0] * d2[1] - d1[1] * d2[0] != 0:
            out.append(b)
    out.append(path[-1])
    return out


def add_path(board, grid, net, path, width):
    ni = board.FindNet(net)
    layers = (pcbnew.F_Cu, pcbnew.B_Cu)
    pts = simplify(path)
    items = []
    for a, b in zip(pts, pts[1:]):
        pa = pcbnew.VECTOR2I(*[MM(v) for v in grid.pos(a[0], a[1])])
        pb = pcbnew.VECTOR2I(*[MM(v) for v in grid.pos(b[0], b[1])])
        if a[2] != b[2]:
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pa)
            via.SetWidth(MM(VIA_D))
            via.SetDrill(MM(VIA_DRILL))
            via.SetNet(ni)
            board.Add(via)
            items.append(via)
            continue
        if pa == pb:
            continue
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(pa)
        t.SetEnd(pb)
        t.SetWidth(MM(width))
        t.SetLayer(layers[a[2]])
        t.SetNet(ni)
        board.Add(t)
        items.append(t)
    return items


def _pt_seg(p, a, b):
    ax, ay = a; bx, by = b; px, py = p
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
    return math.hypot(px - ax - t * dx, py - ay - t * dy)


def _seg_seg(a, b, c, d):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    if (ccw(a, c, d) > 0) != (ccw(b, c, d) > 0) and (ccw(a, b, c) > 0) != (ccw(a, b, d) > 0):
        return 0.0
    return min(_pt_seg(a, c, d), _pt_seg(b, c, d), _pt_seg(c, a, b), _pt_seg(d, a, b))


def _geom(t):
    if t.GetClass() == "PCB_VIA":
        p = t.GetPosition()
        q = (pcbnew.ToMM(p.x), pcbnew.ToMM(p.y))
        return q, q
    s, e = t.GetStart(), t.GetEnd()
    return (pcbnew.ToMM(s.x), pcbnew.ToMM(s.y)), (pcbnew.ToMM(e.x), pcbnew.ToMM(e.y))


def rip_conflicts(board, net, items):
    """Remove other-net tracks and vias the new path runs too close to."""
    new = [(_geom(i), pcbnew.ToMM(i.GetWidth()), i) for i in items]
    ripped = set()
    for t in list(board.GetTracks()):
        if t.GetNetname() == net or t.IsLocked():
            continue
        (a, b), w2 = _geom(t), pcbnew.ToMM(t.GetWidth())
        for (c, d), w1, it in new:
            same_layer = t.GetClass() == "PCB_VIA" or it.GetClass() == "PCB_VIA" or t.GetLayer() == it.GetLayer()
            if not same_layer:
                continue
            need = w1 / 2 + w2 / 2 + max(clearance_for(net), clearance_for(t.GetNetname())) - 0.005
            if _seg_seg(a, b, c, d) < need:
                ripped.add(t.GetNetname())
                board.Remove(t)
                break
    return ripped


def unconnected(report):
    txt = open(report).read()
    txt = txt[txt.find("unconnected pads"):]
    out = []
    for block in re.findall(r"\[unconnected_items\][^\[]*?(@\(.*?\n)\s*(@\(.*?\n)", txt, re.S):
        items = []
        for line in block:
            m = re.match(r"@\(([\d.]+) mm, ([\d.]+) mm\): .*?\[([^\]]+)\].*? on ([FB])\.Cu", line.strip())
            if m:
                items.append(((float(m.group(1)), float(m.group(2))), m.group(3), m.group(4)))
        if len(items) == 2 and items[0][1] == items[1][1]:
            out.append(items)
    return out


def main():
    rip_cost = float(sys.argv[1]) if len(sys.argv) > 1 else 0
    board = pcbnew.LoadBoard(rt.PCB)
    grid = Grid(board)
    pairs = unconnected(os.path.join(rt.WORK, "drc.rpt"))
    print(f"{len(pairs)} open connections")
    for (pa, net, la), (pb, _, lb) in pairs:
        w = width_for(net)
        c = clearance_for(net)
        s = island_cells(board, grid, net, pa, la)
        g = island_cells(board, grid, net, pb, lb)
        if not s or not g:
            print(f"  {net}: could not find its copper")
            continue
        gt, gv = w / 2 + c + SLACK, VIA_D / 2 + c + SLACK
        maps = obstacle_maps(board, grid, net, gt, gv)
        path = astar(maps, s, g)
        ripped = set()
        if not path and rip_cost:
            maps = obstacle_maps(board, grid, net, gt, gv, rip=True)
            path = astar(maps, s, g, rip_cost)
        if not path:
            print(f"  {net}: no path")
            continue
        items = add_path(board, grid, net, path, w)
        if rip_cost:
            ripped = rip_conflicts(board, net, items)
        print(f"  {net}: {len(items)} segments" + (f", ripped up {', '.join(sorted(ripped))}" if ripped else ""))
        if ripped - {"GND"}:
            break       # board edits invalidate pcbnew's track list; next round in a new process
    pcbnew.SaveBoard(rt.PCB, board)

if __name__ == "__main__":
    main()
