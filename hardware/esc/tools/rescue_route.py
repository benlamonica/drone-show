"""Finish the connections Freerouting left open, with a grid maze router.

    python3 tools/rescue_route.py build/rescue_in.json build/drc.json build/rescue_out.json

Plain Python 3 with numpy, scipy and Pillow (not KiCad's Python). Reads the
copper dumped by rescue_export.py and the open connections from a KiCad DRC
report, routes each open net on a 0.05 mm grid over the three routing
layers (top, In2, bottom, and the In1 GND plane at a price), and writes the new tracks
and vias for rescue_apply.py. Other nets' copper is kept the net class
clearance away; the refilling pours are not obstacles.

When a net can't get through, the router looks for a path that crosses the
autorouted tracks of a few other nets, rips those nets up, routes the stuck
net and then the ripped ones again. If any of them fails, it puts
everything back.
"""
import heapq
import json
import re
import sys

import numpy as np
from PIL import Image, ImageDraw
from scipy import ndimage

RES = 0.05
LAYERS = ("F", "In1", "In2", "B")
NL = len(LAYERS)
# In1 is the GND plane: a short hop there is allowed, at a price, when
# nothing else gets through
LAYER_COST = (1.0, 4.0, 1.0, 1.0)
VIA_COST = 30          # in grid steps, so 1.5 mm of track
CROSS_COST = 60        # for crossing another net's track when looking for what to rip up
MARGIN = 0.05          # extra clearance for the grid rounding
SKIP = ("GND", "+BATT")
NO_RIP = ("GND", "+BATT", "+3V3", "+10V")
MAX_RIP = 4


class Grid:
    def __init__(self, d):
        cx, cy = d["centre"]
        half = max(max(abs(x), abs(y)) for x, y in d["outline"]) + 1.0
        self.x0, self.y0 = cx - half, cy - half
        self.n = int(round(2 * half / RES))
        # copper by net code: fixed (pads, zones, what this router adds) and
        # rippable (the autorouter's tracks and vias)
        self.fixed = np.zeros((NL, self.n, self.n), dtype=np.int32)
        self.rip = np.zeros((NL, self.n, self.n), dtype=np.int32)
        self.joins = []       # (net, x, y, rippable) of plated holes, for layer joins
        for it in d["items"]:
            li = LAYERS.index(it["layer"])
            target = self.rip if it["kind"] in ("track", "via") else self.fixed
            for poly in it["polys"]:
                self.fill(target, li, poly, it["net"])
            if it["via"] and it["layer"] == "F":
                xs = [p[0] for poly in it["polys"] for p in poly]
                ys = [p[1] for poly in it["polys"] for p in poly]
                self.joins.append((it["net"], sum(xs) / len(xs), sum(ys) / len(ys), it["kind"] == "via"))
        self.update()
        # board edge (0.3 mm copper to edge) and the copper-free hole rings
        img = Image.new("1", (self.n, self.n), 0)
        ImageDraw.Draw(img).polygon([self.cell(cx + x, cy + y) for x, y in d["outline"]], fill=1)
        inside = np.array(img, dtype=bool)
        self.edge_dist = ndimage.distance_transform_edt(inside) * RES
        yy, xx = np.mgrid[0:self.n, 0:self.n]
        px, py = self.x0 + (xx + 0.5) * RES, self.y0 + (yy + 0.5) * RES
        self.hole_dist = np.full((self.n, self.n), 1e9)
        for hx, hy in d["holes"]:
            self.hole_dist = np.minimum(self.hole_dist,
                                        np.hypot(px - cx - hx, py - cy - hy) - d["hole_free"] / 2)

    def update(self):
        self.net = np.where(self.fixed != 0, self.fixed, self.rip)

    def snapshot(self):
        return self.fixed.copy(), self.rip.copy(), list(self.joins)

    def restore(self, snap):
        self.fixed, self.rip, self.joins = snap[0].copy(), snap[1].copy(), list(snap[2])
        self.update()

    def rip_net(self, code):
        self.rip[self.rip == code] = 0
        self.joins = [j for j in self.joins if not (j[0] == code and j[3])]
        self.update()

    def cell(self, x, y):
        return ((x - self.x0) / RES - 0.5, (y - self.y0) / RES - 0.5)

    def ij(self, x, y):
        return int(round((y - self.y0) / RES - 0.5)), int(round((x - self.x0) / RES - 0.5))

    def xy(self, i, j):
        return self.x0 + (j + 0.5) * RES, self.y0 + (i + 0.5) * RES

    def fill(self, target, li, poly, net):
        if len(poly) < 3:
            return
        pts = [self.cell(x, y) for x, y in poly]
        xs, ys = [p[0] for p in pts], [p[1] for p in pts]
        a0, b0 = max(0, int(min(xs)) - 1), max(0, int(min(ys)) - 1)
        a1, b1 = min(self.n, int(max(xs)) + 2), min(self.n, int(max(ys)) + 2)
        if a1 <= a0 or b1 <= b0:
            return
        img = Image.new("1", (a1 - a0, b1 - b0), 0)
        ImageDraw.Draw(img).polygon([(x - a0, y - b0) for x, y in pts], fill=1)
        m = np.array(img, dtype=bool)
        target[li, b0:b1, a0:a1][m] = net

    def line(self, li, x1, y1, x2, y2, w, net):
        """A track as a stadium polygon."""
        n = 12
        a = np.arctan2(y2 - y1, x2 - x1)
        r = w / 2
        pts = [(x2 + r * np.cos(a + t), y2 + r * np.sin(a + t)) for t in np.linspace(-np.pi / 2, np.pi / 2, n)]
        pts += [(x1 + r * np.cos(a + t), y1 + r * np.sin(a + t)) for t in np.linspace(np.pi / 2, 3 * np.pi / 2, n)]
        self.fill(self.fixed, li, pts, net)

    def disc(self, li, x, y, r, net):
        self.fill(self.fixed, li, [(x + r * np.cos(t), y + r * np.sin(t)) for t in np.linspace(0, 2 * np.pi, 24)],
                  net)


def clusters(g, code):
    """Label the net's copper; cells joined across layers at its vias."""
    labels = np.zeros_like(g.net)
    nxt = 0
    for li in range(NL):
        lab, k = ndimage.label(g.net[li] == code, structure=np.ones((3, 3)))
        labels[li] = np.where(lab > 0, lab + nxt, 0)
        nxt += k
    parent = list(range(nxt + 1))

    def find(a):
        while parent[a] != a:
            parent[a] = parent[parent[a]]
            a = parent[a]
        return a

    for net, x, y, _ in g.joins:
        if net != code:
            continue
        i, j = g.ij(x, y)
        ls = [labels[li, i, j] for li in range(NL) if labels[li, i, j]]
        for a in ls[1:]:
            parent[find(a)] = find(ls[0])
    roots = np.array([find(a) for a in range(nxt + 1)])
    return np.where(labels > 0, roots[labels], 0)


class Masks:
    """Where a net's track centre line and vias may go."""

    def __init__(self, g, code, info, clearance, soft=False):
        self.w, self.vd = info["width"], info["via"]
        hard = g.fixed if soft else g.net
        other = (hard != code) & (hard != 0)
        self.dist = np.stack([ndimage.distance_transform_edt(~other[li]) * RES for li in range(NL)])
        self.edge_ok = (g.edge_dist >= 0.3 + self.w / 2 + MARGIN) & (g.hole_dist >= self.w / 2 + MARGIN)
        self.via_edge_ok = (g.edge_dist >= 0.3 + self.vd / 2 + MARGIN) & (g.hole_dist >= self.vd / 2 + MARGIN)
        self.clearance = clearance
        self.cost = None
        if soft:
            # other nets' rippable copper: passable at a price, and remember whose
            ro = np.where((g.rip != code) & (g.fixed == 0), g.rip, 0)
            self.cost = np.zeros(ro.shape, dtype=np.float32)
            self.owner = np.zeros(ro.shape, dtype=np.int32)
            reach = clearance + max(self.w, self.vd) / 2 + MARGIN
            for li in range(NL):
                dd, ind = ndimage.distance_transform_edt(ro[li] == 0, return_indices=True)
                near = dd * RES < reach
                self.cost[li][near] = CROSS_COST
                self.owner[li] = np.where(near, ro[li][ind[0], ind[1]], 0)

    def compute(self, g, code):
        own = g.net == code
        c, w, vd = self.clearance, self.w, self.vd
        self.free = ((self.dist >= c + w / 2 + MARGIN) | (own & (self.dist >= c + MARGIN))) & self.edge_ok
        self.via_ok = (np.all(self.dist >= c + vd / 2 + MARGIN, axis=0) & self.via_edge_ok
                       & ~np.any(own, axis=0))


def route_net(g, code, info, clearance, soft=False):
    """Join all the net's copper islands. Returns (items, ok, nets crossed)."""
    m = Masks(g, code, info, clearance, soft)
    added, crossed = [], set()
    for _ in range(16):
        lab = clusters(g, code)
        roots = [r for r in np.unique(lab) if r]
        if len(roots) < 2:
            return added, True, crossed
        sizes = {r: int((lab == r).sum()) for r in roots}
        src = max(roots, key=lambda r: sizes[r])
        m.compute(g, code)
        path = search(g, lab, src, m)
        if path is None:
            return added, False, crossed
        if soft:
            # only find out what is in the way; nothing is added
            for li, i, j in path:
                if m.owner[li, i, j]:
                    crossed.add(int(m.owner[li, i, j]))
            return added, False, crossed
        for kind, *a in to_segments(g, path):
            if kind == "track":
                li, x1, y1, x2, y2 = a
                g.line(li, x1, y1, x2, y2, m.w, code)
                added.append({"type": "track", "layer": LAYERS[li], "x1": x1, "y1": y1, "x2": x2, "y2": y2,
                              "w": m.w, "net": info["name"]})
            else:
                x, y = a
                for li in range(NL):
                    g.disc(li, x, y, m.vd / 2, code)
                g.joins.append((code, x, y, False))
                added.append({"type": "via", "x": x, "y": y, "d": m.vd, "drill": info["drill"],
                              "net": info["name"]})
        g.update()
    return added, False, crossed


def search(g, lab, src, m):
    src_cells = np.argwhere(lab == src)
    goal = (lab != 0) & (lab != src)
    allc = np.argwhere(lab != 0)
    pad = int(12 / RES)
    i0, j0 = np.maximum(allc[:, 1:].min(axis=0) - pad, 0)
    i1, j1 = np.minimum(allc[:, 1:].max(axis=0) + pad, g.n - 1)
    # A*: straight-line distance to the nearest goal cell on any layer
    h = ndimage.distance_transform_edt(~np.any(goal, axis=0))
    free, via_ok, cost = m.free, m.via_ok, m.cost
    best, prev, pq = {}, {}, []
    for li, i, j in src_cells:
        s = (int(li), int(i), int(j))
        best[s] = 0.0
        heapq.heappush(pq, (h[i, j], 0.0, s))
    steps = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
             (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
    while pq:
        _, c, s = heapq.heappop(pq)
        if c > best.get(s, 1e18):
            continue
        li, i, j = s
        if goal[li, i, j]:
            path = [s]
            while path[-1] in prev:
                path.append(prev[path[-1]])
            return path[::-1]
        moves = []
        for di, dj, step in steps:
            ni, nj = i + di, j + dj
            if i0 <= ni <= i1 and j0 <= nj <= j1 and free[li, ni, nj]:
                if di and dj and not (free[li, i, nj] and free[li, ni, j]):
                    continue
                moves.append(((li, ni, nj), step * LAYER_COST[li]))
        if via_ok[i, j]:
            for nl in range(NL):
                if nl != li:
                    moves.append(((nl, i, j), VIA_COST))
        for t, step in moves:
            nc = c + step
            if cost is not None:
                nc += cost[t]
            if nc < best.get(t, 1e18):
                best[t] = nc
                prev[t] = s
                heapq.heappush(pq, (nc + h[t[1], t[2]], nc, t))
    return None


def to_segments(g, path):
    out = []
    start = path[0]
    for k in range(1, len(path)):
        a, b = path[k - 1], path[k]
        if a[0] != b[0]:
            if start != a:
                out.append(("track", a[0], *g.xy(*start[1:]), *g.xy(*a[1:])))
            out.append(("via", *g.xy(*a[1:])))
            start = b
            continue
        if k + 1 < len(path):
            c = path[k + 1]
            if c[0] == b[0] and (c[1] - b[1], c[2] - b[2]) == (b[1] - a[1], b[2] - a[2]):
                continue
        if start != b:
            out.append(("track", b[0], *g.xy(*start[1:]), *g.xy(*b[1:])))
        start = b
    return out


# The MCU-to-driver PWM lines are nested: route them inside out, high side
# first, so an early one can't wall in the next
PWM_ORDER = ("CH", "BH", "AH", "AL", "BL", "CL")


def order(name):
    m = re.match(r"/(ESC\d)/(\w+)$", name)
    if m and m.group(2) in PWM_ORDER:
        return (0, m.group(1), PWM_ORDER.index(m.group(2)))
    # then the driver's bootstrap pins, boxed in among its gate pins
    if re.search(r"-VB\d\)$", name):
        return (1, name, 0)
    return (2, name, 0)


def main(src, drc, dst):
    d = json.load(open(src))
    g = Grid(d)
    report = json.load(open(drc))
    names = set()
    for u in report["unconnected_items"]:
        for it in u["items"]:
            m = re.search(r"\[(.+?)\]", it["description"])
            if m and m.group(1) not in SKIP:
                names.add(m.group(1))
    # nets whose tracks rescue_export.py ripped out of the way of the stubs
    names |= set(d.get("ripped", []))
    codes = {v["name"]: int(k) for k, v in d["nets"].items()}
    nets = {int(k): v for k, v in d["nets"].items()}
    added, ripped, failed = [], [], []
    for name in sorted(names, key=order):
        code = codes[name]
        info = nets[code]
        cl = info.get("clearance", 0.2)
        snap = g.snapshot()
        new, ok, _ = route_net(g, code, info, cl)
        if not ok:
            # partial result out; look for what to rip up
            g.restore(snap)
            _, _, crossed = route_net(g, code, info, cl, soft=True)
            crossed = {c for c in crossed if nets[c]["name"] not in NO_RIP and c != code}
            new, ok = [], False
            if crossed and len(crossed) <= MAX_RIP:
                for c in crossed:
                    g.rip_net(c)
                new, ok, _ = route_net(g, code, info, cl)
                for c in sorted(crossed):
                    if not ok:
                        break
                    more, ok, _ = route_net(g, c, nets[c], nets[c].get("clearance", 0.2))
                    new += more
                if ok:
                    ripped += [nets[c]["name"] for c in crossed]
                    print(f"{name}: ok after ripping up {[nets[c]['name'] for c in crossed]}", flush=True)
                else:
                    g.restore(snap)
                    new = []
            if not ok:
                # keep what does connect, without ripping anything
                new, _, _ = route_net(g, code, info, cl)
                print(f"{name}: FAILED", flush=True)
                failed.append(name)
        else:
            print(f"{name}: ok", flush=True)
        added += new
    # a ripped net's earlier rescue tracks are fixed copper and stay; only
    # the autorouter's copper of ripped nets is deleted
    out = [{"type": "rip", "net": n} for n in sorted(set(ripped))] + added
    json.dump(out, open(dst, "w"), indent=0)
    print(f"{len(added)} items, ripped {sorted(set(ripped))}, failed: {failed}")


if __name__ == "__main__":
    main(*sys.argv[1:4])
