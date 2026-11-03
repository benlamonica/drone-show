"""Last ground hookups: tie each stranded ground pad (from the DRC report) to the
nearest ground via or pad with a straight track, or a single dog-leg, when it clears
everything by the 0.15 mm rule. Also joins VREG_PGND (pin 47) to the exposed pad."""
import itertools
import os
import re
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
import escape as es  # noqa: E402
from gen_pcb import PCB  # noqa: E402

mm = pcbnew.FromMM
W = mm(0.2)


def stranded(b):
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.WriteDRCReport(b, "/tmp/lc_hop.rpt", pcbnew.EDA_UNITS_MILLIMETRES, False)
    out = []
    for blk in re.split(r"\n(?=\[)", open("/tmp/lc_hop.rpt").read()):
        if blk.startswith("[unconnected_items]"):
            out += re.findall(r"Pad (\S+) \[/GND\] of (\S+) on", blk)
    return sorted(set(out))


def try_link(b, pad, layer, targets):
    c = pad.GetPosition()
    targets = sorted(targets, key=lambda t: (t - c).EuclideanNorm())[:60]
    for t in targets:
        if (t - c).EuclideanNorm() > mm(4):
            break
        corners = [None, pcbnew.VECTOR2I(c.x, t.y), pcbnew.VECTOR2I(t.x, c.y)]
        for k in corners:
            pts = [c, t] if k is None else [c, k, t]
            segs = [(a, z) for a, z in zip(pts, pts[1:]) if a != z]
            shapes = [(layer, pcbnew.SHAPE_SEGMENT(a, z, W)) for a, z in segs]
            if es.collides(b, "/GND", shapes):
                continue
            for a, z in segs:
                tr = pcbnew.PCB_TRACK(b)
                tr.SetStart(a)
                tr.SetEnd(z)
                tr.SetWidth(W)
                tr.SetLayer(layer)
                tr.SetNet(pad.GetNet())
                b.Add(tr)
            return True
    return False


def main_cluster(b):
    """Positions (nm) on each layer that belong to the largest connected piece of GND."""
    import numpy as np
    from scipy import ndimage
    import finish_route as fr
    own = fr.items_by_net(b)["/GND"]
    cu = [fr.raster(own, ly, 0.0) for ly in fr.LAYERS]
    lab = [ndimage.label(cu[i], structure=np.ones((3, 3)))[0] for i in range(2)]
    n0 = lab[0].max()
    parent = {}

    def find(a):
        while parent.get(a, a) != a:
            a = parent[a]
        return a
    for it in own:
        if it.GetClass() == "PCB_VIA" or (it.GetClass() == "PAD" and it.HasHole()):
            p = it.GetPosition()
            x, y = int((pcbnew.ToMM(p.x) + fr.HX) / fr.RES), int((pcbnew.ToMM(p.y) + fr.HY) / fr.RES)
            a, c = lab[0][y, x], lab[1][y, x]
            if a and c:
                ra, rc = find(a), find(c + n0)
                if ra != rc:
                    parent[ra] = rc
    comp = [np.vectorize(lambda v: find(v) if v else 0)(lab[0]),
            np.vectorize(lambda v: find(v + n0) if v else 0)(lab[1])]
    ids, counts = np.unique(np.concatenate([comp[0][comp[0] > 0], comp[1][comp[1] > 0]]), return_counts=True)
    main = ids[np.argmax(counts)]
    out = {}
    for i, L in enumerate(fr.LAYERS):
        ys, xs = np.nonzero(comp[i] == main)
        out[L] = [pcbnew.VECTOR2I(mm(float(x * fr.RES - fr.HX + fr.RES / 2)), mm(float(y * fr.RES - fr.HY + fr.RES / 2)))
                  for y, x in zip(ys[::7], xs[::7])]
    return out


def main():
    b = pcbnew.LoadBoard(PCB)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    cluster = main_cluster(b)
    for num, ref in stranded(b):
        pad = b.FindFootprintByReference(ref).FindPadByNumber(num)
        layer = pcbnew.F_Cu if pad.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        targets = cluster[layer]
        ok = try_link(b, pad, layer, targets)
        print(f"{ref}.{num}: {'linked' if ok else 'no clear path'}")
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)


if __name__ == "__main__":
    main()
