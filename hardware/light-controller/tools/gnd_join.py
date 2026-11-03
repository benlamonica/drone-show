"""Join leftover ground clusters to the main one with a via placed where the
cluster's copper on one layer overlies the main cluster's copper on the other."""
import os
import sys

import numpy as np
import pcbnew
from scipy import ndimage

sys.path.insert(0, os.path.dirname(__file__))
import escape as es  # noqa: E402
import finish_route as fr  # noqa: E402

mm = pcbnew.FromMM


def clusters(b):
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
            if a and c and find(a) != find(c + n0):
                parent[find(a)] = find(c + n0)
    comp = [np.vectorize(lambda v: find(v) if v else 0)(lab[0]),
            np.vectorize(lambda v: find(v + n0) if v else 0)(lab[1])]
    ids, counts = np.unique(np.concatenate([comp[0][comp[0] > 0], comp[1][comp[1] > 0]]), return_counts=True)
    return comp, ids[np.argmax(counts)], [i for i in ids if i != ids[np.argmax(counts)]]


def main():
    b = pcbnew.LoadBoard(fr.PCB)
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    comp, main_id, others = clusters(b)
    r = int(0.1 / fr.RES)
    disk = np.hypot(*np.mgrid[-r:r + 1, -r:r + 1]) <= r
    gnd = b.FindNet("/GND")
    for cid in others:
        done = False
        for L in (0, 1):
            here = ndimage.binary_erosion(comp[L] == cid, structure=disk)
            there = ndimage.binary_erosion(comp[1 - L] == main_id, structure=disk)
            ys, xs = np.nonzero(here & there)
            if not len(ys):
                # nothing of the main ground underneath: a via where the other layer is
                # free lets that layer's pour refill around it, if the pour can reach
                ys, xs = np.nonzero(here & (comp[1 - L] == 0))
            for y, x in list(zip(ys, xs))[::5]:
                pos = pcbnew.VECTOR2I(mm(float(x * fr.RES - fr.HX)), mm(float(y * fr.RES - fr.HY)))
                if es.collides(b, "/GND", [(None, pcbnew.SHAPE_CIRCLE(pos, mm(0.3)))]) or not es.holes_ok(b, pos):
                    continue
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pos)
                v.SetWidth(mm(0.6))
                v.SetDrill(mm(0.3))
                v.SetNet(gnd)
                b.Add(v)
                pcbnew.ZONE_FILLER(b).Fill(b.Zones())
                c2, m2, o2 = clusters(b)
                if len(o2) < len(others):
                    print(f"cluster {cid}: via at {pcbnew.ToMM(pos)}")
                    others = o2
                    done = True
                    break
                b.Delete(v)
            if done:
                break
        if not done:
            print(f"cluster {cid}: no overlap with the main ground")
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(fr.PCB, b)


if __name__ == "__main__":
    main()
