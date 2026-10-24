"""Import the Freerouting result, fill the pours and write the final board.

    python3 tools/finish_pcb.py build/fanned.kicad_pcb build/route.ses drone-esc.kicad_pcb
"""
import math
import re
import sys

import pcbnew

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import layout as L  # noqa: E402

MM = pcbnew.FromMM


def stackup_2oz(path):
    """2 oz (70 um) copper in the stored stackup. The stackup isn't reachable
    from KiCad's Python, so this edits the saved file."""
    t = open(path).read()
    t = re.sub(r'(\(layer "[FB]\.Cu"\s*\(type "copper"\)\s*\(thickness )0\.035\)', r"\g<1>0.07)", t)
    open(path, "w").write(t)


def tidy_silk(board):
    """Reference designators only where they help someone with a soldering
    iron: connectors, ICs, test points. Passives and FETs are listed in the
    assembly drawing and the pick-and-place file instead."""
    for fp in board.GetFootprints():
        ref = fp.Reference()
        keep = fp.GetReference().startswith(("J", "TP")) or fp.GetValue() in (
            "AT32F421G8U7", "NSG2065Q")
        ref.SetVisible(keep)
        ref.SetTextSize(pcbnew.VECTOR2I(MM(0.8), MM(0.8)))
        ref.SetTextThickness(MM(0.15))


def stitch_islands(board):
    """Put a via into every piece of outer GND / +BATT pour that has none, so
    each piece reaches its inner plane. Pours get cut into pieces by the FETs
    and the routing, and a piece that only touches a pad would otherwise leave
    that pad floating."""
    VIA_R, CLR = 0.33, 0.2
    others = [t for t in board.GetTracks()] + [p for p in board.GetPads()]
    vias = [t for t in board.GetTracks() if t.GetClass() == "PCB_VIA"]
    holes = [(150.0 + x, 100.0 + y) for x, y in L.holes()]   # board centre is (150, 100)
    added = 0

    def free(pos, net):
        x, y = pcbnew.ToMM(pos.x), pcbnew.ToMM(pos.y)
        if any(math.hypot(x - hx, y - hy) < L.HOLE_COPPER_FREE / 2 + VIA_R + 0.1 for hx, hy in holes):
            return False
        for v in vias:
            vp = v.GetPosition()
            if math.hypot(pcbnew.ToMM(vp.x) - x, pcbnew.ToMM(vp.y) - y) < 0.85:
                return False
        for it in others:
            if it.GetNetCode() == net:
                continue
            if it.HitTest(pos, MM(VIA_R + CLR)):
                return False
        return True

    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetNetname() not in ("GND", "+BATT"):
            continue
        layer = z.GetLayer()
        if layer not in (pcbnew.F_Cu, pcbnew.B_Cu):
            continue
        net = z.GetNetCode()
        polys = z.GetFilledPolysList(layer)
        for i in range(polys.OutlineCount()):
            unit = polys.UnitSet(i)
            if any(unit.Contains(v.GetPosition()) for v in vias if v.GetNetCode() == net):
                continue
            if any(unit.Contains(p.GetPosition()) for p in board.GetPads()
                   if p.GetNetCode() == net and p.GetAttribute() == pcbnew.PAD_ATTRIB_PTH):
                continue
            box = unit.BBox()
            step = MM(0.25)
            done = False
            for gx in range(box.GetLeft(), box.GetRight(), step):
                for gy in range(box.GetTop(), box.GetBottom(), step):
                    pos = pcbnew.VECTOR2I(gx, gy)
                    ring = [pcbnew.VECTOR2I(int(gx + MM(VIA_R + 0.05) * math.cos(a * math.pi / 4)),
                                            int(gy + MM(VIA_R + 0.05) * math.sin(a * math.pi / 4)))
                            for a in range(8)]
                    if not all(unit.Contains(q) for q in [pos] + ring) or not free(pos, net):
                        continue
                    v = pcbnew.PCB_VIA(board)
                    v.SetPosition(pos)
                    v.SetWidth(MM(0.66))
                    v.SetDrill(MM(0.3))
                    v.SetNet(z.GetNet())
                    board.Add(v)
                    vias.append(v)
                    others.append(v)
                    added += 1
                    done = True
                    break
                if done:
                    break
    return added


def main(src, ses, out):
    board = pcbnew.LoadBoard(src)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("SES import failed")
    tidy_silk(board)
    # the SES import gives router vias the default 0.4 mm drill; the via sizes are 0.66/0.3 and 0.8/0.4
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            t.SetDrill(MM(0.3) if t.GetWidth(pcbnew.F_Cu) < MM(0.7) else MM(0.4))
    for _ in range(3):
        pcbnew.ZONE_FILLER(board).Fill(board.Zones())
        n = stitch_islands(board)
        print("island vias", n)
        if not n:
            break
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(out)
    stackup_2oz(out)
    print("saved", out)


if __name__ == "__main__":
    main(*sys.argv[1:4])
