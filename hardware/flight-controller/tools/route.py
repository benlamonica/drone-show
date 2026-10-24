"""Route show-fc.kicad_pcb: Specctra export -> Freerouting -> import, then
ground pours on both layers, stitching vias and a DRC report.

    python3 route.py [--passes N] [--reuse-ses]

Needs Freerouting 1.9 (FREEROUTING_JAR, default /opt/fr/fr.jar) and Java.
On a machine without a display it runs under xvfb-run.
"""

import argparse
import os
import re
import shutil
import subprocess
import sys

import pcbnew

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PCB = os.path.join(ROOT, "show-fc.kicad_pcb")
WORK = os.path.join(ROOT, "build")
# "pour": every ground pad gets its own via first and ground is kept out of the
# router; "route": the router wires ground too, and the pours are added after
GND_MODE = os.environ.get("GND_MODE", "route")
MM = pcbnew.FromMM

sys.path.insert(0, HERE)
import make_board as mb  # noqa: E402


def sexp_tokens(text):
    return re.findall(r'\(|\)|"[^"]*"|[^\s()]+', text)


def parse(tokens, i=0):
    out = []
    while i < len(tokens):
        t = tokens[i]
        if t == "(":
            sub, i = parse(tokens, i + 1)
            out.append(sub)
        elif t == ")":
            return out, i + 1
        else:
            out.append(t.strip('"'))
            i += 1
    return out, i


def find(node, key):
    for n in node:
        if isinstance(n, list) and n and n[0] == key:
            yield n


def import_ses(board, path):
    tree, _ = parse(sexp_tokens(open(path).read()))
    sess = tree[0]
    routes = next(find(sess, "routes"))
    res = next(find(routes, "resolution"))
    scale = float(res[2])            # units per micron
    layers = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}
    lib = next(find(routes, "library_out"), [])
    via_sizes = {}
    for ps in find(lib, "padstack"):
        name = ps[1]
        m = re.search(r"(\d+):(\d+)_um", name)
        if m:
            via_sizes[name] = (int(m.group(1)), int(m.group(2)))

    def pt(x, y):
        return pcbnew.VECTOR2I(int(float(x) / scale * 1000), int(-float(y) / scale * 1000))

    n_tracks = n_vias = 0
    for net in find(next(find(routes, "network_out")), "net"):
        ni = board.FindNet(net[1])
        if net[1] == "GND" and GND_MODE == "pour":
            continue          # only our own fixed fan-out, already on the board
        for w in find(net, "wire"):
            p = next(find(w, "path"))
            layer, width = layers[p[1]], int(float(p[2]) / scale * 1000)
            coords = p[3:]
            pts = [pt(coords[k], coords[k + 1]) for k in range(0, len(coords) - 1, 2)]
            for a, b in zip(pts, pts[1:]):
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(a)
                t.SetEnd(b)
                t.SetWidth(width)
                t.SetLayer(layer)
                t.SetNet(ni)
                board.Add(t)
                n_tracks += 1
        for v in find(net, "via"):
            dia, drill = via_sizes.get(v[1], (600, 300))
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pt(v[2], v[3]))
            via.SetWidth(dia * 1000)
            via.SetDrill(drill * 1000)
            via.SetLayerPair(pcbnew.F_Cu, pcbnew.B_Cu)
            via.SetNet(ni)
            board.Add(via)
            n_vias += 1
    return n_tracks, n_vias


def fanout_ground(board):
    """Give every ground pad its own via to the bottom pour, before routing.

    Ground is then left out of the autorouter (the pours carry it), which frees
    most of the routing space a 2-layer board has. Returns pads that found no
    spot; the top pour still reaches those directly.
    """
    import math
    gnd = board.FindNet("GND")
    gcode = gnd.GetNetCode()
    obst = []      # (x0, y0, x1, y1, netcode) copper rectangles, mm
    holes = []
    court = []
    for f in board.GetFootprints():
        c = mb.courtyard(f, 0.0)
        court.append((f.GetReference(), c[0] + mb.CX, c[1] + mb.CY, c[2] + mb.CX, c[3] + mb.CY))
        for p in f.Pads():
            bb = p.GetBoundingBox()
            obst.append((pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()),
                         pcbnew.ToMM(bb.GetBottom()), p.GetNetCode(), f.GetReference()))
            if p.GetDrillSize().x > 0:
                pp = p.GetPosition()
                holes.append((pcbnew.ToMM(pp.x), pcbnew.ToMM(pp.y), pcbnew.ToMM(p.GetDrillSize().x) / 2))

    def rect_dist(x, y, r):
        dx = max(r[0] - x, 0, x - r[2])
        dy = max(r[1] - y, 0, y - r[3])
        return math.hypot(dx, dy)

    def seg_clear(ax, ay, bx, by, own_ref, own_pad_rect, w=0.25):
        for k in range(11):
            t = k / 10
            x, y = ax + (bx - ax) * t, ay + (by - ay) * t
            for o in obst:
                if o[4] == gcode:
                    continue
                if rect_dist(x, y, o) < w / 2 + 0.15:
                    return False
        return True

    u1 = board.FindFootprintByReference("U1").GetPosition()
    ux, uy = pcbnew.ToMM(u1.x), pcbnew.ToMM(u1.y)
    failed, n = [], 0
    for f in board.GetFootprints():
        ref = f.GetReference()
        if ref.startswith(("TP", "H")):
            continue
        for p in f.Pads():
            if p.GetNetCode() != gcode or p.GetAttribute() != pcbnew.PAD_ATTRIB_SMD:
                continue
            if ref == "U1" and p.GetNumber() == "81":
                continue       # exposed pad already has 9 vias
            if ref == "U1":
                # VREG_PGND: straight stub inward onto the exposed pad
                pp = p.GetPosition()
                ep = f.GetPosition()
                end = pcbnew.VECTOR2I(pp.x, ep.y) if abs(pp.x - ep.x) < abs(pp.y - ep.y) else pcbnew.VECTOR2I(ep.x, pp.y)
                t = pcbnew.PCB_TRACK(board)
                t.SetStart(pp)
                t.SetEnd(end)
                t.SetWidth(MM(0.2))
                t.SetLayer(pcbnew.F_Cu)
                t.SetNet(gnd)
                t.SetLocked(True)
                board.Add(t)
                n += 1
                continue
            pp = p.GetPosition()
            px, py = pcbnew.ToMM(pp.x), pcbnew.ToMM(pp.y)
            bb = p.GetBoundingBox()
            prect = (pcbnew.ToMM(bb.GetLeft()), pcbnew.ToMM(bb.GetTop()), pcbnew.ToMM(bb.GetRight()), pcbnew.ToMM(bb.GetBottom()))
            # a via that already touches this pad's net nearby is enough
            best = None
            # try directions pointing away from the MCU first, and never drop a
            # via into the ring the QFN escapes through
            away = math.atan2(py - uy, px - ux)
            angles = sorted((2 * math.pi * k / 16 for k in range(16)),
                            key=lambda a: -math.cos(a - away))
            for dist in (0.75, 0.9, 1.1, 1.3, 1.6, 2.0):
                for a in angles:
                    hw, hh = (prect[2] - prect[0]) / 2, (prect[3] - prect[1]) / 2
                    vx, vy = px + (hw + dist) * math.cos(a), py + (hh + dist) * math.sin(a)
                    if abs(vx - ux) < 6.9 and abs(vy - uy) < 6.9:
                        continue
                    if not mb.inside(vx - mb.CX, vy - mb.CY, 0.7):
                        continue
                    if any(rect_dist(vx, vy, o) < 0.3 + (0.15 if o[4] != gcode else 0.05) for o in obst):
                        continue
                    if any(math.hypot(vx - hx, vy - hy) < hr + 0.3 + 0.45 for hx, hy, hr in holes):
                        continue
                    # not under another part
                    if any(c[1] < vx < c[3] and c[2] < vy < c[4] for c in court if c[0] != ref):
                        continue
                    if not seg_clear(px, py, vx, vy, ref, prect):
                        continue
                    best = (vx, vy)
                    break
                if best:
                    break
            if not best:
                failed.append(f"{ref}.{p.GetNumber()}")
                continue
            vx, vy = best
            via = pcbnew.PCB_VIA(board)
            via.SetPosition(pcbnew.VECTOR2I(MM(vx), MM(vy)))
            via.SetWidth(MM(0.6))
            via.SetDrill(MM(0.3))
            via.SetNet(gnd)
            via.SetLocked(True)
            board.Add(via)
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pp)
            t.SetEnd(pcbnew.VECTOR2I(MM(vx), MM(vy)))
            t.SetWidth(MM(0.25))
            t.SetLayer(pcbnew.F_Cu)
            t.SetNet(gnd)
            t.SetLocked(True)
            board.Add(t)
            holes.append((vx, vy, 0.15))
            obst.append((vx - 0.3, vy - 0.3, vx + 0.3, vy + 0.3, gcode, "via"))
            n += 1
    return n, failed


def strip_ground(dsn):
    """Remove GND's pin list from the Specctra netlist so the router leaves it alone."""
    txt = open(dsn).read()
    txt = re.sub(r"\(net GND\s*\(pins[^)]*\)", "(net GND", txt)
    open(dsn, "w").write(txt)


def clear_routing(board):
    for t in list(board.GetTracks()):
        board.Remove(t)
    for z in [board.GetArea(i) for i in range(board.GetAreaCount())]:
        board.Remove(z)


def add_ground_zones(board):
    gnd = board.FindNet("GND")
    pts = mb.outline_pts()
    for layer in (pcbnew.F_Cu, pcbnew.B_Cu):
        z = pcbnew.ZONE(board)
        z.SetLayer(layer)
        z.SetNet(gnd)
        outline = z.Outline()
        outline.NewOutline()
        for x, y in pts:
            outline.Append(MM(mb.CX + x), MM(mb.CY + y))
        z.SetLocalClearance(MM(0.25))
        z.SetMinThickness(MM(0.2))
        z.SetThermalReliefGap(MM(0.25))
        z.SetThermalReliefSpokeWidth(MM(0.3))
        z.SetPadConnection(pcbnew.ZONE_CONNECTION_THERMAL)
        z.SetIslandRemovalMode(pcbnew.ISLAND_REMOVAL_MODE_ALWAYS)
        z.SetAssignedPriority(0) if hasattr(z, "SetAssignedPriority") else z.SetPriority(0)
        board.Add(z)
    # exposed pads, and the two ground pads with no room for a via, join the
    # pour solidly so they never end up on a single thermal spoke
    for ref, num in (("U1", "81"), ("U4", "7"), ("U5", "7"), ("U2", "2"), ("U7", "7")):
        f = board.FindFootprintByReference(ref)
        for pad in f.Pads():
            if pad.GetNumber() == num:
                pad.SetZoneConnection(pcbnew.ZONE_CONNECTION_FULL)


def fill(board):
    filler = pcbnew.ZONE_FILLER(board)
    filler.Fill(board.Zones())


def _seg_dist(px, py, ax, ay, bx, by):
    dx, dy = bx - ax, by - ay
    L = dx * dx + dy * dy
    t = 0 if L == 0 else max(0, min(1, ((px - ax) * dx + (py - ay) * dy) / L))
    cx, cy = ax + t * dx, ay + t * dy
    return ((px - cx) ** 2 + (py - cy) ** 2) ** 0.5


def stitch(board, pitch=2.5):
    """GND vias on a grid where both pours are solid copper, tying the two pours together.

    A candidate must clear every other-net track and pad by the via radius plus
    0.25 mm, every via/hole by 0.6 mm, sit outside all courtyards (no vias under
    parts) and inside the filled ground on both layers.
    """
    gnd = board.FindNet("GND")
    gcode = gnd.GetNetCode()
    zones = {z.GetLayer(): z for z in (board.GetArea(i) for i in range(board.GetAreaCount()))}
    tracks, holes = [], []
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA":
            p = t.GetPosition()
            holes.append((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y), pcbnew.ToMM(t.GetWidth()) / 2))
            if t.GetNetCode() != gcode:
                tracks.append((pcbnew.ToMM(p.x), pcbnew.ToMM(p.y), pcbnew.ToMM(p.x), pcbnew.ToMM(p.y),
                               pcbnew.ToMM(t.GetWidth()) / 2))
        elif t.GetNetCode() != gcode:
            a, b = t.GetStart(), t.GetEnd()
            tracks.append((pcbnew.ToMM(a.x), pcbnew.ToMM(a.y), pcbnew.ToMM(b.x), pcbnew.ToMM(b.y),
                           pcbnew.ToMM(t.GetWidth()) / 2))
    boxes = []
    for f in board.GetFootprints():
        c = mb.courtyard(f, 0.1)
        boxes.append((c[0] + mb.CX, c[1] + mb.CY, c[2] + mb.CX, c[3] + mb.CY))
        for pad in f.Pads():
            bb = pad.GetBoundingBox()
            boxes.append((pcbnew.ToMM(bb.GetLeft()) - 0.3, pcbnew.ToMM(bb.GetTop()) - 0.3,
                          pcbnew.ToMM(bb.GetRight()) + 0.3, pcbnew.ToMM(bb.GetBottom()) + 0.3))
    added = 0
    r = 0.3
    x = -mb.HALF + 1.0
    while x < mb.HALF:
        y = -mb.HALF + 1.0
        while y < mb.HALF:
            X, Y = mb.CX + x, mb.CY + y
            ok = mb.inside(x, y, 1.0)
            ok = ok and not any(b[0] - r < X < b[2] + r and b[1] - r < Y < b[3] + r for b in boxes)
            ok = ok and all(_seg_dist(X, Y, *t[:4]) > t[4] + r + 0.25 for t in tracks)
            ok = ok and all(((X - hx) ** 2 + (Y - hy) ** 2) ** 0.5 > hr + r + 0.6 for hx, hy, hr in holes)
            if ok:
                p = pcbnew.VECTOR2I(MM(X), MM(Y))
                ok = all(zones[l].GetFilledPolysList(l).Contains(p) for l in (pcbnew.F_Cu, pcbnew.B_Cu))
            if ok:
                via = pcbnew.PCB_VIA(board)
                via.SetPosition(pcbnew.VECTOR2I(MM(X), MM(Y)))
                via.SetWidth(MM(0.6))
                via.SetDrill(MM(0.3))
                via.SetNet(gnd)
                board.Add(via)
                holes.append((X, Y, r))
                added += 1
            y += pitch
        x += pitch
    return added


def drc(board, path):
    pcbnew.WriteDRCReport(board, path, pcbnew.EDA_UNITS_MILLIMETRES, True)
    txt = open(path).read()
    counts = {}
    for m in re.finditer(r"^\[(\w+)\]", txt, re.M):
        counts[m.group(1)] = counts.get(m.group(1), 0) + 1
    unconnected = re.search(r"\*\* Found (\d+) unconnected pads", txt)
    return counts, (int(unconnected.group(1)) if unconnected else None)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--passes", type=int, default=30)
    ap.add_argument("--reuse-ses", action="store_true")
    args = ap.parse_args()
    os.makedirs(WORK, exist_ok=True)
    dsn, ses = os.path.join(WORK, "show-fc.dsn"), os.path.join(WORK, "show-fc.ses")

    # always start from a fresh, unrouted placement
    subprocess.run([sys.executable, os.path.join(HERE, "make_board.py")], check=True)
    board = pcbnew.LoadBoard(PCB)
    if GND_MODE == "pour":
        nf, failed = fanout_ground(board)
        print(f"ground fan-out: {nf} vias; no room at {' '.join(failed) or 'none'}")
    if not args.reuse_ses:
        pcbnew.ExportSpecctraDSN(board, dsn)
        if GND_MODE == "pour":
            strip_ground(dsn)
        jar = os.environ.get("FREEROUTING_JAR", "/opt/fr/fr.jar")
        cmd = ["java", "-jar", jar, "-de", dsn, "-do", ses, "-mp", str(args.passes)]
        if not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
            cmd = ["xvfb-run", "-a"] + cmd
        print("routing:", " ".join(cmd))
        subprocess.run(cmd, check=True, stdout=open(os.path.join(WORK, "freerouting.log"), "w"),
                       stderr=subprocess.STDOUT)
    nt, nv = import_ses(board, ses)
    print(f"imported {nt} track segments, {nv} vias")
    add_ground_zones(board)
    pcbnew.SaveBoard(PCB, board)
    board = pcbnew.LoadBoard(PCB)          # zone filling is only stable on a freshly loaded board
    board.BuildConnectivity()
    fill(board)
    print("stitching vias:", stitch(board))
    pcbnew.SaveBoard(PCB, board)
    board = pcbnew.LoadBoard(PCB)
    board.BuildConnectivity()
    fill(board)
    pcbnew.SaveBoard(PCB, board)
    counts, unc = drc(board, os.path.join(WORK, "drc.rpt"))
    print("DRC:", counts, "unconnected pads:", unc)


if __name__ == "__main__":
    main()
