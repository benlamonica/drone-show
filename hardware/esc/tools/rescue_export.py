"""Dump the routed board's copper for rescue_route.py. Run with KiCad's Python.

    python3 tools/rescue_export.py build/routed.kicad_pcb build/drc.json build/stubbed.kicad_pcb build/rescue_in.json
    python3 tools/rescue_export.py dump build/stubbed.kicad_pcb build/rescue_in.json

Freerouting leaves a few connections open in the crowded middle of the
board. rescue_route.py finishes them with a plain grid maze router; this
script hands it every piece of copper as polygons, in mm.

The grid can't measure clearance finely enough to get out from between
fine-pitch pins, so first every open fine-pitch pin gets a short straight
stub out past the end of its pad, checked here against the real shapes.
"""
import json
import re
import sys

import pcbnew

sys.path.insert(0, __file__.rsplit("/", 1)[0])
import layout as L  # noqa: E402

LAYERS = {pcbnew.F_Cu: "F", pcbnew.In1_Cu: "In1", pcbnew.In2_Cu: "In2", pcbnew.B_Cu: "B"}
# pours that refill round whatever is added: not obstacles
POURS = ("BATT top", "GND bottom", "BATT plane", "GND plane")
NO_RIP = ("GND", "+BATT", "+3V3", "+10V")
# autorouted nets in a stub's way are ripped up for the maze router to put
# back; their own fine-pitch pins get stubs too
RIP_FOR_STUBS = False


def polys(item, layer):
    buf = pcbnew.SHAPE_POLY_SET()
    item.TransformShapeToPolygon(buf, layer, 0, pcbnew.FromMM(0.005), pcbnew.ERROR_INSIDE)
    return outlines(buf)


def outlines(buf):
    out = []
    for i in range(buf.OutlineCount()):
        o = buf.Outline(i)
        out.append([(pcbnew.ToMM(o.CPoint(j).x), pcbnew.ToMM(o.CPoint(j).y)) for j in range(o.PointCount())])
    return out


def add_stubs(board, drc, classes):
    """Stub out from every open pad narrower than 0.35 mm that has no track."""
    open_pads = set()
    for u in json.load(open(drc))["unconnected_items"]:
        for it in u["items"]:
            m = re.match(r"Pad (\S+) \[.*\] of (\S+) on", it["description"])
            if m:
                open_pads.add((m.group(2), m.group(1)))
    tracks = [t for t in board.GetTracks()]
    pads = [p for p in board.GetPads()]
    zones = [z for z in board.Zones() if not z.GetIsRuleArea()
             and z.GetZoneName() not in POURS and not z.GetZoneName().startswith("MCU GND")]
    added, ripped, new = 0, set(), []
    fps = {fp.GetReference(): fp for fp in board.GetFootprints()}
    queue, seen = sorted(open_pads), set(open_pads)
    while queue:
        ref, num = queue.pop(0)
        fp = fps[ref]
        p = next((q for q in fp.Pads() if q.GetNumber() == num), None)
        if p is None or p.GetDrillSize().x > 0:
            continue
        bb = p.GetBoundingBox()
        w, h = bb.GetWidth(), bb.GetHeight()
        if min(w, h) > pcbnew.FromMM(0.35):
            continue
        if any(t.GetNetCode() == p.GetNetCode() and (p.HitTest(t.GetStart()) or p.HitTest(t.GetEnd()))
               for t in tracks if t.GetClass() == "PCB_TRACK"):
            continue
        layer = pcbnew.F_Cu if p.IsOnLayer(pcbnew.F_Cu) else pcbnew.B_Cu
        c = p.GetPosition()
        f = fp.GetPosition()
        if w > h:
            d = pcbnew.VECTOR2I(1 if c.x > f.x else -1, 0)
            length = w / 2 + pcbnew.FromMM(0.45)
        else:
            d = pcbnew.VECTOR2I(0, 1 if c.y > f.y else -1)
            length = h / 2 + pcbnew.FromMM(0.45)
        end = pcbnew.VECTOR2I(int(c.x + d.x * length), int(c.y + d.y * length))
        cls = classes.get(p.GetNet().GetNetClassName(), classes["Default"])
        width = pcbnew.FromMM(cls["track_width"])
        seg = pcbnew.SHAPE_SEGMENT(c, end, width)
        clr = pcbnew.FromMM(cls["clearance"])
        hit = False
        for q in pads:
            if q.GetNetCode() != p.GetNetCode() and q.IsOnLayer(layer) and \
                    seg.Collide(q.GetEffectiveShape(layer), clr):
                hit = True
                break
        # autorouted tracks in the way get ripped up and rerouted later
        in_way = set()
        if not hit:
            for t in tracks + new:
                if t.GetNetCode() != p.GetNetCode() and t.IsOnLayer(layer) and \
                        seg.Collide(t.GetEffectiveShape(layer), clr):
                    in_way.add(t.GetNetname())
            hit = bool(in_way & set(NO_RIP)) or (bool(in_way) and not RIP_FOR_STUBS)
        if not hit:
            for z in zones:
                if z.GetNetCode() != p.GetNetCode() and z.IsOnLayer(layer) and \
                        z.GetFilledPolysList(layer).Collide(seg, clr):
                    hit = True
                    break
        if hit:
            continue
        ripped |= in_way
        tracks = [t for t in tracks if t.GetNetname() not in ripped]
        # the ripped nets' fine-pitch pins need stubs of their own now
        for q in pads:
            key = (q.GetParentFootprint().GetReference(), q.GetNumber())
            if q.GetNetname() in in_way and key not in seen:
                seen.add(key)
                queue.append(key)
        t = pcbnew.PCB_TRACK(board)
        t.SetStart(c)
        t.SetEnd(end)
        t.SetWidth(width)
        t.SetLayer(layer)
        t.SetNet(p.GetNet())
        board.Add(t)
        new.append(t)
        added += 1
    return added, ripped


def main(src, drc, stubbed, dst):
    board = pcbnew.LoadBoard(src)
    pro = json.load(open(src.rsplit(".", 1)[0] + ".kicad_pro"))
    classes = {c["name"]: c for c in pro["net_settings"]["classes"]}
    n, ripped = add_stubs(board, drc, classes)
    print("stubs", n, "ripped", sorted(ripped))
    json.dump(sorted(ripped), open(dst + ".ripped", "w"))
    board.Save(stubbed)
    return stubbed


def dump(src, dst, ripped):
    board = pcbnew.LoadBoard(src)
    pro = json.load(open(src.rsplit(".", 1)[0] + ".kicad_pro"))
    classes = {c["name"]: c for c in pro["net_settings"]["classes"]}
    items = []
    for fp in board.GetFootprints():
        for p in fp.Pads():
            for layer, name in LAYERS.items():
                if p.IsOnLayer(layer) or (p.GetDrillSize().x > 0 and layer in (pcbnew.In1_Cu, pcbnew.In2_Cu)):
                    items.append({"net": p.GetNetCode(), "layer": name, "polys": polys(p, layer),
                                  "via": p.GetDrillSize().x > 0, "kind": "pad"})
    for t in board.GetTracks():
        if t.GetNetname() in ripped:
            continue
        if t.GetClass() == "PCB_VIA":
            for layer, name in LAYERS.items():
                items.append({"net": t.GetNetCode(), "layer": name, "polys": polys(t, layer), "via": True,
                              "kind": "via"})
        elif t.GetLayer() in LAYERS:
            items.append({"net": t.GetNetCode(), "layer": LAYERS[t.GetLayer()], "polys": polys(t, t.GetLayer()),
                          "via": False, "kind": "track"})
    for z in board.Zones():
        if z.GetIsRuleArea() or z.GetZoneName() in POURS or z.GetZoneName().startswith("MCU GND"):
            continue
        for layer, name in LAYERS.items():
            if z.IsOnLayer(layer):
                items.append({"net": z.GetNetCode(), "layer": name, "polys": outlines(z.Outline()), "via": False,
                              "kind": "zone"})
    nets = {}
    for code, n in board.GetNetsByNetcode().items():
        if code <= 0:
            continue
        c = classes.get(n.GetNetClassName(), classes["Default"])
        nets[code] = {"name": n.GetNetname(), "width": c["track_width"],
                      "via": c["via_diameter"], "drill": c["via_drill"], "clearance": c["clearance"]}
    out = {
        "centre": (150.0, 100.0),
        "outline": L.outline(),
        "holes": L.holes(),
        "hole_free": L.HOLE_COPPER_FREE,
        "items": items,
        "nets": nets,
        "ripped": sorted(ripped),
    }
    json.dump(out, open(dst, "w"))
    print("items", len(items))
    # the ripped nets' tracks go now, at the end: the Python wrappers of a
    # board go stale after removals
    for t in [t for t in board.GetTracks() if t.GetNetname() in ripped]:
        board.Remove(t)
    board.Save(src)


if __name__ == "__main__":
    # two runs: the Python wrappers go stale after removing tracks
    if sys.argv[1] == "dump":
        dump(sys.argv[2], sys.argv[3], json.load(open(sys.argv[3] + ".ripped")))
    else:
        main(*sys.argv[1:5])
