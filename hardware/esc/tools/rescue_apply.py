"""Apply rescue_route.py's result: take out the tracks and vias of the nets it
ripped up, add its new tracks and vias, refill and save.
Run with KiCad's Python.

    python3 tools/rescue_apply.py rip build/stubbed.kicad_pcb build/rescue_out.json build/ripped.kicad_pcb
    python3 tools/rescue_apply.py build/ripped.kicad_pcb build/rescue_out.json drone-esc.kicad_pcb
"""
import json
import sys

import pcbnew

sys.path.insert(0, __file__.rsplit("/", 1)[0])
from finish_pcb import stackup_2oz, stitch_islands  # noqa: E402

MM = pcbnew.FromMM
LAYERS = {"F": pcbnew.F_Cu, "In1": pcbnew.In1_Cu, "In2": pcbnew.In2_Cu, "B": pcbnew.B_Cu}


def rip(src, items, out):
    """First pass: take out the ripped nets' tracks and vias, and save. The
    Python wrappers of a board go stale after removals, so adding is a
    second run."""
    board = pcbnew.LoadBoard(src)
    nets = {it["net"] for it in json.load(open(items)) if it["type"] == "rip"}
    for t in [t for t in board.GetTracks() if t.GetNetname() in nets]:
        board.Remove(t)
    board.Save(out)


def main(src, items, out):
    board = pcbnew.LoadBoard(src)
    items = json.load(open(items))
    for it in items:
        net = board.FindNet(it["net"])
        if it["type"] == "rip":
            continue
        if it["type"] == "track":
            t = pcbnew.PCB_TRACK(board)
            t.SetStart(pcbnew.VECTOR2I(MM(it["x1"]), MM(it["y1"])))
            t.SetEnd(pcbnew.VECTOR2I(MM(it["x2"]), MM(it["y2"])))
            t.SetWidth(MM(it["w"]))
            t.SetLayer(LAYERS[it["layer"]])
        else:
            t = pcbnew.PCB_VIA(board)
            t.SetPosition(pcbnew.VECTOR2I(MM(it["x"]), MM(it["y"])))
            t.SetWidth(MM(it["d"]))
            t.SetDrill(MM(it["drill"]))
        t.SetNet(net)
        board.Add(t)
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    print("island vias", stitch_islands(board))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(out)
    stackup_2oz(out)
    print("saved", out)


if __name__ == "__main__":
    if sys.argv[1] == "rip":
        rip(*sys.argv[2:5])
    else:
        main(*sys.argv[1:4])
