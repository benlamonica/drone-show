"""Import the Freerouting result, fill the pours and write the final board.

    python3 tools/finish_pcb.py build/fanned.kicad_pcb build/route.ses drone-esc.kicad_pcb
"""
import re
import sys

import pcbnew

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


def main(src, ses, out):
    board = pcbnew.LoadBoard(src)
    if not pcbnew.ImportSpecctraSES(board, ses):
        raise SystemExit("SES import failed")
    tidy_silk(board)
    # the SES import gives router vias the default 0.4 mm drill; 0.6/0.3 is what was routed
    for t in board.GetTracks():
        if t.GetClass() == "PCB_VIA" and t.GetWidth(pcbnew.F_Cu) == MM(0.6):
            t.SetDrill(MM(0.3))
    # Freerouting necks tracks down to the 0.18 mm MCU pads; PCBWay's minimum is 0.2
    for t in board.GetTracks():
        if t.GetClass() == "PCB_TRACK" and t.GetWidth() < MM(0.2):
            t.SetWidth(MM(0.2))
    pcbnew.ZONE_FILLER(board).Fill(board.Zones())
    board.Save(out)
    stackup_2oz(out)
    print("saved", out)


if __name__ == "__main__":
    main(*sys.argv[1:4])
