"""Render top and bottom views of a board to PNG (host side, needs cairosvg)."""
import os
import subprocess
import sys

import cairosvg

pcb, outdir = sys.argv[1], sys.argv[2]
os.makedirs(outdir, exist_ok=True)
views = {
    "top": "F.Cu,F.SilkS,F.Mask,Edge.Cuts,F.CrtYd",
    "bottom": "B.Cu,B.SilkS,Edge.Cuts,B.CrtYd",
}
for name, layers in views.items():
    svg = os.path.join(outdir, f"{name}.svg")
    cmd = ["kicad-cli", "pcb", "export", "svg", "--mode-single", "--fit-page-to-board",
           "--exclude-drawing-sheet", "-l", layers, "-o", svg, pcb]
    if name == "bottom":
        cmd.insert(4, "--mirror")
    subprocess.run(cmd, check=True, capture_output=True)
