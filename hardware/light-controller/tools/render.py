"""PNG previews of the board (top and bottom) via kicad-cli SVG export."""
import os
import subprocess
import sys

import cairosvg

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PCB = os.path.join(ROOT, "light-controller.kicad_pcb")
OUT = os.path.join(ROOT, "images")

VIEWS = {
    "top": "F.Cu,F.SilkS,Edge.Cuts,User.1",
    "bottom": "B.Cu,B.SilkS,Edge.Cuts,User.1",
    "top_copper": "F.Cu,Edge.Cuts",
    "bottom_copper": "B.Cu,Edge.Cuts",
}


def main(views=None, scale=12):
    os.makedirs(OUT, exist_ok=True)
    for name in views or VIEWS:
        svg = os.path.join("/tmp", f"lc_{name}.svg")
        args = ["kicad-cli", "pcb", "export", "svg", "--layers", VIEWS[name], "--exclude-drawing-sheet",
                "--page-size-mode", "2", "-o", svg, PCB]
        if name.startswith("bottom"):
            args.insert(-1, "--mirror")
        subprocess.run(args, check=True, capture_output=True)
        png = os.path.join(OUT, f"{name}.png")
        cairosvg.svg2png(url=svg, write_to=png, scale=scale, background_color="white")
        print(png)


if __name__ == "__main__":
    main(sys.argv[1:] or None)
