#!/bin/sh
# render_route.sh <pcb> <out.png> [layers]: routed board preview
set -e
T=$(mktemp -d)
L=${3:-F.Cu,B.Cu,Edge.Cuts,F.Fab}
kicad-cli pcb export svg --layers $L --page-size-mode 2 --exclude-drawing-sheet -o $T/b.svg "$1" >/dev/null
python3 -c "
import cairosvg
cairosvg.svg2png(url='$T/b.svg', write_to='$2', output_width=1800, background_color='white')"
