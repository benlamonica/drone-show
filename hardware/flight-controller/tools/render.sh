#!/bin/sh
# render.sh <pcb> <out.png>: top copper + silk + outline preview
set -e
T=$(mktemp -d)
kicad-cli pcb export svg --layers F.Cu,B.Cu,F.SilkS,Edge.Cuts,F.CrtYd --page-size-mode 2 --exclude-drawing-sheet -o $T/b.svg "$1" >/dev/null
python3 -c "
import cairosvg,sys
from PIL import Image
cairosvg.svg2png(url='$T/b.svg', write_to='$T/b.png', output_width=1600, background_color='white')
" 
cp $T/b.png "$2"
