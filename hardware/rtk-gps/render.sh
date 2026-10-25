#!/bin/bash
# render.sh <png> [layers]
L=${2:-F.Cu,B.Cu,F.Silkscreen,F.Courtyard,Edge.Cuts,F.Fab}
./kr.sh bash -c "cd rtk-gps && kicad-cli pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet -l $L -o /w/tmp.svg rtk_gps.kicad_pcb >/dev/null"
python3 -c "import cairosvg;cairosvg.svg2png(url='tmp.svg',write_to='$1',output_width=1100,background_color='white')"
