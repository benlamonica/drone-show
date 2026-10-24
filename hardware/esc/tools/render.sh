#!/bin/sh
# render.sh board.kicad_pcb outdir  -> outdir/top.svg, outdir/bottom.svg
set -e
mkdir -p "$2"
kicad-cli pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet \
  -l F.Cu,F.Paste,F.SilkS,Edge.Cuts,F.CrtYd,F.Fab -o "$2/top.svg" "$1" >/dev/null
kicad-cli pcb export svg --mode-single --fit-page-to-board --exclude-drawing-sheet --mirror \
  -l B.Cu,B.Paste,B.SilkS,Edge.Cuts,B.CrtYd,B.Fab -o "$2/bottom.svg" "$1" >/dev/null
