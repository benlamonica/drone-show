#!/bin/sh
# dev helper: rebuild placement and render PNGs
D="docker run --rm -u root -v /home/claude/drone-show:/w -w /w/hardware/esc kicad/kicad:10.0"
$D python3 tools/build_pcb.py build/netlist.json build/placed.kicad_pcb 2>&1 | grep -v Font | tail -20
$D sh tools/render.sh build/placed.kicad_pcb build/render
python3 -c "
import cairosvg
for n in ('top','bottom'):
    cairosvg.svg2png(url='build/render/%s.svg'%n, write_to='build/render/%s.png'%n, output_width=1400, background_color='white')
"
