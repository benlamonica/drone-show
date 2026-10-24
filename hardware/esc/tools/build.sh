#!/bin/sh
# Rebuild the board from the schematic. Needs kicad-cli and KiCad's Python
# (on macOS: /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli and the
# bundled python3), java 21 and freerouting.jar for the routing step.
set -e
cd "$(dirname "$0")/.."
mkdir -p build
kicad-cli sch export netlist --format kicadsexpr -o build/drone-esc.net drone-esc.kicad_sch
