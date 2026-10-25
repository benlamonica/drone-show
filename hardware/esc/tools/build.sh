#!/bin/sh
# Rebuild the board from the schematic. Needs kicad-cli and KiCad 10's Python
# (on macOS: /Applications/KiCad/KiCad.app/Contents/MacOS/kicad-cli and the
# bundled python3), java and freerouting 1.9.0 for the routing step:
#
#   FREEROUTING=/path/to/freerouting-1.9.0.jar sh tools/build.sh
#
# On a headless Linux box run it under xvfb-run (freerouting 1.9 opens a window).
# Routing takes up to an hour. GND and +BATT go through the inner planes, so
# the router skips the Power class.
# Steps: netlist -> footprints -> placement -> fanout + DSN -> autoroute ->
# import + pour fill -> clean-up router -> drone-esc.kicad_pcb. The
# clean-up router needs a Python 3 with numpy, scipy and Pillow (PYTHON=...). tools/export.sh then makes the
# fab files.
set -e
cd "$(dirname "$0")/.."
FREEROUTING=${FREEROUTING:-freerouting-1.9.0.jar}
mkdir -p build
kicad-cli sch export netlist --format kicadsexpr -o build/drone-esc.net drone-esc.kicad_sch
python3 tools/netlist_to_json.py build/drone-esc.net build/netlist.json
python3 tools/project_settings.py drone-esc.kicad_pro
python3 tools/make_footprints.py
python3 tools/build_pcb.py build/netlist.json build/placed.kicad_pcb
for b in placed fanned; do
  cp drone-esc.kicad_pro build/$b.kicad_pro
  cp drone-esc.kicad_dru build/$b.kicad_dru
done
python3 tools/route_prep.py signals build/placed.kicad_pcb build/fanned.kicad_pcb build/route.dsn
(cd build && java -jar "$FREEROUTING" -de route.dsn -do route.ses -mp 40 -mt 0 -inc Power)
python3 tools/finish_pcb.py build/fanned.kicad_pcb build/route.ses build/routed.kicad_pcb
# clean-up pass for what Freerouting left open
for b in routed stubbed ripped; do
  cp drone-esc.kicad_pro build/$b.kicad_pro
  cp drone-esc.kicad_dru build/$b.kicad_dru
done
kicad-cli pcb drc --format json -o build/drc0.json build/routed.kicad_pcb || true
python3 tools/rescue_export.py build/routed.kicad_pcb build/drc0.json build/stubbed.kicad_pcb build/rescue_in.json
python3 tools/rescue_export.py dump build/stubbed.kicad_pcb build/rescue_in.json
(cd build && ${PYTHON:-python3} ../tools/rescue_route.py rescue_in.json drc0.json rescue_out.json)
python3 tools/rescue_apply.py rip build/stubbed.kicad_pcb build/rescue_out.json build/ripped.kicad_pcb
python3 tools/rescue_apply.py build/ripped.kicad_pcb build/rescue_out.json drone-esc.kicad_pcb
kicad-cli pcb drc --schematic-parity --severity-error -o build/drc.rpt drone-esc.kicad_pcb || true
echo "board written to drone-esc.kicad_pcb, DRC report in build/drc.rpt"
