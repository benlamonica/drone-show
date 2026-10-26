#!/bin/sh
# Rebuild the board from the schematic. Needs kicad-cli and KiCad 10's Python
# (KIPY; on macOS the app's bundled python3 is picked up by itself), a plain
# Python 3 with sexpdata, numpy, scipy and Pillow (PYTHON), java and
# freerouting 1.9.0 for the routing step:
#
#   FREEROUTING=/path/to/freerouting-1.9.0.jar PYTHON=venv/bin/python sh tools/build.sh
#
# On a headless Linux box run it under xvfb-run (freerouting 1.9 opens a window).
# Routing takes up to an hour. GND and +BATT go through the inner planes, so
# the router skips the Power class.
# Steps: netlist -> footprints -> placement -> fanout -> local nets (maze
# router) -> DSN -> autoroute ->
# import + pour fill -> clean-up router -> drone-esc.kicad_pcb.
# tools/export.sh then makes the fab files.
set -e
cd "$(dirname "$0")/.."
FREEROUTING=${FREEROUTING:-freerouting-1.9.0.jar}
MAC=/Applications/KiCad/KiCad.app/Contents
if [ -d "$MAC" ]; then
  PATH="$MAC/MacOS:$PATH"
  KIPY=${KIPY:-$MAC/Frameworks/Python.framework/Versions/Current/bin/python3}
  export KICAD_FOOTPRINTS=${KICAD_FOOTPRINTS:-$MAC/SharedSupport/footprints}
fi
KIPY=${KIPY:-python3}
PYTHON=${PYTHON:-python3}
mkdir -p build
kicad-cli sch export netlist --format kicadsexpr -o build/drone-esc.net drone-esc.kicad_sch
$PYTHON tools/netlist_to_json.py build/drone-esc.net build/netlist.json
$KIPY tools/project_settings.py drone-esc.kicad_pro
$KIPY tools/make_footprints.py
$KIPY tools/build_pcb.py build/netlist.json build/placed.kicad_pcb
[ "$1" = "place" ] && exit 0
for b in placed fanned pstub prerouted; do
  cp drone-esc.kicad_pro build/$b.kicad_pro
  cp drone-esc.kicad_dru build/$b.kicad_dru
done
$KIPY tools/route_prep.py fanout build/placed.kicad_pcb build/fanned.kicad_pcb
# the short channel-local nets first, with the maze router, on the empty board
kicad-cli pcb drc --format json -o build/drc_fanned.json build/fanned.kicad_pcb || true
$PYTHON tools/local_nets.py build/drc_fanned.json build/drc_local.json
$KIPY tools/rescue_export.py build/fanned.kicad_pcb build/drc_local.json build/pstub.kicad_pcb build/pre_in.json
$KIPY tools/rescue_export.py dump build/pstub.kicad_pcb build/pre_in.json
(cd build && $PYTHON ../tools/rescue_route.py pre_in.json drc_local.json pre_out.json)
$KIPY tools/rescue_apply.py build/pstub.kicad_pcb build/pre_out.json build/prerouted.kicad_pcb
# everything else with Freerouting, the local nets kept as they are
$KIPY tools/route_prep.py resume build/prerouted.kicad_pcb build/route.dsn
(cd build && java -jar "$FREEROUTING" -de route.dsn -do route.ses -mp 40 -mt 0 -inc Power)
$KIPY tools/finish_pcb.py build/prerouted.kicad_pcb build/route.ses build/routed.kicad_pcb
# clean-up pass for what Freerouting left open
for b in routed stubbed ripped; do
  cp drone-esc.kicad_pro build/$b.kicad_pro
  cp drone-esc.kicad_dru build/$b.kicad_dru
done
kicad-cli pcb drc --format json -o build/drc0.json build/routed.kicad_pcb || true
$KIPY tools/rescue_export.py build/routed.kicad_pcb build/drc0.json build/stubbed.kicad_pcb build/rescue_in.json
$KIPY tools/rescue_export.py dump build/stubbed.kicad_pcb build/rescue_in.json
(cd build && $PYTHON ../tools/rescue_route.py rescue_in.json drc0.json rescue_out.json)
$KIPY tools/rescue_apply.py rip build/stubbed.kicad_pcb build/rescue_out.json build/ripped.kicad_pcb
$KIPY tools/rescue_apply.py build/ripped.kicad_pcb build/rescue_out.json drone-esc.kicad_pcb
kicad-cli pcb drc --schematic-parity --severity-error -o build/drc.rpt drone-esc.kicad_pcb || true
echo "board written to drone-esc.kicad_pcb, DRC report in build/drc.rpt"
