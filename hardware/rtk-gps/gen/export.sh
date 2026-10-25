#!/bin/bash
# Fabrication outputs for PCBWay: Gerbers + drill (zipped), pick-and-place, BOM, schematic PDF, STEP, renders.
set -e
cd /home/claude/work
rm -rf rtk-gps/fab && mkdir -p rtk-gps/fab/gerbers && chmod -R a+rw rtk-gps
./kr.sh bash -c '
cd rtk-gps
kicad-cli pcb export gerbers --no-protel-ext --subtract-soldermask -l F.Cu,B.Cu,F.Paste,F.Silkscreen,B.Silkscreen,F.Mask,B.Mask,Edge.Cuts -o fab/gerbers/ rtk_gps.kicad_pcb
kicad-cli pcb export drill --format excellon --excellon-separate-th --generate-map --map-format gerberx2 -o fab/gerbers/ rtk_gps.kicad_pcb
kicad-cli pcb export pos --side front --format csv --units mm --exclude-dnp --use-drill-file-origin -o fab/rtk_gps-top-pos.csv rtk_gps.kicad_pcb
kicad-cli sch export bom --fields "Reference,Value,Footprint,MPN,Manufacturer,Mouser,\${QUANTITY}" --labels "Designator,Value,Footprint,MPN,Manufacturer,Mouser,Qty" --group-by "Value,Footprint,MPN" --exclude-dnp -o fab/rtk_gps-bom.csv rtk_gps.kicad_sch
kicad-cli sch export pdf -o fab/rtk_gps-schematic.pdf rtk_gps.kicad_sch
kicad-cli pcb export step --subst-models --force -o fab/rtk_gps.step rtk_gps.kicad_pcb >/dev/null 2>&1 || true
kicad-cli pcb drc --severity-all --schematic-parity -o fab/drc-report.txt rtk_gps.kicad_pcb >/dev/null || true
kicad-cli sch erc --severity-all -o fab/erc-report.txt rtk_gps.kicad_sch >/dev/null || true
kicad-cli pcb render --side top --width 1600 --height 1600 --quality high -o fab/render-top.png rtk_gps.kicad_pcb >/dev/null 2>&1 || true
kicad-cli pcb render --side bottom --width 1600 --height 1600 --quality high -o fab/render-bottom.png rtk_gps.kicad_pcb >/dev/null 2>&1 || true
'
(cd rtk-gps/fab/gerbers && zip -q ../rtk_gps-gerbers-pcbway.zip *)
ls -la rtk-gps/fab
