#!/bin/sh
# Fab outputs for PCBWay from drone-esc.kicad_pcb, into fab/:
#   gerbers + Excellon drill (zipped), BOM and pick-and-place CSV, renders.
set -e
cd "$(dirname "$0")/.."
rm -rf fab && mkdir -p fab/gerber
kicad-cli pcb export gerbers --no-protel-ext --subtract-soldermask \
  -l F.Cu,In1.Cu,In2.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts \
  -o fab/gerber/ drone-esc.kicad_pcb
kicad-cli pcb export drill --format excellon --excellon-separate-th \
  --generate-map --map-format gerberx2 -o fab/gerber/ drone-esc.kicad_pcb
(cd fab/gerber && python3 -c "import zipfile,os; z=zipfile.ZipFile('../drone-esc-gerbers.zip','w',zipfile.ZIP_DEFLATED); [z.write(f) for f in sorted(os.listdir('.'))]")
kicad-cli pcb export pos --format csv --units mm --side both --exclude-dnp \
  -o fab/pos-raw.csv drone-esc.kicad_pcb
kicad-cli sch export bom --fields 'Reference,${QUANTITY},Value,Footprint,Manufacturer,MPN,LCSC,Mouser' \
  --labels 'Designator,Qty,Value,Footprint,Manufacturer,MPN,LCSC,Mouser' \
  --group-by 'Value,Footprint,MPN' --ref-range-delimiter '' --exclude-dnp \
  -o fab/bom-raw.csv drone-esc.kicad_sch
python3 tools/fab_files.py fab
kicad-cli pcb render --side top --quality high -w 1600 -h 1600 -o fab/render-top.png drone-esc.kicad_pcb
kicad-cli pcb render --side bottom --quality high -w 1600 -h 1600 -o fab/render-bottom.png drone-esc.kicad_pcb
echo "fab outputs in fab/"
