#!/bin/bash
./kr.sh python3 gen/gen_pcb.py rtk-gps rtk-gps/rtk_gps.net 2>&1 | tail -1
./kr.sh bash -c 'cd rtk-gps && kicad-cli pcb drc --severity-all -o /w/drc.rpt rtk_gps.kicad_pcb >/dev/null'
grep -E "^\[" drc.rpt | grep -v "unconnected_items\|silk_\|lib_footprint" | sort | uniq -c
python3 - <<'PY'
import re
t=open('drc.rpt').read()
for blk in t.split('\n[')[1:]:
    if blk.startswith('courtyards_overlap') or blk.startswith('shorting') or blk.startswith('clearance') or blk.startswith('items_not') or blk.startswith('solder_mask'):
        refs=re.findall(r'(?:Footprint|of) (\w+)',blk)
        print(blk.split(']')[0], refs[:4])
PY
