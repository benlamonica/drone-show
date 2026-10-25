#!/bin/bash
# re-run only the SES import + planes + DRC (no autorouting)
cd /home/claude/work
./kr.sh python3 gen/gen_pcb.py rtk-gps rtk-gps/rtk_gps.net | tail -1
python3 gen/gen_pro.py rtk-gps
./kr.sh python3 gen/route_pre.py rtk-gps/rtk_gps.kicad_pcb /tmp/x.dsn >/dev/null
./kr.sh python3 gen/route_import.py rtk-gps/rtk_gps.kicad_pcb rtk-gps/rtk_gps.ses
./kr.sh python3 gen/route_post.py rtk-gps/rtk_gps.kicad_pcb rtk-gps/rtk_gps.ses | grep -v "^$"
./kr.sh bash -c 'cd rtk-gps && kicad-cli pcb drc --severity-all -o /w/drc.rpt rtk_gps.kicad_pcb >/dev/null'
grep -E "^\[" drc.rpt | cut -c1-90 | sort | uniq -c
