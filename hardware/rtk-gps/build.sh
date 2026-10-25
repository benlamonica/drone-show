#!/bin/bash
# full rebuild: schematic -> netlist -> placed board -> pre-route + DSN -> freerouting -> SES import
set -e
cd /home/claude/work
python3 gen/gen_symbols.py rtk-gps/lib/rtk_gps.kicad_sym
python3 gen/gen_sch.py libs rtk-gps
python3 gen/gen_pro.py rtk-gps
./kr.sh kicad-cli sch export netlist -o rtk-gps/rtk_gps.net rtk-gps/rtk_gps.kicad_sch >/dev/null
./kr.sh python3 gen/gen_pcb.py rtk-gps rtk-gps/rtk_gps.net | tail -1
python3 gen/gen_pro.py rtk-gps
./kr.sh python3 gen/route_pre.py rtk-gps/rtk_gps.kicad_pcb rtk-gps/rtk_gps.dsn | grep -v "^$"
cd rtk-gps
timeout 1200 xvfb-run -a java -jar ../../dl/freerouting.jar -de rtk_gps.dsn -do rtk_gps.ses -mp ${PASSES:-40} -oit 0.5 -dct 0 > ../fr.log 2>&1
grep -E "completed|unrouted|incomplete" ../fr.log | tail -3
cd ..
./kr.sh python3 gen/route_import.py rtk-gps/rtk_gps.kicad_pcb rtk-gps/rtk_gps.ses
./kr.sh python3 gen/route_post.py rtk-gps/rtk_gps.kicad_pcb rtk-gps/rtk_gps.ses | grep -v "^$"
./kr.sh bash -c 'cd rtk-gps && kicad-cli pcb drc --severity-all -o /w/drc.rpt rtk_gps.kicad_pcb >/dev/null'
grep -E "^\[" drc.rpt | cut -c1-90 | sort | uniq -c
