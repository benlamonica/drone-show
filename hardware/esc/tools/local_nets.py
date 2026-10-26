"""Pick the short channel-local connections out of a DRC report, for routing
them before Freerouting. Plain Python 3.

    python3 tools/local_nets.py build/drc_fanned.json build/drc_local.json

The MCU's own small nets (back-EMF dividers, VDDA, NRST, BOOT0, DShot,
SWD, the 3V3 feed to its parts on top) and the gate driver's bootstrap
pins are a few millimetres long but packed round 0.4 and 0.5 mm pitch QFN
pins. Freerouting does the long nets first and walls these in, so
rescue_route.py routes them first on the near-empty board and route_prep.py
hands them to Freerouting as protected wiring.
"""
import json
import re
import sys

LOCAL = re.compile(r"^(\+3V3|/ESC\d/(FB[ABC]|FBCOMMON|vdda)|Net-\(U\d+-(NRST|BOOT0|PB4|PA13|PA14|VB\d)\))$")

d = json.load(open(sys.argv[1]))
keep = []
for u in d["unconnected_items"]:
    names = {m.group(1) for it in u["items"] for m in [re.search(r"\[(.+?)\]", it["description"])] if m}
    if names and all(LOCAL.match(n) for n in names):
        keep.append(u)
d["unconnected_items"] = keep
json.dump(d, open(sys.argv[2], "w"))
print(len(keep), "local connections")
