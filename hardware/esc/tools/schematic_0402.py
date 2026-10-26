"""One-shot schematic change for the edge-connector layout. Run from hardware/esc
with a plain Python 3 after exporting the netlist (see tools/build.sh):

    python3 tools/schematic_0402.py build/netlist.json

* The passives round each channel's MCU (decoupling, NRST, BOOT0, VDDA,
  BEMF dividers, DShot series resistor) go from 0603 to 0402, so the MCU has
  room for its escape ring nearer the board centre. Gate resistors, bootstrap
  caps and the driver supply cap stay 0603. ESC.kicad_sch is one sheet used
  four times, so a footprint change there covers all four channels.
* J1 becomes the side-entry JST SH (SM08B-SRSS-TB), so the FC lead leaves the
  board edge sideways instead of standing up into the stack.
"""
import json
import re
import sys

RES_0402 = {"10k": "RC0402FR-0710KL", "1k": "RC0402FR-071KL", "220R": "RC0402FR-07220RL",
            "10R": "RC0402FR-0710RL"}
CAP_0402 = {"100nF": "CL05B104KB5NNNC"}   # 100 nF 50 V X7R 0402


def logic_refs(netlist):
    """ESC1 references of the logic passives (the other channels share the symbols)."""
    nl = json.load(open(netlist))
    comps = {c["ref"]: c for c in nl["components"]}
    nets = nl["nets"]
    s = "/ESC1/"

    def on(net, prefix):
        return [r for r, _ in nets.get(net, []) if r.startswith(prefix) and comps[r]["sheet"] == s]

    mcu = next(r for r, c in comps.items() if c["sheet"] == s and c["value"] == "AT32F421G8U7")
    refs = {on("+3V3", "C")[0], on(f"{s}vdda", "R")[0], on(f"{s}vdda", "C")[0],
            on(f"Net-({mcu}-NRST)", "R")[0], on(f"Net-({mcu}-NRST)", "C")[0],
            on(f"Net-({mcu}-BOOT0)", "R")[0], on(f"Net-({mcu}-PB4)", "R")[0]}
    for ph in "ABC":
        refs.update(on(f"{s}Motor{ph}", "R"))
        refs.update(on(f"{s}FB{ph}", "R"))
    return refs


def prop(block, name, value):
    return re.sub(rf'\(property "{name}" "[^"]*"', f'(property "{name}" "{value}"', block, count=1)


def to_0402(path, refs):
    t = open(path).read()
    parts = t.split("\n(symbol (lib_id ")
    n = 0
    for i, b in enumerate(parts[1:], 1):
        mine = re.findall(r'\(reference "([^"]+)"\)', b)
        if not refs.intersection(mine):
            continue
        val = re.search(r'\(property "Value" "([^"]*)"', b).group(1)
        if 'R_0603' in b and val in RES_0402:
            b = prop(b, "Footprint", "Resistor_SMD:R_0402_1005Metric")
            b = prop(b, "MPN", RES_0402[val])
        elif 'C_0603' in b and val in CAP_0402:
            b = prop(b, "Footprint", "Capacitor_SMD:C_0402_1005Metric")
            b = prop(b, "MPN", CAP_0402[val])
        else:
            raise SystemExit(f"don't know a 0402 part for {mine} {val}")
        parts[i] = b
        n += 1
    open(path, "w").write("\n(symbol (lib_id ".join(parts))
    print(path, n, "symbols to 0402")


def side_entry_j1(path):
    t = open(path).read()
    t = t.replace('"BM08B-SRSS-TB"', '"SM08B-SRSS-TB"')
    t = t.replace('"drone-esc:JST_SH_BM08B-SRSS-TB_Vertical"', '"drone-esc:JST_SH_SM08B-SRSS-TB_Horizontal"')
    t = t.replace('"BM08B-SRSS-TB(LF)(SN)"', '"SM08B-SRSS-TB(LF)(SN)"')
    # LCSC C160407 is SM08B-SRSS-TB(LF)(SN); the old C160390 was a 4-pin BM04B
    t = t.replace('(property "LCSC" "C160390"', '(property "LCSC" "C160407"')
    open(path, "w").write(t)
    print(path, "J1 -> SM08B-SRSS-TB")


if __name__ == "__main__":
    to_0402("ESC.kicad_sch", logic_refs(sys.argv[1]))
    side_entry_j1("drone-esc.kicad_sch")
