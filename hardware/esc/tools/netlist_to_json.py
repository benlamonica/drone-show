"""Turn the KiCad netlist into JSON that the pcbnew build script can read.

KiCad's bundled Python has no s-expression parser, so this runs on the host:
    kicad-cli sch export netlist --format kicadsexpr -o build/drone-esc.net drone-esc.kicad_sch
    python3 tools/netlist_to_json.py build/drone-esc.net build/netlist.json
"""
import json
import sys

import sexpdata as s


def items(x, name):
    return [e for e in x if isinstance(e, list) and e and e[0] == s.Symbol(name)]


def one(x, name, default=None):
    r = items(x, name)
    return r[0][1] if r else default


def main(src, dst):
    d = s.loads(open(src).read())
    comps = []
    for c in items(items(d, "components")[0], "comp"):
        fields = {}
        for f in items(c, "fields"):
            for fld in items(f, "field"):
                name = one(fld, "name")
                fields[name] = fld[2] if len(fld) > 2 and isinstance(fld[2], str) else ""
        sp = items(c, "sheetpath")[0]
        comps.append({
            "ref": one(c, "ref"),
            "value": one(c, "value"),
            "footprint": one(c, "footprint", ""),
            "sheet": one(sp, "names"),
            "path": one(sp, "tstamps") + one(c, "tstamps"),
            "fields": fields,
        })
    nets = {}
    for n in items(items(d, "nets")[0], "net"):
        nets[one(n, "name")] = [[one(x, "ref"), one(x, "pin")] for x in items(n, "node")]
    json.dump({"components": comps, "nets": nets}, open(dst, "w"), indent=1)


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
