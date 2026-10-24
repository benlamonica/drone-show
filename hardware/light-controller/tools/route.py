"""Autoroute the board with Freerouting and bring the result back into KiCad.

    python3 tools/route.py            # export DSN, run Freerouting, import, fill, save
    python3 tools/route.py --import   # only import an existing /tmp/lc.ses

KiCad 7's ImportSpecctraSES needs the GUI, so the session file is read here: every
wire becomes a track and every via a via, on the nets Freerouting reports. DSN units
are um with the y axis flipped.
"""
import os
import re
import subprocess
import sys

import pcbnew

sys.path.insert(0, os.path.dirname(__file__))
from gen_pcb import sexp, PCB  # noqa: E402

DSN, SES = "/tmp/lc.dsn", "/tmp/lc.ses"
FREEROUTING = os.environ.get("FREEROUTING", "/opt/freerouting.jar")


def export():
    pcbnew.ExportSpecctraDSN(pcbnew.LoadBoard(PCB), DSN)


def run(passes=25):
    subprocess.run(["java", "-Xmx8g", "-jar", FREEROUTING, "-de", DSN, "-do", SES, "-mp", str(passes),
                    "-mt", "4", "--gui.enabled=false"], check=True)


def import_ses():
    b = pcbnew.LoadBoard(PCB)
    tree = sexp(open(SES).read())
    routes = next(i for i in tree if isinstance(i, list) and i[0] == "routes")
    res = next(i for i in routes if isinstance(i, list) and i[0] == "resolution")
    scale = 1000.0 / float(res[2]) if res[1] == "um" else None     # SES units -> nm
    padstacks = {}
    lib = next((i for i in routes if isinstance(i, list) and i[0] == "library_out"), [])
    for ps in lib[1:]:
        if isinstance(ps, list) and ps[0] == "padstack":
            m = re.search(r"(\d+):(\d+)_um", ps[1])
            padstacks[ps[1]] = (int(m.group(1)) * 1000, int(m.group(2)) * 1000)
    net_out = next(i for i in routes if isinstance(i, list) and i[0] == "network_out")
    layers = {"F.Cu": pcbnew.F_Cu, "B.Cu": pcbnew.B_Cu}
    nt, nv = 0, 0
    for net in net_out[1:]:
        ni = b.FindNet(net[1])
        for item in net[2:]:
            if item[0] == "wire":
                path = item[1]
                layer, width = layers[path[1]], int(float(path[2]) * scale)
                pts = [(int(float(path[i]) * scale), -int(float(path[i + 1]) * scale)) for i in range(3, len(path) - 1, 2)]
                for a, c in zip(pts, pts[1:]):
                    t = pcbnew.PCB_TRACK(b)
                    t.SetStart(pcbnew.VECTOR2I(*a))
                    t.SetEnd(pcbnew.VECTOR2I(*c))
                    t.SetWidth(width)
                    t.SetLayer(layer)
                    t.SetNet(ni)
                    b.Add(t)
                    nt += 1
            elif item[0] == "via":
                d, drill = padstacks[item[1]]
                v = pcbnew.PCB_VIA(b)
                v.SetPosition(pcbnew.VECTOR2I(int(float(item[2]) * scale), -int(float(item[3]) * scale)))
                v.SetWidth(d)
                v.SetDrill(drill)
                v.SetNet(ni)
                b.Add(v)
                nv += 1
    pcbnew.ZONE_FILLER(b).Fill(b.Zones())
    pcbnew.SaveBoard(PCB, b)
    print(f"imported {nt} track segments, {nv} vias")


if __name__ == "__main__":
    if "--import" not in sys.argv:
        export()
        run()
    import_ses()
