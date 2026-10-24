"""Fabrication outputs for PCBWay: Gerbers + drill (zipped), pick-and-place,
and an assembly BOM in PCBWay's column layout. Writes into ../fab/.

    python3 make_fab.py
"""

import csv
import os
import shutil
import subprocess
import zipfile
from collections import OrderedDict

import design

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PCB = os.path.join(ROOT, "show-fc.kicad_pcb")
FAB = os.path.join(ROOT, "fab")
NAME = "show-fc-revA"

LAYERS = "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts"


def run(*args):
    subprocess.run(["kicad-cli", *args], check=True, stdout=subprocess.DEVNULL)


def gerbers():
    g = os.path.join(FAB, "gerber")
    shutil.rmtree(g, ignore_errors=True)
    os.makedirs(g)
    run("pcb", "export", "gerbers", "--layers", LAYERS, "--subtract-soldermask",
        "--no-x2", "--use-drill-file-origin", "-o", g + "/", PCB)
    run("pcb", "export", "drill", "--format", "excellon", "--excellon-separate-th",
        "--excellon-units", "mm", "--generate-map", "--map-format", "gerberx2", "-o", g + "/", PCB)
    z = os.path.join(FAB, f"{NAME}-gerbers.zip")
    with zipfile.ZipFile(z, "w", zipfile.ZIP_DEFLATED) as f:
        for n in sorted(os.listdir(g)):
            f.write(os.path.join(g, n), n)
    return z


def placement():
    out = os.path.join(FAB, f"{NAME}-cpl.csv")
    run("pcb", "export", "pos", "--format", "csv", "--units", "mm", "--side", "front",
        "--exclude-dnp", "-o", out, PCB)
    return out


def bom():
    out = os.path.join(FAB, f"{NAME}-bom.csv")
    groups = OrderedDict()
    for p in design.PARTS:
        if p.kind in ("HOLE", "TP"):
            continue
        key = (p.value, p.footprint, p.mpn, p.mfr)
        groups.setdefault(key, []).append(p.ref)
    with open(out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Item #", "Designator", "Qty", "Manufacturer", "Mfg Part #", "Description / Value",
                    "Package/Footprint", "Type", "LCSC", "Your Instructions / Notes"])
        for i, ((value, fp, mpn, mfr), refs) in enumerate(groups.items(), 1):
            part = next(p for p in design.PARTS if p.ref == refs[0])
            pkg = fp.split(":")[1]
            w.writerow([i, ",".join(refs), len(refs), mfr, mpn, value, pkg, "SMD", part.lcsc, part.note])
    return out


if __name__ == "__main__":
    os.makedirs(FAB, exist_ok=True)
    for path in (gerbers(), placement(), bom()):
        print("wrote", os.path.relpath(path, ROOT))
