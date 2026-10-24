"""Fabrication outputs for PCBWay: gerbers + drill (zipped), BOM, pick-and-place.

Writes fab/ next to the project:
  light-controller-gerbers.zip   upload this for the bare board
  light-controller-bom.csv       PCBWay assembly BOM format
  light-controller-cpl.csv       placement, both sides (LEDs and a few parts are on the bottom)
"""
import collections
import csv
import os
import shutil
import subprocess
import sys

sys.path.insert(0, os.path.dirname(__file__))
import design  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
PCB = os.path.join(ROOT, "light-controller.kicad_pcb")
FAB = os.path.join(ROOT, "fab")
LAYERS = "F.Cu,B.Cu,F.Paste,B.Paste,F.SilkS,B.SilkS,F.Mask,B.Mask,Edge.Cuts"


def cli(*args):
    subprocess.run(["kicad-cli", "pcb", "export", *args], check=True, capture_output=True)


def main():
    shutil.rmtree(FAB, ignore_errors=True)
    g = os.path.join(FAB, "gerbers")
    os.makedirs(g)
    cli("gerbers", "--layers", LAYERS, "--subtract-soldermask", "--no-x2", "-o", g + "/", PCB)
    cli("drill", "--format", "excellon", "--excellon-units", "mm", "--excellon-separate-th",
        "--generate-map", "--map-format", "gerberx2", "-o", g + "/", PCB)
    shutil.make_archive(os.path.join(FAB, "light-controller-gerbers"), "zip", g)

    pos = os.path.join(FAB, "pos.csv")
    cli("pos", "--format", "csv", "--units", "mm", "--side", "both", "--exclude-dnp", "-o", pos, PCB)
    with open(pos) as f, open(os.path.join(FAB, "light-controller-cpl.csv"), "w", newline="") as o:
        w = csv.writer(o)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in csv.DictReader(f):
            w.writerow([r["Ref"], r["PosX"] + "mm", r["PosY"] + "mm",
                        "Top" if r["Side"] == "top" else "Bottom", r["Rot"]])
    os.remove(pos)

    groups = collections.OrderedDict()
    for p in design.PARTS:
        if p["dnp"]:
            continue
        key = (p["mfr"], p["mpn"], p["value"], p["fp"])
        groups.setdefault(key, []).append(p["ref"])
    with open(os.path.join(FAB, "light-controller-bom.csv"), "w", newline="") as o:
        w = csv.writer(o)
        w.writerow(["Item #", "Designator", "Qty", "Manufacturer", "Mfg Part #", "Description / Value",
                    "Package/Footprint", "Type", "Your Instructions / Notes"])
        for i, ((mfr, mpn, val, fp), refs) in enumerate(groups.items(), 1):
            tht = fp.startswith("SolderWire") or fp.startswith("USB_C")
            note = ""
            if fp == "SolderWire-0.5sqmm_1x02_P4.6mm_D0.9mm_OD2.1mm":
                note = "Do not place: battery leads soldered by hand"
            if fp.startswith("LED_Cree"):
                note = "Bottom side. Any in-stock flux bin of this colour."
            if fp == "TestPoint_Pad_D1.5mm":
                note = "Bare pad, nothing to place"
            w.writerow([i, " ".join(refs), len(refs), mfr, mpn, val, fp, "THT/SMD" if tht else "SMD", note])
    dnp = [p["ref"] for p in design.PARTS if p["dnp"]]
    print("fab outputs in", FAB, "| DNP:", " ".join(dnp))


if __name__ == "__main__":
    main()
