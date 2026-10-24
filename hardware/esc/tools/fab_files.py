"""Turn kicad-cli's BOM and position files into the columns PCBWay's assembly
quote asks for.

    python3 tools/fab_files.py fab
"""
import csv
import os
import sys

# symbols that are copper on the board, not parts to buy
NOT_PARTS = ("U3",)


def bom(d):
    rows = list(csv.DictReader(open(os.path.join(d, "bom-raw.csv"))))
    out = []
    for r in rows:
        refs = [x for x in r["Designator"].split(",") if x.strip() and x.strip() not in NOT_PARTS]
        if not refs:
            continue
        r["Designator"] = ",".join(x.strip() for x in refs)
        r["Qty"] = str(len(refs))
        out.append(r)
    with open(os.path.join(d, "drone-esc-bom.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(out)
    os.remove(os.path.join(d, "bom-raw.csv"))
    return sum(int(r["Qty"]) for r in out), len(out)


def cpl(d):
    rows = list(csv.DictReader(open(os.path.join(d, "pos-raw.csv"))))
    with open(os.path.join(d, "drone-esc-cpl.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["Designator", "Mid X", "Mid Y", "Layer", "Rotation"])
        for r in rows:
            if r["Ref"] in NOT_PARTS:
                continue
            w.writerow([r["Ref"], f'{float(r["PosX"]):.3f}mm', f'{float(r["PosY"]):.3f}mm',
                        "T" if r["Side"] == "top" else "B", f'{float(r["Rot"]):.1f}'])
    os.remove(os.path.join(d, "pos-raw.csv"))
    return len(rows)


if __name__ == "__main__":
    d = sys.argv[1]
    parts, lines = bom(d)
    placements = cpl(d)
    print(f"BOM: {parts} parts in {lines} lines, CPL: {placements} placements")
