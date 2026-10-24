"""Second router pass on an already routed board: keeps the existing wiring,
lets Freerouting finish the connections it left open, then re-pours ground.

    python3 finish.py [--passes N]
"""

import argparse
import os
import shutil
import subprocess
import sys

import pcbnew

import route as rt


def export(dsn):
    """Pour-free copy of the routed board, exported for the router."""
    board = pcbnew.LoadBoard(rt.PCB)
    for z in [board.GetArea(i) for i in range(board.GetAreaCount())]:
        if not z.GetIsRuleArea():
            board.Remove(z)
    pcbnew.ExportSpecctraDSN(board, dsn)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--passes", type=int, default=30)
    ap.add_argument("--export", help=argparse.SUPPRESS)
    ap.add_argument("--step", help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.step:
        step(args.step)
        return
    if args.export:
        export(args.export)
        return
    dsn, ses = os.path.join(rt.WORK, "finish.dsn"), os.path.join(rt.WORK, "finish.ses")
    if not os.environ.get("REUSE"):
        # the DSN exporter leaves the board unusable, so it runs in its own process
        subprocess.run([sys.executable, __file__, "--export", dsn], check=True)
        jar = os.environ.get("FREEROUTING_JAR", "/opt/fr/fr.jar")
        cmd = ["java", "-jar", jar, "-de", dsn, "-do", ses, "-mp", str(args.passes)]
        if not os.environ.get("DISPLAY") and shutil.which("xvfb-run"):
            cmd = ["xvfb-run", "-a"] + cmd
        subprocess.run(cmd, check=True, stdout=open(os.path.join(rt.WORK, "finish.log"), "w"),
                       stderr=subprocess.STDOUT)
    # pcbnew's bindings misbehave after a board is edited and reloaded in the
    # same process, so each step runs in a fresh one
    for name in ("import", "stitch", "check"):
        subprocess.run([sys.executable, __file__, "--step", name], check=True)


def step(name):
    board = pcbnew.LoadBoard(rt.PCB)
    if name == "import":
        for t in list(board.GetTracks()):
            board.Remove(t)
        nt, nv = rt.import_ses(board, os.path.join(rt.WORK, "finish.ses"))
        print(f"imported {nt} track segments, {nv} vias")
    elif name == "stitch":
        board.BuildConnectivity()
        rt.fill(board)
        print("stitching vias:", rt.stitch(board))
    else:
        board.BuildConnectivity()
        rt.fill(board)
        counts, unc = rt.drc(board, os.path.join(rt.WORK, "drc.rpt"))
        print("DRC:", counts, "unconnected pads:", unc)
    pcbnew.SaveBoard(rt.PCB, board)

if __name__ == "__main__":
    main()
