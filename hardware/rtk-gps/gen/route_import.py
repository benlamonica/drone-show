"""Import the freerouting session (own process: SWIG proxies go stale after it) and clear RF_IN,
which route_post.py lays by hand."""
import sys, pcbnew
b = pcbnew.LoadBoard(sys.argv[1])
pcbnew.ImportSpecctraSES(b, sys.argv[2])
b.Save(sys.argv[1])
b = pcbnew.LoadBoard(sys.argv[1])
rf = [t for t in b.GetTracks() if t.GetNetname() == "/RF_IN"]
for t in rf:
    b.Delete(t)
b.Save(sys.argv[1])
print("cleared RF tracks", len(rf))
