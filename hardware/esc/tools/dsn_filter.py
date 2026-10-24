"""Strip the DSN down to what Freerouting should route.

Drops every plane: the pours are added back in KiCad after routing. GND and
+BATT stay in the file, but in the Power class that Freerouting is told to
ignore (-inc Power); their pads get vias from route_prep.py instead.
"""
import sys

src, dst = sys.argv[1], sys.argv[2]
t = open(src).read()


def drop_blocks(text, head):
    out, i = [], 0
    while True:
        j = text.find(head, i)
        if j < 0:
            out.append(text[i:])
            return "".join(out)
        out.append(text[i:j])
        depth, k = 0, j
        while True:
            if text[k] == "(":
                depth += 1
            elif text[k] == ")":
                depth -= 1
                if depth == 0:
                    break
            k += 1
        i = k + 1


t = drop_blocks(t, "(plane ")
open(dst, "w").write(t)
