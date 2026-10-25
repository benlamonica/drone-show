"""Generate rtk_gps.kicad_sch from design.py.

Every pin gets a short wire stub ending in a net label (or a no-connect flag),
so connectivity is explicit and checkable by ERC. Run with: gen_sch.py <stdlib dir> <project dir>
"""
import copy, math, os, sys, uuid
import sexpdata
from sexpdata import Symbol as S

sys.path.insert(0, os.path.dirname(__file__))
from design import PARTS, FLAGS

STD, PRJ = sys.argv[1], sys.argv[2]
NAME = "rtk_gps"
ROOT_UUID = str(uuid.uuid5(uuid.NAMESPACE_URL, "rtk_gps/root"))


def uid(*k):
    return str(uuid.uuid5(uuid.NAMESPACE_URL, "rtk_gps/" + "/".join(map(str, k))))


_libs = {}


def lib(nick):
    if nick not in _libs:
        path = os.path.join(PRJ, "lib", "rtk_gps.kicad_sym") if nick == "rtk_gps" else os.path.join(STD, nick + ".kicad_sym")
        _libs[nick] = sexpdata.loads(open(path).read(), nil=None, true=None, false=None)
    return _libs[nick]


def find(tree, name):
    for e in tree[1:]:
        if isinstance(e, list) and e and e[0] == S("symbol") and e[1] == name:
            return e
    raise KeyError(name)


def flat_symbol(lib_id):
    """Return the symbol with `extends` resolved, renamed to lib_id for lib_symbols."""
    nick, name = lib_id.split(":")
    s = copy.deepcopy(find(lib(nick), name))
    ext = [e for e in s if isinstance(e, list) and e[0] == S("extends")]
    if ext:
        parent = copy.deepcopy(find(lib(nick), ext[0][1]))
        pname = parent[1]
        props = {e[1]: e for e in s if isinstance(e, list) and e[0] == S("property")}
        body = []
        for e in parent[2:]:
            if isinstance(e, list) and e[0] == S("property") and e[1] in props:
                body.append(props.pop(e[1]))
            elif isinstance(e, list) and e[0] == S("symbol"):
                e[1] = e[1].replace(pname, name, 1)
                body.append(e)
            else:
                body.append(e)
        body += list(props.values())
        s = [S("symbol"), name] + body
    s[1] = lib_id
    return s


def pins_of(sym):
    out = []

    def walk(x):
        if isinstance(x, list) and x:
            if x[0] == S("pin"):
                at = [y for y in x if isinstance(y, list) and y[0] == S("at")][0]
                num = [y for y in x if isinstance(y, list) and y[0] == S("number")][0][1]
                out.append((num, float(at[1]), float(at[2]), float(at[3]) if len(at) > 3 else 0.0, str(x[1])))
            for y in x:
                walk(y)
    walk(sym)
    return out


def q(s):
    return '"' + str(s).replace("\\", "\\\\").replace('"', '\\"') + '"'


def r(v):
    return f"{round(v / 1.27) * 1.27:.2f}"


items, libsyms = [], {}


def place(lib_id, ref, value, fp, at, fields, pins_net, dnp=False, in_bom=True, on_board=True, power=False):
    sym = libsyms.setdefault(lib_id, flat_symbol(lib_id))
    sx, sy = at
    sx, sy = round(sx / 2.54) * 2.54, round(sy / 2.54) * 2.54
    pins = pins_of(sym)
    props = [("Reference", ref, (sx, sy - 6, 0), False), ("Value", value, (sx, sy + 6, 0), False),
             ("Footprint", fp, (sx, sy, 0), True), ("Datasheet", "", (sx, sy, 0), True)]
    for k, v in fields.items():
        props.append((k, v, (sx, sy, 0), True))
    txt = [f'\t(symbol (lib_id {q(lib_id)}) (at {sx:.2f} {sy:.2f} 0) (unit 1) (exclude_from_sim no) '
           f'(in_bom {"yes" if in_bom else "no"}) (on_board {"yes" if on_board else "no"}) (dnp {"yes" if dnp else "no"}) '
           f'(uuid {q(uid("sym", ref))})']
    for k, v, (x, y, a), hide in props:
        txt.append(f'\t\t(property {q(k)} {q(v)} (at {x:.2f} {y:.2f} {a}) (effects (font (size 1.27 1.27)){" (hide yes)" if hide else ""}))')
    seen = set()
    for num, *_ in pins:
        if num in seen:
            continue
        seen.add(num)
        txt.append(f'\t\t(pin {q(num)} (uuid {q(uid("pin", ref, num))}))')
    txt.append(f'\t\t(instances (project {q(NAME)} (path {q("/" + ROOT_UUID)} (reference {q(ref)}) (unit 1))))\n\t)')
    items.append("\n".join(txt))

    done = set()
    for num, px, py, ang, typ in pins:
        x, y = sx + px, sy - py
        key = (round(x, 2), round(y, 2))
        net = pins_net.get(num)
        if key in done:
            continue
        if net is None:
            # a stacked pin may share the location with a connected one
            if any(pins_net.get(n2) for n2, qx, qy, *_ in pins if (round(sx + qx, 2), round(sy - qy, 2)) == key):
                continue
            done.add(key)
            items.append(f'\t(no_connect (at {x:.2f} {y:.2f}) (uuid {q(uid("nc", ref, num))}))')
            continue
        done.add(key)
        dx, dy = -math.cos(math.radians(ang)), math.sin(math.radians(ang))
        ex, ey = x + 2.54 * dx, y + 2.54 * dy
        langle = {(-1, 0): 180, (1, 0): 0, (0, 1): 270, (0, -1): 90}[(round(dx), round(dy))]
        just = "right" if langle in (180, 270) else "left"
        items.append(f'\t(wire (pts (xy {x:.2f} {y:.2f}) (xy {ex:.2f} {ey:.2f})) (stroke (width 0) (type default)) (uuid {q(uid("w", ref, num))}))')
        items.append(f'\t(label {q(net)} (at {ex:.2f} {ey:.2f} {langle}) (effects (font (size 1 1)) (justify {just} bottom)) (uuid {q(uid("l", ref, num))}))')


for p in PARTS:
    fields = {"MPN": p["mpn"], "Manufacturer": p["mfr"], "Mouser": p["mouser"]}
    if p["desc"]:
        fields["Description"] = p["desc"]
    place(p["lib"], p["ref"], p["value"], p["fp"], p["at"], fields, p["pins"], dnp=p["dnp"],
          in_bom=not p["lib"].startswith("Connector:TestPoint"))

for i, net in enumerate(FLAGS):
    place("power:PWR_FLAG", f"#FLG{i+1:02d}", "PWR_FLAG", "", (40 + 20 * i, 250), {}, {"1": net},
          in_bom=False, on_board=False)

notes = [
    ((20, 15), "RTK GPS board: NEO-F10N-00B-20 raw L1/L5 -> RP2350 running RTKLIB -> flight controller"),
    ((20, 20), "J1 is the Pixhawk GPS port (pin-for-pin cable). FC_TX/FC_RX are named from the FC side."),
    ((20, 25), "Compass U4 sits on the FC's I2C bus (addr 0x30, ArduPilot MMC5XX3); RP2350 does not touch it."),
    ((300, 230), "RF: keep J4->U3.11 trace short (<5 mm), ground-stitched both sides."),
    ((300, 235), "Antenna bias: VCC_RF -> R11 10R -> L2 27nH -> RF_IN (module DC-blocks RF_IN)."),
]
for (x, y), t in notes:
    items.append(f'\t(text {q(t)} (exclude_from_sim no) (at {x:.2f} {y:.2f} 0) (effects (font (size 1.5 1.5)) (justify left bottom)) (uuid {q(uid("t", t))}))')

import re, collections
_pts = collections.defaultdict(set)
for it in items:
    for m in re.finditer(r'\(wire \(pts \(xy ([\d.-]+) ([\d.-]+)\) \(xy ([\d.-]+) ([\d.-]+)\)\).*?uuid "([^"]+)"', it):
        for x, y in ((m[1], m[2]), (m[3], m[4])):
            _pts[(x, y)].add(m[5])
clash = [k for k, v in _pts.items() if len(v) > 1]
assert not clash, f"wire endpoints collide at {clash}"
out = [f'(kicad_sch (version 20250114) (generator "eeschema") (generator_version "9.0") (uuid {q(ROOT_UUID)}) (paper "A3")',
       '\t(title_block (title "RTK GPS + compass") (rev "A") (company "drone-show") (comment 1 "2-layer, PCBWay rules, 0603 minimum passives"))',
       "\t(lib_symbols"]
for s in libsyms.values():
    out.append("\t\t" + sexpdata.dumps(s))
out.append("\t)")
out += items
out.append('\t(sheet_instances (path "/" (page "1")))\n\t(embedded_fonts no)\n)')
open(os.path.join(PRJ, NAME + ".kicad_sch"), "w").write("\n".join(out) + "\n")
print("parts", len(PARTS))
