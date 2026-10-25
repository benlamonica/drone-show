"""Board geometry shared by make_footprints.py and build_pcb.py.

All coordinates are mm from the board centre, KiCad screen axes (x right,
y down). The board is drawn with the FC connector edge (front) up.

The board is an octagon, 80 mm across the flats: four 26.9 mm axis flats at
+-40 mm and four corner flats 37.8 mm from the centre. The corners point at the
arms. It needs a slightly bigger hub than frame/frame.py has today (see
README.md).

Four copper layers: signals and the phase copper on the outside, a solid GND
plane on In1 and a +BATT pour on In2 that signals may also use.

Each motor channel owns one edge. Its three half-bridges sit in a row along
that edge: high-side FET on top, low-side FET directly underneath on the
bottom, and the phase copper runs out to a plated motor pad on the edge. The
channel's MCU and gate driver sit on the bottom in the corner clockwise from
its edge.
"""
import math

S2 = math.sqrt(2.0)

HALF = 40.0           # axis flats
CHAMFER = 37.8        # corner flats, distance from the centre
EDGE_MARGIN = 0.4     # parts to board edge
LAYERS = 4

HOLE_PITCH = 30.5     # stack standard
HOLE_D = 3.3          # M3 clearance, unplated: the stack sits on rubber standoffs
HOLE_COPPER_FREE = 6.9   # diameter kept clear of copper on every layer (same as the FC)
HOLE_PART_KEEPOUT = 3.7   # grommet / standoff flange radius, no parts inside

# channel -> number of 90 degree clockwise turns from the right-hand (E) edge.
# Betaflight order with the connector at the front: 1 rear-right, 2 front-right,
# 3 rear-left, 4 front-left. Each motor's pads are on the edge clockwise from
# its arm.
CHANNELS = {"ESC2": 0, "ESC1": 1, "ESC3": 2, "ESC4": 3}

PHASE_Y = {"A": -6.5, "B": 0.0, "C": 6.5}
FET_X = 27.0          # centre of the stacked high/low-side FET pair
MOTOR_PAD_X = 35.6
MOTOR_PAD_W, MOTOR_PAD_H, MOTOR_PAD_DRILL = 3.2, 4.0, 1.2

# prototype symbol pin numbers for the motor pads
MOTOR_PAD_NUM = {
    ("ESC4", "A"): "12", ("ESC4", "B"): "13", ("ESC4", "C"): "14",
    ("ESC3", "A"): "15", ("ESC3", "B"): "16", ("ESC3", "C"): "17",
    ("ESC2", "C"): "18", ("ESC2", "B"): "19", ("ESC2", "A"): "20",
    ("ESC1", "C"): "21", ("ESC1", "B"): "22", ("ESC1", "A"): "23",
}

BATT_PAD_D, BATT_PAD_DRILL = 4.6, 2.0   # 14 AWG goes through the hole
BATT_X, BATT_Y = (-3.0, 4.6), 2.6

JST_POS = (0.0, -6.4)


def rot(x, y, k):
    """Rotate a point k quarter turns clockwise (as seen on screen)."""
    for _ in range(k % 4):
        x, y = -y, x
    return x, y


def outline():
    c = CHAMFER * S2 - HALF
    return [(HALF, -c), (HALF, c), (c, HALF), (-c, HALF),
            (-HALF, c), (-HALF, -c), (-c, -HALF), (c, -HALF)]


def holes():
    h = HOLE_PITCH / 2
    return [(h, -h), (h, h), (-h, h), (-h, -h)]


def motor_pads():
    out = {}
    for ch, k in CHANNELS.items():
        for ph, y in PHASE_Y.items():
            x, yy = rot(MOTOR_PAD_X, y, k)
            out[MOTOR_PAD_NUM[(ch, ph)]] = (x, yy, 90.0 if k % 2 == 0 else 0.0)
    return out


def corner_uv(u, v, corner):
    """Point u along a corner diagonal, v across it. corner in TR, BR, BL, TL."""
    du = {"TR": (1, -1), "BR": (1, 1), "BL": (-1, 1), "TL": (-1, -1)}[corner]
    dv = (-du[1], du[0])
    return ((u * du[0] + v * dv[0]) / S2, (u * du[1] + v * dv[1]) / S2)


def battery_pads():
    """Battery leads come up through the window in the hub floor under the
    stack, go through these holes from the bottom and are soldered on both
    sides. Pad 1 (battery +) feeds the shunts on top, pad 2 (battery -) lands
    on the bottom GND pour. Power enters in the middle and spreads out to the
    four edges."""
    return {"1": (BATT_X[0], BATT_Y, 0.0), "2": (BATT_X[1], BATT_Y, 0.0)}
