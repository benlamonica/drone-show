"""Board geometry shared by make_footprints.py and build_pcb.py.

All coordinates are mm from the board centre, KiCad screen axes (x right,
y down). The board is drawn with the front up.

The board is an octagon, 80 mm across the flats: four 26.9 mm axis flats at
+-40 mm and four corner flats 37.8 mm from the centre. The corners point at the
arms. It needs a slightly bigger hub than frame/frame.py has today (see
README.md).

Four copper layers: signals and the phase copper on the outside, a solid GND
plane on In1 and a +BATT pour on In2 that signals may also use.

Each motor channel owns one axis edge. Its three half-bridges sit in a row
along that edge: high-side FET on top, low-side FET directly underneath on the
bottom, and the phase copper runs out to a plated motor pad on the edge. The
gate driver sits inboard of the FETs on the bottom and the channel's MCU right
above it on top, both in the channel's wedge between the centre and the FETs.

The corner flats are free for the connections: the battery pads (soldered
from both sides, with the bulk electrolytic) on the rear-right corner flat and
the side-entry FC connector on the front-left one.
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

# each channel's MCU and gate driver, both on the bottom, in channel
# coordinates (x out towards the channel's edge). They are placed exactly, so
# route_prep.pwm_lines can draw the same PWM escape in every channel.
MCU_POS = (6.0, 0.0)
DRV_POS = (11.5, 4.2)
# vias of the hand-drawn MCU escapes (route_prep.pwm_lines), channel
# coordinates, so the parts on top can keep clear of them
MCU_VIAS = {"+3V3": (6.75, 2.8), "NRST": (5.3, -3.4), "vdda": (6.5, -3.9)}
C3V3_POS = (7.0, 4.3)     # on top, pad 1 (3V3) towards the via
CVCC_POS = (13.4, 0.4)    # driver supply cap, bottom, +10V pad towards the driver
RFCA_POS = (8.2, -3.4)    # back-EMF resistor FBA-FBCOMMON, bottom, FBCOMMON pad towards the MCU
# bootstrap caps A, B, C below the driver; C is higher to clear the
# mounting hole
CB_POS = {"A": (9.6, 10.6), "B": (11.6, 10.6), "C": (13.6, 10.0)}

# battery pads on the rear-right corner flat: u along the corner diagonal from
# the centre, v across it (towards the rear edge). Pad 1 is battery +, pad 2
# battery -. Each is a big plated pad, bare copper on both sides, so the lead
# can go on one side and the electrolytic's leg on the other.
BATT_CORNER = "BR"
BATT_U = 33.0
BATT_V = {"1": 1.0, "2": 10.0}
BATT_PAD_L, BATT_PAD_W = 7.0, 6.0     # along the diagonal, across it
BATT_PAD_DRILL = 2.2                  # 12 AWG fits through
BATT_VIA_DRILL, BATT_VIA_D, BATT_VIA_R = 0.4, 0.8, 2.3   # ring of vias in each pad

# FC connector: side-entry JST SH on the front-left corner flat, opening
# facing out so the lead leaves sideways
J1_CORNER = "TL"
J1_U = 34.0


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


CORNER_ANGLE = {"TR": -45.0, "BR": 45.0, "BL": 135.0, "TL": -135.0}   # screen, y down


def battery_pads():
    """Pad 1 (battery +) feeds the shunts on top through the "battery +"
    pour, pad 2 (battery -) lands on the bottom GND pour and the GND plane.
    The third value is the pad's KiCad orientation, long side along the
    corner diagonal."""
    return {n: (*corner_uv(BATT_U, v, BATT_CORNER), -CORNER_ANGLE[BATT_CORNER])
            for n, v in BATT_V.items()}


def j1_pose():
    """Centre of J1 and the screen angle its opening faces."""
    return (*corner_uv(J1_U, 0.0, J1_CORNER), CORNER_ANGLE[J1_CORNER])
