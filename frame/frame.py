"""
Parametric 3D-printable 5" drone-show quad frame (build123d), v3.

Two kinds of parts, as in the Eclipson / 3DLabPrint planes:

* WEB parts (arms, legs): modelled as solid prisms with 0.05 mm slits that
  start at the outer skin and run inward. Printed with 1 wall, 0 % infill and
  no top layers, every layer is ONE closed loop: the nozzle runs along the
  skin, into each slit, around its tip and back out. So there is no travel or
  retraction inside a layer, and each slit becomes a double-line internal web
  fused to the skin.
* FITTING parts (hub, motor pods): normal prints that take the bolts. Every
  surface is vertical or a 45 deg slope (hex pockets, teardrop holes, diamond
  windows), so nothing needs support.
* Shells: vase mode.

Layout: arms on +X/+Y/-X/-Y, shell faces perpendicular to the arms so props
never overlap the shells in plan view. FC stack holes rotated 45 deg so the
flight controller still flies as an X quad. Legs sit outboard of the motors
and pass through the pods; drones stack foot-on-leg inside the pod sockets.

Coordinates: z = 0 is the arm centre line, +z up, mm.

Run:  python frame.py   -> STL + STEP in ./out, prints clearances, masses
                           and a per-layer loop check of the web parts
"""

from __future__ import annotations

import math
from collections import defaultdict
from pathlib import Path

import numpy as np
from build123d import (
    Axis,
    Box,
    Circle,
    Compound,
    Cylinder,
    Part,
    Plane,
    Polygon,
    RegularPolygon,
    Pos,
    Rectangle,
    RectangleRounded,
    Rot,
    Sketch,
    chamfer,
    export_step,
    export_stl,
    extrude,
)

# ---------------------------------------------------------------- printing
SLIT = 0.05                # Eclipson slit width
SLIT_OVERSHOOT = 0.5       # slits start outside the skin so they open onto it
SLIT_TIP_GAP = 0.3         # slits stop this far from each other at the centre
LINE_W = 0.45              # single wall width (for mass estimates)
VASE_WALL = 0.5            # shell wall thickness (for the shell fit)
PINCH_SLIT = 1.0           # clamp slits in the fitting parts

# ---------------------------------------------------------------- props / motors
PROP_DIAMETER = 127.0
MOTOR_RADIUS_POS = 120.0   # 240 mm motor-to-motor across
MOTOR_HOLE_SPACING_A = 16.0
MOTOR_HOLE_SPACING_B = 19.0
MOTOR_SCREW_D = 3.2
MOTOR_CENTER_HOLE_D = 9.0
MOTOR_BELL_D = 27.6
MOTOR_PLATE_R = 14.0
MOTOR_PLATE_T = 3.6        # 3.0 + 0.6 so M3 x 6 screws stay clear of the windings
MOTOR_SHAFT_CLR = 5.0      # room under the plate for the shaft end and its clip

# ---------------------------------------------------------------- FC stack
STACK_SPACING = 30.5
STACK_ROTATION = 45.0      # 45 => FC flies as an X quad on this + frame
STACK_HOLE_D = 3.2         # M3 up from underneath into the stack standoffs
STACK_BOSS_D = 8.0

# ---------------------------------------------------------------- arm (web part)
ARM_W = 12.0               # horizontal
ARM_H = 20.0               # vertical, apex to apex. 45 deg faces top and bottom
FIT = 0.15                 # clearance per side in every socket
WALL = 1.6                 # socket walls in the fitting parts
BOLT_D = 3.3               # M3 clamp bolts

# ---------------------------------------------------------------- leg (web part)
LEG_W = 16.0               # radial (x)
LEG_D = 12.0               # tangential (y)
LEG_R = 3.0
FOOT_SKIN = 0.8            # solid bottom layers on the foot
LEG_TOP_GAP = 2.0          # leg stops this far below the pod top; the foot of
                           # the drone above drops into that space when stacked
STACK_PITCH = 110.0

# ---------------------------------------------------------------- hub / shells
BODY = 90.0                # shell outer size
BODY_R = 12.0
SHELL_FIT_CLEARANCE = 0.2
RING_T = 1.2               # hub ring wall
HUB_Z = 16.0               # hub is z -16 .. +16; the top shell overlaps its top 4 mm
SHELL_OVERLAP = 4.0
TOP_SHELL_H = 28.0
HUB_FLOOR_T = 1.6          # floor across the bottom of the hub; the bay hangs from it
STACK_BOSS_H = 5.0         # floor + boss under each FC stack hole (like a 5 mm frame plate)
NUT_AF = 5.8               # M3 nut pocket across flats (5.5 nut + clearance)
NUT_T = 2.6                # nut pocket depth

# ---------------------------------------------------------------- battery bay
# Ovonic 3S 2200 mAh: 75 x 33 x 25 mm (+/-3 mm). The bay hangs under the hub,
# directly below the FC stack, and runs along a diagonal so the pack slides out
# toward a corner between two legs without hitting an arm or a pod.
BATTERY_L, BATTERY_W, BATTERY_H = 75.0, 33.0, 25.0
BATTERY_TOL = 3.0
BAY_ANGLE = 45.0           # direction the pack slides (deg from +X)
BAY_IN_W = BATTERY_W + BATTERY_TOL + 1.0
BAY_IN_H = BATTERY_H + BATTERY_TOL + 0.5
BAY_IN_L = BATTERY_L - 1.0  # pack sticks out ~1-4 mm at the open end, to grab it
BAY_TOP_T = 2.4            # roof; hooks into the hub floor at the lip end, bolts at the open end
BAY_WALL = 1.6             # sides and bottom
BAY_LIP = 4.0              # stop lip at the closed end; leads pass through its opening
BAY_LIP_T = 2.0
BAY_EAR = 6.0              # side-wall tabs past the open end, notched for a strap
# Mounting: two hooks on the roof at the lip end go up through slots in the
# hub floor; the bay then slides toward its open end so the hooks catch on the
# floor, and two M3 x 8 bolts through a roof tab past the open end go up into
# nuts in the hub floor. No bolt shares the FC stack.
BAY_HOOK_U = -33.0         # hook start along the bay (lip end is -BAY_LEN / 2)
BAY_HOOK_V = 13.0          # hooks either side of the bay centre line
BAY_HOOK_W = 8.0
BAY_HOOK_RISER = 5.0       # riser length along the bay
BAY_HOOK_BARB = 3.0        # how far the hook reaches over the floor
BAY_HOOK_T = 2.0           # barb thickness
BAY_TAB_L = 12.0           # roof tab past the open end
BAY_TAB_W = 26.0
BAY_BOLT_U = 8.0           # bolt distance past the open end
BAY_BOLT_V = 8.0
BAY_BOSS_D = 8.5
BAY_BOSS_H = 5.0           # floor + boss, nut pocket at the top
STACK_HEAD_D = 6.5         # FC stack bolt heads sit in holes in the bay roof

# ---------------------------------------------------------------- GPS
# A hollow mast rises from the hub floor in the corner away from the bay's
# hooks and bolts, through a hole in the top shell. The GPS cap slips over its
# top and is held with one M3 bolt; it also clamps the top shell down onto the
# mast's shoulder. The cap's plate sits outside the bay's footprint so it
# doesn't hit the drone stacked above.
GPS_ANGLE = 135.0          # mast direction from the hub centre
GPS_MAST_R = 38.0          # mast centre distance from the hub centre
GPS_MAST_W = 16.0          # mast below the shell
GPS_NECK_W = 13.0          # mast above the shoulder, through the shell
GPS_MAST_WALL = 1.2
GPS_SKIRT_H = 10.0         # cap socket over the neck
GPS_PLATE = 45.0           # same size as the RTK GPS board (45 x 45 mm)
GPS_PLATE_T = 2.0
GPS_PLATE_R = 47.0         # plate centre distance from the hub centre; inner edge clears the bay above
GPS_BOARD_HOLES = 30.5     # board's M3 holes, on the same 30.5 mm pattern as the FC stack
GPS_BOARD_HOLE_D = 3.2     # M3 clearance; the board sits on M3 standoffs

# ---------------------------------------------------------------- light PCB
# The LED board slides into two rails under the bay, LEDs facing down, and a
# vase-mode diffuser cup slips over the rails from below.
PCB_W = 35.0               # across the rails
PCB_L = 73.0               # along the bay
PCB_T = 1.6
PCB_TOP_GAP = 4.0          # room for parts on the top side of the board
PCB_CLR = 0.15             # board-to-groove clearance
RAIL_T = 1.6               # rail wall
RAIL_LEDGE = 2.0           # how far the ledges reach over the board edge (LED keep-out)
RAIL_LIP_T = 1.2           # ledges above and below the board
RAIL_STOP = 2.0            # rails are closed this long at the lip end
DIFFUSER_GAP = 6.0         # LED to diffuser distance; more = softer light
DIFFUSER_WALL = 0.5        # vase wall (for the fit)
DIFFUSER_CLR = 0.2
SHELL_CHAMFER = 6.0
HUB_SOCKET_DEPTH = 15.0
POD_SOCKET_DEPTH = 15.0

OUT_DIR = Path(__file__).parent / "out"

# ---------------------------------------------------------------- derived
SQ2 = math.sqrt(2)
RING_O = BODY - 2 * (VASE_WALL + SHELL_FIT_CLEARANCE)   # shells slip over the ring
HALF_RING = RING_O / 2
SOCK_Y = ARM_W / 2 + FIT + WALL
POCKET_W, POCKET_H = ARM_W + 2 * FIT, ARM_H + 2 * FIT * SQ2
SOCK_W, SOCK_H = POCKET_W + 2 * WALL, POCKET_H + 2 * WALL * SQ2   # hex socket tube
SOCK_Z = SOCK_H / 2                                     # ridge of the socket = pod top / motor seat
EAR_Y = PINCH_SLIT / 2 + 3.0
ARM_X0 = HALF_RING - HUB_SOCKET_DEPTH
POD_X0 = MOTOR_RADIUS_POS - 35.0                        # arm end of the pod socket
ARM_X1 = POD_X0 + POD_SOCKET_DEPTH
ARM_LEN = ARM_X1 - ARM_X0
LEG_SOCK_X = LEG_W / 2 + FIT + WALL
LEG_SOCK_Y = LEG_D / 2 + FIT + WALL
LEG_CX = MOTOR_RADIUS_POS + MOTOR_BELL_D / 2 + 3.0 + LEG_SOCK_X
LEG_TOP_Z = SOCK_Z - LEG_TOP_GAP
LEG_FOOT_Z = LEG_TOP_Z - STACK_PITCH
TOP_SHELL_Z0 = HUB_Z - SHELL_OVERLAP
BAY_OUT_W = BAY_IN_W + 2 * BAY_WALL
BAY_OUT_H = BAY_IN_H + BAY_TOP_T + BAY_WALL
BAY_LEN = BAY_IN_L + BAY_LIP_T
BAY_Z0 = -HUB_Z - BAY_OUT_H              # bottom of the bay
PCB_Z_TOP = BAY_Z0 - PCB_TOP_GAP
PCB_Z_BOT = PCB_Z_TOP - PCB_T
RAIL_Z0 = PCB_Z_BOT - PCB_CLR - RAIL_LIP_T   # bottom of the rails
DIFFUSER_W = BAY_OUT_W + 2 * (DIFFUSER_WALL + DIFFUSER_CLR)
DIFFUSER_L = BAY_IN_L + BAY_LIP_T + 2 * (DIFFUSER_WALL + DIFFUSER_CLR)
DIFFUSER_H = BAY_Z0 - RAIL_Z0 + DIFFUSER_GAP
LOWEST_Z = BAY_Z0 - DIFFUSER_H
SHELL_TOP_Z = TOP_SHELL_Z0 + TOP_SHELL_H
SHELL_TOP_T = 0.8                         # 4 solid layers on the vase shell's top
GPS_SHOULDER_Z = SHELL_TOP_Z - SHELL_TOP_T  # shell top rests on the mast shoulder here
GPS_CAP_Z0 = SHELL_TOP_Z + 0.1            # cap skirt bottom, clamping the shell top
GPS_BOLT_Z = GPS_CAP_Z0 + GPS_SKIRT_H / 2
GPS_PLATE_Z = GPS_CAP_Z0 + GPS_SKIRT_H    # underside of the receiver plate
BAY_U1 = BAY_LEN / 2                      # open end of the bay
HOOK_SLIDE = BAY_HOOK_BARB + 0.3          # how far the bay slides to engage the hooks


# ---------------------------------------------------------------- 2D helpers
def hex_section(w: float, h: float) -> Sketch:
    """Arm section: w wide, h tall, 45 deg faces meeting at a ridge top and
    bottom, so its pockets print without support."""
    s = h / 2 - w / 2
    return Polygon((0, h / 2), (w / 2, s), (w / 2, -s), (0, -h / 2), (-w / 2, -s),
                   (-w / 2, s), align=None)


def slit(x0: float, y0: float, x1: float, y1: float) -> Sketch:
    """SLIT-wide cut from (x0, y0) to (x1, y1)."""
    length = math.hypot(x1 - x0, y1 - y0)
    ang = math.degrees(math.atan2(y1 - y0, x1 - x0))
    return Pos((x0 + x1) / 2, (y0 + y1) / 2) * Rot(0, 0, ang) * Rectangle(length, SLIT)


def plus_slits(w: float, h: float) -> Sketch:
    """Four slits from the skin toward the centre (top, bottom, left, right).
    They never meet, so the section stays one region with one outline."""
    o, g = SLIT_OVERSHOOT, SLIT_TIP_GAP
    return (slit(0, h / 2 + o, 0, g) + slit(0, -h / 2 - o, 0, -g)
            + slit(-w / 2 - o, 0, -g, 0) + slit(w / 2 + o, 0, g, 0))


def x_slits(w: float, h: float) -> Sketch:
    """Four slits from the corners toward the centre."""
    d = math.hypot(w, h) / 2 + SLIT_OVERSHOOT
    a = math.atan2(h, w)
    s = Sketch()
    for sx, sy in ((1, 1), (-1, 1), (-1, -1), (1, -1)):
        ux, uy = sx * math.cos(a), sy * math.sin(a)
        s += slit(ux * d, uy * d, ux * SLIT_TIP_GAP, uy * SLIT_TIP_GAP)
    return s


def teardrop(d: float, up: int = 1) -> Sketch:
    """Horizontal-hole profile with a 45 deg point toward print-up (up=+1) or
    print-down (up=-1) in the sketch's v direction."""
    r = d / 2
    c = r * math.cos(math.radians(45))
    return Circle(r) + Polygon((-c, up * c), (0, up * r * SQ2), (c, up * c), (0, 0), align=None)


def diamond(size: float) -> Sketch:
    return Rot(0, 0, 45) * Rectangle(size / SQ2, size / SQ2)


def hex_tube(x0: float, length: float) -> Part:
    """Socket tube around the arm pocket, along +X from x0 (pocket not cut)."""
    return extrude(Plane.YZ.offset(x0) * hex_section(SOCK_W, SOCK_H), length, dir=(1, 0, 0))


def y_hole(x: float, z: float, length: float, up: int) -> Part:
    """Teardrop hole along Y through (x, z)."""
    return extrude(Plane.XZ * Pos(x, z) * teardrop(BOLT_D, up), length / 2, both=True)


def motor_holes(xm: float, z_floor: float) -> list[Part]:
    """Screw holes, and a centre hole for the shaft that stops at z_floor with
    a 45 deg point, so the web below it stays joined and prints without
    support (the pod prints upside down)."""
    r = MOTOR_CENTER_HOLE_D / 2
    out = [Pos(xm, 0, z_floor + 40) * Cylinder(r, 80),
           extrude(Plane.XZ * Polygon((xm - r - 0.5, z_floor + 0.5), (xm + r + 0.5, z_floor + 0.5), (xm, z_floor - r),
                                      align=None), r, both=True)]
    # 19 mm pair on the +45 deg diagonal, 16 mm on the other: this puts the
    # motor wires along the pod instead of hanging off its side
    for spacing, ang in ((MOTOR_HOLE_SPACING_A, 135), (MOTOR_HOLE_SPACING_B, 45)):
        for s in (-1, 1):
            out.append(Pos(xm + s * spacing / 2 * math.cos(math.radians(ang)),
                           s * spacing / 2 * math.sin(math.radians(ang)), 0)
                       * Cylinder(MOTOR_SCREW_D / 2, 80))
    return out


def stack_points() -> list[tuple[float, float]]:
    h = STACK_SPACING / 2
    c, s = math.cos(math.radians(STACK_ROTATION)), math.sin(math.radians(STACK_ROTATION))
    return [(h * sx * c - h * sy * s, h * sx * s + h * sy * c) for sx in (-1, 1) for sy in (-1, 1)]


def bay_xy(u: float, v: float) -> tuple[float, float]:
    """Bay (u along the slide direction, v across) to hub x, y."""
    c, s = math.cos(math.radians(BAY_ANGLE)), math.sin(math.radians(BAY_ANGLE))
    return u * c - v * s, u * s + v * c


def bay_bolt_points() -> list[tuple[float, float]]:
    return [bay_xy(BAY_U1 + BAY_BOLT_U, side * BAY_BOLT_V) for side in (-1, 1)]


def gps_local(p):
    """Place a part built around the mast axis (x along GPS_ANGLE) on the hub."""
    return Rot(0, 0, GPS_ANGLE) * Pos(GPS_MAST_R, 0, 0) * p


# ---------------------------------------------------------------- web parts
def arm() -> Part:
    """Flight position, along +X. Print standing on end:
    1 wall, 0 % infill, 0 top, 0 bottom layers."""
    sec = Plane.YZ.offset(ARM_X0)
    return extrude(sec * (hex_section(ARM_W, ARM_H) - plus_slits(ARM_W, ARM_H)), ARM_LEN, dir=(1, 0, 0))


def leg() -> Part:
    """Flight position. Print foot down: 1 wall, 0 % infill, 4 bottom (0.8 mm),
    0 top layers."""
    length = LEG_TOP_Z - LEG_FOOT_Z
    sec = RectangleRounded(LEG_W, LEG_D, LEG_R)
    lg = extrude(sec, FOOT_SKIN) + Pos(0, 0, FOOT_SKIN) * extrude(
        sec - x_slits(LEG_W, LEG_D), length - FOOT_SKIN
    )
    return Pos(LEG_CX, 0, LEG_FOOT_Z) * lg


# ---------------------------------------------------------------- fitting parts
def hub() -> Part:
    """Printed as flown (bed at z = -HUB_Z). 3 walls, 15 % infill, no support."""
    zb, zt = -HUB_Z, HUB_Z
    h = zt - zb
    # outer ring the shells slip over
    ring_plan = RectangleRounded(RING_O, RING_O, BODY_R - VASE_WALL) - RectangleRounded(
        RING_O - 2 * RING_T, RING_O - 2 * RING_T, BODY_R - VASE_WALL - RING_T
    )
    hub_ = Pos(0, 0, zb) * extrude(ring_plan, h)
    hub_ += Pos(0, 0, zb) * extrude(hub_floor_plan(), HUB_FLOOR_T)

    # FC stack bosses, joined by a low ring and spokes to the arm sockets
    pts = stack_points()
    boss_h = STACK_BOSS_H
    r_stack = math.hypot(*pts[0])
    low = Circle(r_stack + 0.8) - Circle(r_stack - 0.8)
    for (px, py) in pts:
        low += Pos(px, py) * Circle(STACK_BOSS_D / 2)
    hub_ += Pos(0, 0, zb) * extrude(low, boss_h)

    sx0 = ARM_X0 - WALL
    for a in (0, 90, 180, 270):
        r = Rot(0, 0, a)
        # spoke from the boss ring out to the socket, along the arm axis
        hub_ += r * Pos((r_stack + sx0) / 2, 0, zb + boss_h / 2) * Box(sx0 - r_stack, 3.0, boss_h)
        # hex socket tube, a fin under its lower ridge down to the bed, and
        # clamp ears on its upper ridge
        hub_ += r * hex_tube(sx0, HALF_RING - sx0)
        hub_ += r * Pos((sx0 + HALF_RING) / 2, 0, (zb - SOCK_Z) / 2 + 0.5) * Box(
            HALF_RING - sx0, WALL, -SOCK_Z - zb + 1)
        ear_z0 = SOCK_Z - EAR_Y - 0.5        # ears start inside the 45 deg ridge faces
        hub_ += r * Pos((sx0 + HALF_RING) / 2, 0, (ear_z0 + zt) / 2) * Box(
            HALF_RING - sx0, 2 * EAR_Y, zt - ear_z0)
    for a in (0, 90, 180, 270):
        r = Rot(0, 0, a)
        pocket = extrude(Plane.YZ.offset(ARM_X0) * hex_section(POCKET_W, POCKET_H),
                         HUB_SOCKET_DEPTH + 2, dir=(1, 0, 0))
        hub_ -= r * pocket
        # clamp slit from the pocket ridge up through the ears
        apex = ARM_H / 2 + FIT * SQ2
        hub_ -= r * Pos((ARM_X0 + HALF_RING) / 2 + 1, 0, (apex + zt) / 2 + 0.5) * Box(
            HUB_SOCKET_DEPTH + 2, PINCH_SLIT, zt - apex + 1
        )
        hub_ -= r * y_hole(ARM_X0 + HUB_SOCKET_DEPTH / 2, (SOCK_Z + zt) / 2, 2 * EAR_Y + 2, +1)
        # diamond windows in the ring either side of the socket (wires, weight)
        for side in (-1, 1):
            win = extrude(Plane.YZ.offset(HALF_RING - RING_T - 1) * Pos(side * 20.5, 0) * diamond(20.0),
                          RING_T + 2, dir=(1, 0, 0))
            hub_ -= r * win
    for (px, py) in pts:
        hub_ -= Pos(px, py, 0) * Cylinder(STACK_HOLE_D / 2, 2 * HUB_Z + 2)

    # battery bay: nut bosses for the two bolts at its open end, slots for the
    # two hooks at its lip end, and the ESC power lead pass-through
    for (bx, by) in bay_bolt_points():
        hub_ += Pos(bx, by, zb) * extrude(Circle(BAY_BOSS_D / 2), BAY_BOSS_H)
    for (bx, by) in bay_bolt_points():
        hub_ -= Pos(bx, by, 0) * Cylinder(BOLT_D / 2, 2 * HUB_Z + 2)
        hub_ -= Pos(bx, by, zb + BAY_BOSS_H - NUT_T) * extrude(
            Rot(0, 0, BAY_ANGLE) * RegularPolygon(NUT_AF / math.sqrt(3), 6), NUT_T + 1)
    slot_u0 = BAY_HOOK_U - HOOK_SLIDE - 0.2
    slot_u1 = BAY_HOOK_U + BAY_HOOK_RISER + 0.1
    for side in (-1, 1):
        hub_ -= Rot(0, 0, BAY_ANGLE) * Pos((slot_u0 + slot_u1) / 2, side * BAY_HOOK_V, zb) * Box(
            slot_u1 - slot_u0, BAY_HOOK_W + 0.4, 2 * HUB_FLOOR_T + 1)
    hub_ -= Rot(0, 0, BAY_ANGLE) * Pos(-BAY_LEN / 2 + 10, 0, zb) * extrude(
        Rot(0, 0, 45) * Rectangle(10, 10), 5, both=True)

    # GPS mast: square tube from the floor up through the top shell, with a
    # 45 deg shoulder the shell top rests on and a narrower neck for the cap
    zs = GPS_SHOULDER_Z - (GPS_MAST_W - GPS_NECK_W) / 2
    z_top = GPS_PLATE_Z - 0.5
    t = GPS_MAST_WALL

    def tube(w: float, z0: float) -> Part:
        body = Pos(0, 0, z0) * extrude(Rectangle(w, w), zs - z0)
        body += Pos(0, 0, zs) * extrude(Rectangle(w, w), (GPS_MAST_W - GPS_NECK_W) / 2, taper=45)
        body += Pos(0, 0, GPS_SHOULDER_Z) * extrude(Rectangle(w - (GPS_MAST_W - GPS_NECK_W),
                                                              w - (GPS_MAST_W - GPS_NECK_W)),
                                                    z_top - GPS_SHOULDER_Z)
        return body

    hub_ += gps_local(tube(GPS_MAST_W, zb))
    hub_ -= gps_local(Pos(0, 0, 0.01) * tube(GPS_MAST_W - 2 * t, zb + HUB_FLOOR_T))
    hub_ -= gps_local(Pos(0, 0, z_top) * extrude(Rectangle(GPS_NECK_W - 2 * t, GPS_NECK_W - 2 * t), 2))
    # cable exit into the hub, facing the centre, and the cap bolt
    hub_ -= gps_local(Pos(-GPS_MAST_W / 2, 0, zb + HUB_FLOOR_T + 7) * Rot(0, 90, 0) * extrude(
        diamond(9.0), 3, both=True))
    hub_ -= gps_local(y_hole(0, GPS_BOLT_Z, GPS_MAST_W + 2, +1))
    return hub_


def hub_floor_plan() -> Sketch:
    """Floor across the hub, with windows where nothing is mounted."""
    plan = RectangleRounded(RING_O, RING_O, BODY_R - VASE_WALL)
    plan -= diamond(30.0)                                 # under the FC stack
    plan -= Pos(*bay_xy(30.0, 0)) * diamond(16.0)         # between the bay bolts
    plan -= Rot(0, 0, -45) * Pos(40.0, 0) * diamond(34.0)  # the free corner
    return plan


def motor_pod() -> Part:
    """Printed upside down (motor seat on the bed). 3 walls, 15-25 % infill,
    no support."""
    xm = MOTOR_RADIUS_POS
    zt = SOCK_Z
    ear_z = 5.0
    # hex arm socket with clamp ears on its lower ridge (upper, as printed)
    sx1 = POD_X0 + POD_SOCKET_DEPTH + WALL
    pod = hex_tube(POD_X0, sx1 - POD_X0)
    ear_top = -SOCK_Z + EAR_Y + 0.5          # ears start inside the 45 deg ridge faces
    pod += Pos(POD_X0 + POD_SOCKET_DEPTH / 2, 0, (ear_top - SOCK_Z - ear_z) / 2) * Box(
        POD_SOCKET_DEPTH, 2 * EAR_Y, ear_top + SOCK_Z + ear_z)
    pod -= extrude(Plane.YZ.offset(POD_X0 - 1) * hex_section(POCKET_W, POCKET_H), POD_SOCKET_DEPTH + 1,
                   dir=(1, 0, 0))
    apex = POCKET_H / 2
    pod -= Pos(POD_X0 + POD_SOCKET_DEPTH / 2 - 0.5, 0, -(apex + SOCK_Z + ear_z) / 2 - 0.5) * Box(
        POD_SOCKET_DEPTH + 1, PINCH_SLIT, SOCK_Z + ear_z - apex + 1
    )
    pod -= y_hole(POD_X0 + POD_SOCKET_DEPTH / 2, -SOCK_Z - ear_z / 2 + 0.5, 2 * EAR_Y + 2, -1)

    # leg socket: through hole, clamp slit and ears on the outboard face
    lx0, lx1 = LEG_CX - LEG_SOCK_X, LEG_CX + LEG_SOCK_X
    pod += Pos(LEG_CX, 0, 0) * Box(2 * LEG_SOCK_X, 2 * LEG_SOCK_Y, 2 * zt)
    ear_x = 6.0
    ear_y = EAR_Y
    pod += Pos(lx1 + ear_x / 2, 0, 0) * Box(ear_x, 2 * ear_y, 2 * zt)
    pod -= Pos(LEG_CX, 0, 0) * extrude(
        RectangleRounded(LEG_W + 2 * FIT, LEG_D + 2 * FIT, LEG_R + FIT), 2 * zt + 2, both=True
    )
    pod -= Pos((LEG_CX + lx1 + ear_x) / 2 + 0.5, 0, 0) * Box(lx1 + ear_x - LEG_CX + 1, PINCH_SLIT, 2 * zt + 2)
    for zc in (-zt / 2, zt / 2):
        pod -= y_hole(lx1 + ear_x / 2, zc, 2 * ear_y + 2, -1)

    # motor plate (top, on the bed as printed) and a central web below it
    zp0 = zt - MOTOR_PLATE_T
    plate = Pos(xm, 0, zp0) * extrude(Circle(MOTOR_PLATE_R), MOTOR_PLATE_T)
    # The plate runs back over the whole arm socket: the bare hex ridge only
    # touched the bed along a line and lifted, the plate gives it a flat face.
    plate += Pos((POD_X0 + lx0 + 1) / 2, 0, zp0) * extrude(Rectangle(lx0 + 1 - POD_X0, 2 * SOCK_Y), MOTOR_PLATE_T)
    pod += plate
    web_h = zp0 + SOCK_Z
    pod += Pos((sx1 + lx0) / 2, 0, -SOCK_Z + web_h / 2) * Box(lx0 - sx1 + 2, WALL, web_h)
    for hole in motor_holes(xm, zp0 - MOTOR_SHAFT_CLR):
        pod -= hole
    # re-cut the arm pocket so the plate can't intrude into it
    pod -= extrude(Plane.YZ.offset(POD_X0 - 1) * hex_section(POCKET_W, POCKET_H), POD_SOCKET_DEPTH + 1,
                   dir=(1, 0, 0))
    return pod


def battery_bay() -> Part:
    """Sleeve under the hub. Open at one end (pack slides out), stop lip at the
    other. Hooks into the hub floor at the lip end and bolts to it at the open
    end. Print standing on the lip end: 4 walls, 15 % infill, no support."""
    u0, u1 = -BAY_LEN / 2, BAY_LEN / 2              # lip end, open end
    zc = -HUB_Z - BAY_OUT_H / 2
    zin_top = -HUB_Z - BAY_TOP_T
    zin_c = zin_top - BAY_IN_H / 2
    sec = Pos(0, zc) * RectangleRounded(BAY_OUT_W, BAY_OUT_H, 3)
    bay = extrude(Plane.YZ.offset(u0) * sec, BAY_LEN, dir=(1, 0, 0))
    # cavity, and the smaller opening through the stop lip for the leads
    bay -= extrude(Plane.YZ.offset(u0 + BAY_LIP_T) * Pos(0, zin_c) * RectangleRounded(BAY_IN_W, BAY_IN_H, 1.5),
                   BAY_IN_L + 1, dir=(1, 0, 0))
    bay -= extrude(Plane.YZ.offset(u0 - 1) * Pos(0, zin_c)
                   * RectangleRounded(BAY_IN_W - 2 * BAY_LIP, BAY_IN_H - 2 * BAY_LIP, 1.5),
                   BAY_LIP_T + 2, dir=(1, 0, 0))
    # side-wall tabs past the open end with a notch for a strap / rubber band;
    # the notch opens upward as printed, so no overhang
    for side in (-1, 1):
        yc = side * (BAY_OUT_W - BAY_WALL) / 2
        # kept inside the straight part of the wall so it never overhangs the
        # rounded corners below it
        tab = Pos(u1 + BAY_EAR / 2, yc, zc) * Box(BAY_EAR, BAY_WALL, BAY_OUT_H - 6.0)
        tab -= Pos(u1 + BAY_EAR / 2 + 1.5, yc, zin_c) * Box(BAY_EAR, BAY_WALL + 1, 12)
        bay += tab
    # diamond windows in the side walls and the bottom (weight, airflow);
    # diamonds are 45 deg on every edge as printed
    for uc in (-18.0, 18.0):
        for side in (-1, 1):
            win = Pos(uc, side * BAY_OUT_W / 2, zin_c) * Rot(90, 0, 0) * extrude(
                Rot(0, 0, 45) * Rectangle(14, 14), BAY_WALL * 3, both=True)
            bay -= win
        bay -= Pos(uc, 0, BAY_Z0) * extrude(Rot(0, 0, 45) * Rectangle(16, 16), BAY_WALL * 3, both=True)
    # rails for the light PCB: C-channels along the bay bottom, one per side.
    # Constant profile along the bay axis, so they print as plain walls.
    yo = BAY_OUT_W / 2
    yi = PCB_W / 2 + PCB_CLR                  # groove bottom (board edge)
    yl = PCB_W / 2 - RAIL_LEDGE               # inner edge of the ledges
    assert yo - RAIL_T >= yi, "PCB_W too wide for the rails"
    g_top, g_bot = PCB_Z_TOP + PCB_CLR, PCB_Z_BOT - PCB_CLR
    for side in (-1, 1):
        prof = (Pos(side * (yo - RAIL_T / 2), (BAY_Z0 + 3.5 + RAIL_Z0) / 2)
                * Rectangle(RAIL_T, BAY_Z0 + 3.5 - RAIL_Z0))
        prof += Pos(side * (yo + yl) / 2, g_top + RAIL_LIP_T / 2) * Rectangle(yo - yl, RAIL_LIP_T)
        prof += Pos(side * (yo + yl) / 2, g_bot - RAIL_LIP_T / 2) * Rectangle(yo - yl, RAIL_LIP_T)
        bay += extrude(Plane.YZ.offset(u0) * prof, BAY_LEN, dir=(1, 0, 0))
        # end stop at the lip end (sits on the bed as printed)
        bay += Pos(u0 + RAIL_STOP / 2, side * (yi + yo - RAIL_T) / 2, (g_top + g_bot) / 2) * Box(
            RAIL_STOP, yo - RAIL_T - yi + 0.5, g_top - g_bot)
    # hooks on the roof at the lip end. They go up through the hub floor slots
    # and catch on the floor once the bay slides toward its open end. The
    # barbs point toward the open end (up as printed) and start with a 45 deg
    # ramp, so they print without support.
    z0 = -HUB_Z
    z1 = z0 + HUB_FLOOR_T + 0.2
    z2 = z1 + BAY_HOOK_T
    ua, ub = BAY_HOOK_U, BAY_HOOK_U + BAY_HOOK_RISER
    uc = ub + BAY_HOOK_BARB
    hook = Polygon((ua, z0 - 0.5), (ub, z0 - 0.5), (ub, z1), (uc, z1), (uc, z2), (ua + z2 - z0 + 0.5, z2),
                   align=None)
    for side in (-1, 1):
        bay += Pos(0, side * BAY_HOOK_V, 0) * extrude(Plane.XZ * hook, BAY_HOOK_W / 2, both=True)
    # roof tab past the open end for the two bolts (up as printed)
    bay += Pos(u1 + BAY_TAB_L / 2 - 0.5, 0, z0 - BAY_TOP_T / 2) * Box(BAY_TAB_L + 1, BAY_TAB_W, BAY_TOP_T)

    # move into flight position, then cut the holes through the roof: bolts at
    # the tab, clearance for the FC stack bolt heads (teardrops pointing toward
    # the open end, which is up as printed)
    bay = Rot(0, 0, BAY_ANGLE) * bay

    def roof_hole(x: float, y: float, d: float) -> Part:
        return Pos(x, y, z0 - BAY_TOP_T / 2) * Rot(0, 0, BAY_ANGLE) * extrude(
            Rot(0, 0, -90) * teardrop(d, +1), BAY_TOP_T / 2 + 0.5, both=True)

    for (px, py) in stack_points():
        bay -= roof_hole(px, py, STACK_HEAD_D)
    for (bx, by) in bay_bolt_points():
        bay -= roof_hole(bx, by, BOLT_D)
    # ESC power lead pass-through into the hub, near the lip end
    lead = extrude(Rot(0, 0, 45) * Rectangle(10, 10), 10, both=True)
    bay -= Rot(0, 0, BAY_ANGLE) * Pos(u0 + 10, 0, -HUB_Z) * lead
    return bay


def light_pcb() -> Part:
    """Placeholder light board, in flight position (for renders and checks)."""
    u = -BAY_LEN / 2 + RAIL_STOP + PCB_L / 2
    return Rot(0, 0, BAY_ANGLE) * Pos(u, 0, PCB_Z_BOT) * extrude(Rectangle(PCB_L, PCB_W), PCB_T)


def light_diffuser() -> Part:
    """Vase-mode cup that slips up over the PCB rails. Flight orientation is
    also the print orientation (closed face on the bed)."""
    c = extrude(RectangleRounded(DIFFUSER_L, DIFFUSER_W, DIFFUSER_WALL + DIFFUSER_CLR + 0.3), DIFFUSER_H)
    c = chamfer(c.edges().group_by(Axis.Z)[0], 0.8)
    return Rot(0, 0, BAY_ANGLE) * Pos(0, 0, LOWEST_Z) * c


def shell(height: float) -> Part:
    """Vase-mode cup; print with the chamfered face on the bed."""
    s = extrude(RectangleRounded(BODY, BODY, BODY_R), height)
    return chamfer(s.edges().group_by(Axis.Z)[-1], SHELL_CHAMFER)


def top_shell() -> Part:
    """Top shell in flight position, with the hole the GPS mast comes up through.
    The hole is in the solid top layers (on the bed as printed), so the shell
    still prints in vase mode."""
    hole = GPS_NECK_W + 0.6
    return Pos(0, 0, TOP_SHELL_Z0) * shell(TOP_SHELL_H) - gps_local(
        Pos(0, 0, SHELL_TOP_Z - SHELL_TOP_T / 2) * Box(hole, hole, SHELL_TOP_T + 0.02))


def gps_cap() -> Part:
    """RTK GPS board plate on a socket that slips over the mast neck, held with one
    M3 x 20 bolt. Its skirt clamps the top shell down onto the mast shoulder.
    Flight position; print upside down (plate on the bed): 3 walls, no support."""
    inner = GPS_NECK_W + 2 * FIT
    outer = inner + 2 * WALL
    cap = Pos(0, 0, GPS_CAP_Z0) * extrude(Rectangle(outer, outer) - Rectangle(inner, inner), GPS_SKIRT_H)
    pc = GPS_PLATE_R - GPS_MAST_R
    plate = Pos(pc, 0) * RectangleRounded(GPS_PLATE, GPS_PLATE, 1.5)
    plate -= Rectangle(GPS_NECK_W - 2 * GPS_MAST_WALL - 1, GPS_NECK_W - 2 * GPS_MAST_WALL - 1)  # cable
    h = GPS_BOARD_HOLES / 2
    for sx in (-1, 1):                                     # board standoff holes
        for sy in (-1, 1):
            plate -= Pos(pc + sx * h, sy * h) * Circle(GPS_BOARD_HOLE_D / 2)
    cap += Pos(0, 0, GPS_PLATE_Z) * extrude(plate, GPS_PLATE_T)
    cap -= y_hole(0, GPS_BOLT_Z, outer + 2, -1)
    return gps_local(cap)


# ---------------------------------------------------------------- assembly / export
def assembly() -> dict[str, list[Part]]:
    parts: dict[str, list[Part]] = {"hub": [hub()], "arm": [], "motor_pod": [], "leg": []}
    a_, p_, l_ = arm(), motor_pod(), leg()
    for a in (0, 90, 180, 270):
        r = Rot(0, 0, a)
        parts["arm"].append(r * a_)
        parts["motor_pod"].append(r * p_)
        parts["leg"].append(r * l_)
    parts["top_shell"] = [top_shell()]
    parts["gps_cap"] = [gps_cap()]
    parts["battery_bay"] = [battery_bay()]
    parts["light_diffuser"] = [light_diffuser()]
    return parts


def on_bed(p: Part) -> Part:
    bb = p.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * p


def print_parts() -> dict[str, tuple[Part, int, dict]]:
    """name: (part in print orientation, quantity, how it is printed)."""
    return {
        "arm": (on_bed(Rot(0, -90, 0) * arm()), 4, {"mode": "web", "bottom": 0.0}),
        "leg": (on_bed(leg()), 4, {"mode": "web", "bottom": FOOT_SKIN}),
        "hub": (on_bed(hub()), 1, {"mode": "fitting"}),
        "motor_pod": (on_bed(Rot(180, 0, 0) * motor_pod()), 4, {"mode": "fitting"}),
        "top_shell_vase": (on_bed(Rot(180, 0, 0) * top_shell()), 1, {"mode": "vase"}),
        "gps_cap": (on_bed(Rot(180, 0, 0) * gps_cap()), 1, {"mode": "fitting"}),
        # standing on the lip end: un-rotate to the bay's own axis, then lip down
        "light_diffuser_vase": (on_bed(Rot(0, 0, -BAY_ANGLE) * light_diffuser()), 1, {"mode": "vase"}),
        "battery_bay": (on_bed(Rot(0, -90, 0) * Rot(0, 0, -BAY_ANGLE) * battery_bay()), 1,
                        {"mode": "fitting"}),
    }


def printed_volume(p: Part, how: dict) -> float:
    """Rough printed volume in mm^3. Web/vase: wall area x line width plus bottom
    layers. Fittings: 70 % of the solid (thin walls are nearly all perimeter)."""
    if how["mode"] == "fitting":
        return p.volume * 0.7
    lw = VASE_WALL if how["mode"] == "vase" else LINE_W
    bottom = 0.8 if how["mode"] == "vase" else how["bottom"]
    vol = 0.0
    for f in p.faces():
        if abs(f.normal_at(f.center()).Z) < 0.7:
            vol += f.area * lw
        elif f.center().Z < 0.01:
            vol += f.area * bottom
    return vol


def layer_loops(p: Part, z: float) -> int:
    """Number of closed outlines the slicer sees on layer z (1 = no travel)."""
    verts, faces = p.tessellate(0.005, 0.1)
    v = np.array([(q.X, q.Y, q.Z) for q in verts])
    tris = v[np.array(faces)]
    d = tris[:, :, 2] - z
    segs = []
    for t, dd in zip(tris, d):
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            if (dd[i] < 0) != (dd[j] < 0):
                s = dd[i] / (dd[i] - dd[j])
                pts.append(t[i] + s * (t[j] - t[i]))
        if len(pts) == 2:
            segs.append(pts)
    key = lambda q: (round(q[0] * 1e4), round(q[1] * 1e4))
    adj = defaultdict(list)
    for i, (a, b) in enumerate(segs):
        adj[key(a)].append(i)
        adj[key(b)].append(i)
    seen, n = set(), 0
    for i in range(len(segs)):
        if i in seen:
            continue
        n += 1
        stack = [i]
        while stack:
            j = stack.pop()
            if j in seen:
                continue
            seen.add(j)
            for q in segs[j]:
                stack.extend(adj[key(q)])
    return n


def report(parts) -> None:
    pla = 1.24e-3
    print(f"motor to motor {2 * MOTOR_RADIUS_POS:.0f} mm, arm {ARM_LEN:.0f} mm, "
          f"leg {LEG_TOP_Z - LEG_FOOT_Z:.0f} mm")
    adj = MOTOR_RADIUS_POS * SQ2
    print(f"prop tip gap {adj - PROP_DIAMETER:.1f} mm, prop tip to shell "
          f"{MOTOR_RADIUS_POS - PROP_DIAMETER / 2 - BODY / 2:.1f} mm")
    print(f"stack pitch {STACK_PITCH:.0f} mm, gap between diffuser and the top shell below when stacked "
          f"{STACK_PITCH + LOWEST_Z - (TOP_SHELL_Z0 + TOP_SHELL_H):.1f} mm, "
          f"ground clearance under the diffuser {LOWEST_Z - LEG_FOOT_Z:.0f} mm")
    print(f"light PCB {PCB_L:.0f} x {PCB_W:.0f} x {PCB_T} mm, {RAIL_LEDGE:.0f} mm edge keep-out on the "
          f"long sides, {PCB_TOP_GAP:.0f} mm for top-side parts, LEDs {DIFFUSER_GAP:.0f} mm above the diffuser")
    print(f"battery bay inside {BAY_IN_L:.0f} x {BAY_IN_W:.0f} x {BAY_IN_H:.0f} mm, "
          f"outside {BAY_LEN + BAY_EAR:.0f} x {BAY_OUT_W:.1f} x {BAY_OUT_H:.1f} mm")
    total = 0.0
    for name, (p, n, how) in parts.items():
        g = printed_volume(p, how) * pla
        total += g * n
        print(f"{name:18s} x{n}  ~{g:5.1f} g each  ({how['mode']})")
    print(f"frame total ~{total:.0f} g PLA")
    for name in ("arm", "leg"):
        p = parts[name][0]
        top = p.bounding_box().max.Z
        zs = [FOOT_SKIN / 2, 2.0, top / 2, top - 0.5]
        print(f"{name}: loops per layer at z={zs}: {[layer_loops(p, z) for z in zs]}")


def main() -> None:
    OUT_DIR.mkdir(exist_ok=True)
    parts = print_parts()
    for name, (p, _, _) in parts.items():
        export_stl(p, str(OUT_DIR / f"{name}.stl"), tolerance=0.005, angular_tolerance=0.1)
        export_step(p, str(OUT_DIR / f"{name}.step"))
    asm = assembly()
    export_step(Compound([p for ps in asm.values() for p in ps]), str(OUT_DIR / "assembly.step"))
    export_pcb_outline(OUT_DIR / "light_pcb_outline.dxf")
    report(parts)


def export_pcb_outline(path: Path) -> None:
    """Board outline plus the strips the rail ledges cover (keep LEDs out),
    origin at the board centre, for the PCB layout tool."""
    from build123d import ExportDXF, Unit

    dxf = ExportDXF(unit=Unit.MM)
    dxf.add_layer("Edge.Cuts")
    dxf.add_layer("KeepOut")
    dxf.add_shape(Rectangle(PCB_L, PCB_W).wire(), layer="Edge.Cuts")
    for side in (-1, 1):
        y = side * (PCB_W / 2 - RAIL_LEDGE / 2)
        dxf.add_shape((Pos(0, y) * Rectangle(PCB_L, RAIL_LEDGE)).wire(), layer="KeepOut")
    dxf.add_shape((Pos(-PCB_L / 2 + 1, 0) * Rectangle(2, PCB_W)).wire(), layer="KeepOut")
    dxf.write(str(path))


if __name__ == "__main__":
    main()
