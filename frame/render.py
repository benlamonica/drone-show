"""Render PNG previews of the frame (single drone, 3-high stack, print plates).

Run:  python render.py   (after or instead of frame.py; writes ./out/*.png)
"""

from __future__ import annotations

import math

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from build123d import Part, Pos
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

import frame as f

COLORS = {
    "hub": "#3a3a3a",
    "arm": "#4a4a4a",
    "leg": "#333333",
    "motor_pod": "#2b2b2b",
    "top_shell": "#e8eef5",
    "battery_bay": "#3f3f3f",
    "battery": "#d0861c",
    "light_diffuser": "#f4f8ff",
    "light_pcb": "#1f7a3a",
    "gps_cap": "#2f4f6f",
    "motor": "#a01818",
    "prop": "#c03030",
}
ALPHA = {"top_shell": 0.35, "prop": 0.25, "light_diffuser": 0.3}


def tris(p: Part, tol: float = 0.3) -> np.ndarray:
    verts, faces = p.tessellate(tol, 0.5)
    v = np.array([(q.X, q.Y, q.Z) for q in verts])
    return v[np.array(faces)]


def motor_and_prop() -> dict[str, list[Part]]:
    from build123d import Cylinder

    out: dict[str, list[Part]] = {"motor": [], "prop": []}
    z0 = f.SOCK_Z  # motor plate top
    for a in (0, 90, 180, 270):
        x = f.MOTOR_RADIUS_POS * math.cos(math.radians(a))
        y = f.MOTOR_RADIUS_POS * math.sin(math.radians(a))
        out["motor"].append(Pos(x, y, z0 + 17.6 / 2) * Cylinder(f.MOTOR_BELL_D / 2, 17.6))
        out["prop"].append(Pos(x, y, z0 + 22) * Cylinder(f.PROP_DIAMETER / 2, 0.5))
    return out


def battery(slide: float = 0.0) -> Part:
    """Nominal pack in the bay, optionally slid out along the bay axis."""
    from build123d import Box, Rot

    zin_c = -f.HUB_Z - f.BAY_TOP_T - f.BAY_IN_H / 2
    u = -f.BAY_LEN / 2 + f.BAY_LIP_T + f.BATTERY_L / 2 + slide
    return Rot(0, 0, f.BAY_ANGLE) * Pos(u, 0, zin_c - (f.BAY_IN_H - f.BATTERY_H) / 2 + 1) * Box(
        f.BATTERY_L, f.BATTERY_W, f.BATTERY_H)


def draw(ax, groups: dict[str, list[Part]], dz: float = 0.0, light=np.array([0.3, -0.5, 0.8])):
    light = light / np.linalg.norm(light)
    for name, parts in groups.items():
        key = next(k for k in COLORS if name.startswith(k))
        base = np.array(matplotlib.colors.to_rgb(COLORS[key]))
        for p in parts:
            t = tris(p) + np.array([0, 0, dz])
            n = np.cross(t[:, 1] - t[:, 0], t[:, 2] - t[:, 0])
            n /= np.linalg.norm(n, axis=1, keepdims=True) + 1e-12
            shade = 0.45 + 0.55 * np.abs(n @ light)
            cols = np.clip(base[None, :] * shade[:, None] + 0.12, 0, 1)
            pc = Poly3DCollection(t, facecolors=np.c_[cols, np.full(len(cols), ALPHA.get(key, 1.0))],
                                  linewidths=0)
            ax.add_collection3d(pc)


def setup(ax, lim, zlim, elev=22, azim=-60):
    ax.set_xlim(-lim, lim)
    ax.set_ylim(-lim, lim)
    ax.set_zlim(*zlim)
    ax.set_box_aspect((2 * lim, 2 * lim, zlim[1] - zlim[0]))
    ax.view_init(elev=elev, azim=azim)
    ax.set_axis_off()


def main() -> None:
    f.OUT_DIR.mkdir(exist_ok=True)
    asm = f.assembly()
    mp = motor_and_prop()

    fig = plt.figure(figsize=(10, 7), dpi=130)
    ax = fig.add_subplot(projection="3d")
    draw(ax, {**asm, **mp, "battery": [battery()], "light_pcb": [f.light_pcb()]})
    setup(ax, 160, (-90, 60))
    ax.set_title("5\" show frame, flight position (props as discs)")
    fig.savefig(f.OUT_DIR / "assembly.png", bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(10, 7), dpi=130)
    ax = fig.add_subplot(projection="3d")
    draw(ax, {**asm, **mp}, light=np.array([0.2, -1, 0.3]))
    setup(ax, 160, (-90, 60), elev=2, azim=-45)
    ax.set_title("side view")
    fig.savefig(f.OUT_DIR / "side.png", bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(8, 10), dpi=130)
    ax = fig.add_subplot(projection="3d")
    for i in range(3):
        draw(ax, {**asm, "motor": mp["motor"]}, dz=i * f.STACK_PITCH)
    setup(ax, 160, (-90, 2 * f.STACK_PITCH + 60), elev=15, azim=-55)
    ax.set_title(f"3 stacked (pitch {f.STACK_PITCH:.0f} mm, props off)")
    fig.savefig(f.OUT_DIR / "stack.png", bbox_inches="tight")
    plt.close(fig)

    sh = Pos(-f.MOTOR_RADIUS_POS, 0, 0)
    fig = plt.figure(figsize=(10, 7), dpi=130)
    ax = fig.add_subplot(projection="3d")
    draw(ax, {"motor_pod": [sh * f.motor_pod()], "leg": [sh * f.leg()], "arm": [sh * f.arm()]})
    setup(ax, 50, (-80, 20), elev=20, azim=-130)
    ax.set_title("motor pod with arm and leg")
    fig.savefig(f.OUT_DIR / "motor_pod.png", bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(10, 7), dpi=130)
    ax = fig.add_subplot(projection="3d")
    draw(ax, {"hub": asm["hub"]})
    setup(ax, 65, (-20, 20), elev=50, azim=-60)
    ax.set_title("hub")
    fig.savefig(f.OUT_DIR / "hub.png", bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(10, 7), dpi=130)
    ax = fig.add_subplot(projection="3d")
    draw(ax, {"hub": asm["hub"], "battery_bay": asm["battery_bay"], "arm": asm["arm"],
              "battery": [battery(slide=45.0)], "light_pcb": [f.light_pcb()],
              "light_diffuser": asm["light_diffuser"]})
    setup(ax, 85, (-75, 20), elev=-22, azim=-100)
    ax.set_title("battery slides out toward a corner; light PCB in rails below, diffuser over it")
    fig.savefig(f.OUT_DIR / "battery_bay.png", bbox_inches="tight")
    plt.close(fig)

    fig = plt.figure(figsize=(13, 6), dpi=130)
    ax = fig.add_subplot(1, 2, 1, projection="3d")
    draw(ax, {"hub": asm["hub"]}, light=np.array([0.3, -0.5, -0.8]))
    setup(ax, 48, (-20, 50), elev=-50, azim=-60)
    ax.set_title("hub floor from below: hook slots, bay bolt holes", fontsize=9)
    ax = fig.add_subplot(1, 2, 2, projection="3d")
    draw(ax, {"battery_bay": asm["battery_bay"]})
    setup(ax, 48, (-60, 0), elev=35, azim=-30)
    ax.set_title("bay roof: 2 hooks at the lip end, bolt tab at the open end", fontsize=9)
    fig.savefig(f.OUT_DIR / "bay_mount.png", bbox_inches="tight")
    plt.close(fig)

    sections()


def _section(p: Part, z: float):
    """Segments where plane z cuts the tessellated part."""
    t = tris(p, 0.05)
    d = t[:, :, 2] - z
    segs = []
    for tri, dd in zip(t, d):
        pts = []
        for i in range(3):
            j = (i + 1) % 3
            if (dd[i] < 0) != (dd[j] < 0):
                s = dd[i] / (dd[i] - dd[j])
                pts.append(tri[i] + s * (tri[j] - tri[i]))
        if len(pts) == 2:
            segs.append(pts)
    return segs


def sections() -> None:
    """Cross-sections in print orientation: what the slicer sees on one layer."""
    pp = f.print_parts()
    views = [("arm", 35.0), ("leg", 40.0)]
    fig, axs = plt.subplots(1, 2, figsize=(10, 4.5), dpi=150)
    for ax, (name, z) in zip(axs, views):
        for a, b in _section(pp[name][0], z):
            ax.plot([a[0], b[0]], [a[1], b[1]], "k-", lw=0.7)
        ax.set_aspect("equal")
        ax.set_title(f"{name}: layer at z={z:g} mm, {f.layer_loops(pp[name][0], z)} loop", fontsize=9)
        ax.tick_params(labelsize=7)
    fig.suptitle(f"{f.SLIT} mm slits from the skin = internal webs; 1 wall, 0% infill, one loop per layer")
    fig.savefig(f.OUT_DIR / "sections.png", bbox_inches="tight")
    plt.close(fig)


if __name__ == "__main__":
    main()
