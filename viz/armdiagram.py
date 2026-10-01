"""A labelled drawing of the robot: every dimension, joint and travel in one
figure.

``arm_model.png`` is the model's specification sheet -- the elevation with the
link lengths dimensioned, the plan with the shoulder offset and the workable
annulus, and the two tables (this model's four driven joints against the
manufacturer's six).  Nothing here is typed in twice: every number is read
from :mod:`robot`, so the figure cannot drift away from the code that the
optimiser actually runs.

The pose drawn is a real one -- a top-down pick at ``(0.60, -d3, 0.30)`` on
the shoulder-right / elbow-up branch -- so the vertical approach, the joint
angles and the tool tip on a port are all the ones the evaluator would see.
"""

from __future__ import annotations

import math
import os
from typing import List, Sequence, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Arc, Circle, Polygon, Rectangle, Wedge

from armmesh import collision_spheres, posed_polygons
from robot import (
    A2,
    A3,
    D1,
    D3,
    D4,
    L3,
    LOCKED_PUMA_JOINTS,
    PSI3,
    PUMA_DH,
    PUMA_JOINT_INDEX,
    PUMA_MAX_SPEED_DEG,
    PUMA_OFFSET,
    PUMA_TRAVEL,
    Puma560Arm,
)
from visualize import DPI, reachable_radii

#: The pose the elevation is drawn in: a top-down pick, shoulder right, elbow
#: up.  Chosen only because all four joints are visibly away from zero, so
#: every angle arc in the figure is readable.
PICK_TARGET: Tuple[float, float] = (0.60, 0.30)
PICK_BRANCH: Tuple[int, int] = (1, -1)

#: The cell the instance family puts this arm in, for the plan view.
CELL_HALF = 0.70
KEEP_OUT = 0.18
MACHINE_HEIGHTS = (0.10, 0.40)
SIGMA_MIN = 0.14
RETRACT = 0.06

DIM = "#b4272d"        # dimension lines and their labels
ANGLE = "#1a5fb4"      # joint angle arcs
CENTRE = "#22262b"     # the kinematic centreline
GHOST = "#8d939b"      # reference lines a dimension is measured from
MACHINE = "#4c72b0"


# ----------------------------------------------------------------------
# drawing primitives
# ----------------------------------------------------------------------


def _unit(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    return vector / norm if norm > 1e-12 else vector


def dimension(
    ax,
    p0: Sequence[float],
    p1: Sequence[float],
    label: str,
    offset: float = 0.06,
    *,
    fontsize: float = 8.0,
    colour: str = DIM,
    text_offset: float = 0.022,
    rotate: bool = True,
) -> None:
    """A dimensioned length: witness lines, a double arrow, a rotated label.

    ``offset`` is signed and measured along the left-hand normal of
    ``p0 -> p1``, so which side of the link the dimension sits on is a sign.
    """
    p0 = np.asarray(p0, dtype=float)
    p1 = np.asarray(p1, dtype=float)
    along = _unit(p1 - p0)
    normal = np.array([-along[1], along[0]]) * offset
    a, b = p0 + normal, p1 + normal

    for end, shifted in ((p0, a), (p1, b)):
        ax.plot(*zip(end, shifted + _unit(normal) * 0.012), color=colour,
                linewidth=0.6, alpha=0.8, zorder=5)
    ax.annotate("", xy=a, xytext=b, zorder=5,
                arrowprops=dict(arrowstyle="<|-|>", color=colour, linewidth=0.9,
                                shrinkA=0, shrinkB=0, mutation_scale=9))

    angle = math.degrees(math.atan2(along[1], along[0])) if rotate else 0.0
    if angle > 90.0:
        angle -= 180.0
    elif angle < -90.0:
        angle += 180.0
    mid = 0.5 * (a + b) + _unit(normal) * text_offset
    ax.text(*mid, label, color=colour, fontsize=fontsize, ha="center", va="center",
            rotation=angle, rotation_mode="anchor", zorder=6,
            bbox=dict(boxstyle="round,pad=0.16", facecolor="white",
                      edgecolor="none", alpha=0.82))


def angle_arc(
    ax,
    centre: Sequence[float],
    radius: float,
    theta_from: float,
    theta_to: float,
    label: str,
    *,
    colour: str = ANGLE,
    fontsize: float = 8.5,
    label_radius: float = 1.30,
    label_deg: float | None = None,
) -> None:
    """The arc swept from a reference direction to a link, with its name.

    ``label_deg`` puts the label somewhere other than the middle of the arc,
    for the joints whose arc is small or lands on top of the drawing.
    """
    centre = np.asarray(centre, dtype=float)
    lo, hi = sorted((theta_from, theta_to))
    ax.add_patch(Arc(centre, 2 * radius, 2 * radius, theta1=lo, theta2=hi,
                     edgecolor=colour, linewidth=1.2, zorder=5))
    # An arrowhead on the swept end says which way the angle is measured.
    end = math.radians(theta_to)
    tip = centre + radius * np.array([math.cos(end), math.sin(end)])
    direction = 1.0 if theta_to >= theta_from else -1.0
    tangent = direction * np.array([-math.sin(end), math.cos(end)])
    ax.annotate("", xy=tip, xytext=tip - tangent * 0.02, zorder=5,
                arrowprops=dict(arrowstyle="-|>", color=colour, linewidth=0.9,
                                shrinkA=0, shrinkB=0, mutation_scale=8))
    mid = math.radians(0.5 * (lo + hi) if label_deg is None else label_deg)
    ax.text(*(centre + radius * label_radius * np.array([math.cos(mid), math.sin(mid)])),
            label, color=colour, fontsize=fontsize, ha="center", va="center", zorder=6,
            bbox=dict(boxstyle="round,pad=0.16", facecolor="white",
                      edgecolor="none", alpha=0.85))


def ray(ax, origin: Sequence[float], angle_deg: float, length: float,
        colour: str = GHOST) -> None:
    """A dashed reference direction -- what a joint angle is measured from."""
    origin = np.asarray(origin, dtype=float)
    end = origin + length * np.array([math.cos(math.radians(angle_deg)),
                                      math.sin(math.radians(angle_deg))])
    ax.plot(*zip(origin, end), linestyle=(0, (5, 3)), color=colour,
            linewidth=0.8, zorder=3)


def callout(ax, point: Sequence[float], text: str, offset: Sequence[float],
            *, fontsize: float = 8.0, colour: str = CENTRE, ha: str = "left") -> None:
    """A leader line from a feature to a label set clear of the drawing."""
    point = np.asarray(point, dtype=float)
    ax.annotate(text, xy=point, xytext=point + np.asarray(offset, dtype=float),
                fontsize=fontsize, color=colour, ha=ha, va="center", zorder=7,
                arrowprops=dict(arrowstyle="-", color=colour, linewidth=0.7,
                                shrinkA=0, shrinkB=2, alpha=0.7),
                bbox=dict(boxstyle="round,pad=0.22", facecolor="white",
                          edgecolor="#d8dade", alpha=0.92))


def _project(arm: Puma560Arm, q: np.ndarray, plane: str) -> Tuple[List[np.ndarray], np.ndarray]:
    """The solid model's polygons flattened onto a plane, painter-sorted.

    ``plane`` is ``"xz"`` for the elevation (drawn far side first, the camera
    on ``-y``) or ``"xy"`` for the plan (lowest first, the camera above).
    """
    polys, colours = posed_polygons(arm, q)
    if plane == "xz":
        depth = np.array([float(poly[:, 1].mean()) for poly in polys])
        flat = [poly[:, [0, 2]] for poly in polys]
        order = np.argsort(-depth)
    else:
        depth = np.array([float(poly[:, 2].mean()) for poly in polys])
        flat = [poly[:, [0, 1]] for poly in polys]
        order = np.argsort(depth)
    return [flat[k] for k in order], colours[order]


def draw_solid(ax, arm: Puma560Arm, q: np.ndarray, plane: str,
               alpha: float = 1.0, zorder: float = 2.0) -> None:
    """The shaded solid model, as flat polygons in the given plane."""
    for poly, colour in zip(*_project(arm, q, plane)):
        ax.add_patch(Polygon(poly, closed=True, facecolor=(*colour, alpha),
                             edgecolor=(*colour, alpha), linewidth=0.3, zorder=zorder))


# ----------------------------------------------------------------------
# the elevation: every link length, in the plane of the arm
# ----------------------------------------------------------------------


def draw_elevation(ax, arm: Puma560Arm, q: np.ndarray, spheres: bool = False) -> None:
    """Side view in the arm plane: links dimensioned, joints named and arced.

    ``q[0]`` is zero, so the arm lies in the plane ``y = -d3`` and the drawing
    is an honest orthographic projection of the solid model rather than a
    schematic redrawn by hand.
    """
    points = arm.joint_positions(q)[:, [0, 2]]
    foot, shoulder, elbow, wrist, tip = (points[k] for k in range(5))
    q2, q3, q4 = (math.degrees(float(q[k])) for k in (1, 2, 3))
    upper_deg, fore_deg, tool_deg = q2, q2 + q3, q2 + q3 + q4

    # -- the floor, and a machine under the tool ------------------------
    ax.axhline(0.0, color="#4a4f57", linewidth=1.1, zorder=1)
    ax.add_patch(Rectangle((-0.55, -0.045), 1.75, 0.045, facecolor="none",
                           edgecolor="#9aa0a8", hatch="////", linewidth=0.0, zorder=1))
    port_height = float(tip[1])
    ax.add_patch(Rectangle((tip[0] - 0.09, 0.0), 0.18, port_height, facecolor=MACHINE,
                           edgecolor="#22344f", alpha=0.45, linewidth=1.0, zorder=1.2))
    ax.text(tip[0], 0.5 * port_height, "machine\n(benchtop,\n0.10-0.40 m tall)",
            ha="center", va="center", fontsize=7.0, color="#1b2c48", zorder=1.3)

    # -- the envelope the wrist can reach, about the shoulder axis -------
    # Named in the legend rather than labelled on the arc: both arcs run off
    # the top of the panel, where a rotated label would land on the title.
    tightest = min(abs(math.degrees(arm.limits.q_min[2])),
                   abs(math.degrees(arm.limits.q_max[2])))
    handles = []
    for radius, style, note in (
        (arm.max_reach, (0, (6, 4)),
         f"wrist envelope, elbow straight: $a_2+L_3$ = {arm.max_reach:.4f} m"),
        (arm.min_reach, (0, (2, 3)),
         f"inner limit, elbow folded to {tightest:.2f}\u00b0: {arm.min_reach:.4f} m"),
    ):
        ax.add_patch(Arc(shoulder, 2 * radius, 2 * radius, theta1=-62.0, theta2=24.0,
                         edgecolor="#2ca02c", linestyle=style, linewidth=1.0,
                         alpha=0.75, zorder=1.4))
        handles.append(plt.Line2D([], [], color="#2ca02c", linestyle=style,
                                  linewidth=1.0, label=note))

    # -- the robot -------------------------------------------------------
    draw_solid(ax, arm, q, "xz", zorder=2.0)
    if spheres:
        chain = collision_spheres(arm)
        centres = np.atleast_2d(arm.points_along_arm(q, chain.coefficients))
        for (x, _y, z), radius in zip(centres, chain.radii):
            ax.add_patch(Circle((x, z), radius, fill=False, edgecolor="#d62728",
                                linewidth=0.55, alpha=0.5, zorder=2.5))
        handles.append(plt.Line2D([], [], color="#d62728", linewidth=0.8, alpha=0.6,
                                  label=f"{chain.radii.size} bounding spheres: "
                                        "what the 5 mm clearance test sees"))

    # The kinematic chain the lengths are measured along.
    ax.plot(points[1:, 0], points[1:, 1], "-", color=CENTRE, linewidth=1.1,
            alpha=0.9, zorder=4)
    ax.plot([foot[0], shoulder[0]], [foot[1], shoulder[1]], linestyle=(0, (6, 3)),
            color=CENTRE, linewidth=1.0, alpha=0.75, zorder=4)
    for point in (shoulder, elbow, wrist):
        ax.plot(*point, "o", markersize=6.5, markerfacecolor="white",
                markeredgecolor=CENTRE, markeredgewidth=1.3, zorder=6)
    ax.plot(*tip, "o", markersize=6.0, color=DIM, zorder=6)

    # -- the four link lengths -------------------------------------------
    dimension(ax, (foot[0], 0.0), (foot[0], shoulder[1]),
              f"$d_1$ = {arm.d1:.4f} m", offset=0.46)
    dimension(ax, shoulder, elbow, f"$a_2$ = {arm.a2:.4f} m", offset=0.085)
    dimension(ax, elbow, wrist, f"$L_3$ = {arm.L3:.4f} m", offset=-0.090)
    dimension(ax, wrist, tip, f"tool = {arm.tool:.3f} m", offset=0.075)

    # -- what each joint angle is measured from --------------------------
    ray(ax, shoulder, 0.0, 0.30)
    ray(ax, elbow, upper_deg, 0.26)
    ray(ax, wrist, fore_deg, 0.20)
    angle_arc(ax, shoulder, 0.175, 0.0, upper_deg, f"$q_2$ = {q2:+.1f}\u00b0",
              label_radius=1.45, label_deg=-26.0)
    angle_arc(ax, elbow, 0.150, upper_deg, fore_deg, f"$q_3$ = {q3:+.1f}\u00b0",
              label_radius=1.45, label_deg=-14.0)
    angle_arc(ax, wrist, 0.115, fore_deg, tool_deg, f"$q_4$ = {q4:+.1f}\u00b0",
              label_radius=1.70, label_deg=-142.0)

    # -- names on the parts ----------------------------------------------
    for a, b, text, angle, shift in (
        (shoulder, elbow, "upper arm  (frame 2)", upper_deg, 0.0),
        (elbow, wrist, "forearm  (frame 3)", fore_deg, 0.0),
        (foot, shoulder, "pedestal + column", 90.0, -0.055),
    ):
        mid = 0.5 * (np.asarray(a) + np.asarray(b))
        label_angle = angle - 180.0 if angle > 90.0 else angle
        ax.text(mid[0] + shift, mid[1], text, fontsize=7.8, color=CENTRE,
                ha="center", va="center", rotation=label_angle,
                rotation_mode="anchor", zorder=6,
                bbox=dict(boxstyle="round,pad=0.18", facecolor="white",
                          edgecolor="none", alpha=0.80))

    callout(ax, (foot[0], 0.52),
            "waist  $q_1$  (Puma J1)\nturns about this axis; see the plan",
            (-0.13, 0.0), ha="right")
    callout(ax, shoulder,
            "shoulder  $q_2$  (Puma J2)\npitch axis perpendicular to the page",
            (-0.36, 0.21))
    callout(ax, elbow, "elbow  $q_3$  (Puma J3)", (-0.31, 0.17), ha="right")
    callout(ax, wrist, "wrist pitch  $q_4$  (Puma J5)\nrolls J4 and J6 locked at 0\u00b0",
            (0.27, 0.19))
    callout(ax, tip,
            "tool tip on the loading port\n"
            f"$\\rho$ = {tip[0] - arm.base[0]:.3f} m, $z$ = {port_height:.3f} m\n"
            f"({math.hypot(tip[0] - arm.base[0], arm.d3):.3f} m from the waist axis)\n"
            f"approach pitch $q_2+q_3+q_4$ = {tool_deg:.0f}\u00b0",
            (0.21, -0.13))

    # -- the lift pose the cycle retracts to ------------------------------
    lift = MACHINE_HEIGHTS[1] + RETRACT
    ax.annotate("", xy=(tip[0], lift), xytext=(tip[0], port_height + 0.01), zorder=5,
                arrowprops=dict(arrowstyle="-|>", color="#7a3d00", linewidth=1.0,
                                linestyle=(0, (3, 2)), shrinkA=0, shrinkB=0,
                                mutation_scale=9))
    ax.plot([tip[0] - 0.07, tip[0] + 0.07], [lift, lift], color="#7a3d00",
            linewidth=1.0, zorder=5)
    ax.text(tip[0] + 0.10, lift - 0.005,
            f"lift pose: {RETRACT:.2f} m above the\ntallest machine in the cell",
            fontsize=7.0, color="#7a3d00", ha="left", va="center", zorder=6)

    ax.legend(handles=handles, loc="lower right", fontsize=7.0, framealpha=0.92,
              borderpad=0.6, handlelength=2.6)
    ax.set_xlim(-0.55, 1.28)
    ax.set_ylim(-0.07, 1.06)
    ax.set_aspect("equal")
    ax.set_xlabel("$\\rho$: distance from the waist axis, measured in the plane of "
                  "the arm (m)")
    ax.set_ylabel("height above the floor (m)")
    ax.grid(alpha=0.18, linewidth=0.5)
    ax.set_title("Elevation in the arm plane, from the solid model at a real pose "
                 "($q_1$ = 0, shoulder right / elbow up)", fontsize=9.5)


def draw_elbow_detail(ax, arm: Puma560Arm) -> None:
    """Why the forearm is one link: the ``a3`` / ``d4`` right triangle."""
    psi = arm.psi3
    elbow = np.array([0.0, 0.0])
    wrist = np.array([arm.L3, 0.0])
    corner = arm.d4 * np.array([math.cos(psi), math.sin(psi)])

    ax.plot(*zip(elbow, corner), color="#6a7079", linewidth=1.0, linestyle=(0, (4, 2)))
    ax.plot(*zip(corner, wrist), color="#6a7079", linewidth=1.0, linestyle=(0, (4, 2)))
    ax.plot(*zip(elbow, wrist), color=CENTRE, linewidth=2.0)
    for point in (elbow, wrist):
        ax.plot(*point, "o", markersize=5.0, markerfacecolor="white",
                markeredgecolor=CENTRE, markeredgewidth=1.2)

    ax.text(*(0.62 * corner + np.array([0.0, 0.008])),
            f"$d_4$ = {arm.d4:.4f} m  (DH: along the wrist axis)",
            fontsize=7.0, color="#4a4f57", ha="center", va="bottom")
    ax.annotate(f"$a_3$ = {arm.a3:.4f} m", xy=0.5 * (corner + wrist),
                xytext=(arm.L3 - 0.055, 0.052), fontsize=7.0, color="#4a4f57",
                ha="center", va="bottom",
                arrowprops=dict(arrowstyle="-", color="#9aa0a8", linewidth=0.7))
    ax.text(0.5 * arm.L3, -0.012, f"$L_3=\\sqrt{{a_3^2+d_4^2}}$ = {arm.L3:.4f} m",
            fontsize=7.4, color=CENTRE, ha="center", va="top")
    ax.add_patch(Arc(elbow, 0.22, 0.22, theta1=0.0, theta2=math.degrees(psi),
                     edgecolor=ANGLE, linewidth=1.0))
    ax.annotate(f"$\\psi_3$ = {math.degrees(psi):.4f}\u00b0", xy=(0.112, 0.0026),
                xytext=(0.075, 0.045), fontsize=7.2, color=ANGLE,
                ha="center", va="bottom",
                arrowprops=dict(arrowstyle="-", color=ANGLE, linewidth=0.7))
    ax.text(0.0, -0.03, "elbow", fontsize=7.0, color=CENTRE, ha="center", va="top")
    ax.text(arm.L3, -0.03, "wrist", fontsize=7.0, color=CENTRE, ha="center", va="top")

    ax.set_xlim(-0.035, arm.L3 + 0.045)
    ax.set_ylim(-0.070, 0.082)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    for spine in ax.spines.values():
        spine.set_edgecolor("#c8ccd2")
    ax.set_title("elbow detail, to scale: elbow $\\to$ wrist is one rigid link",
                 fontsize=7.8, pad=3)


# ----------------------------------------------------------------------
# the plan: the shoulder offset, the waist travel, the workable annulus
# ----------------------------------------------------------------------


def draw_plan(ax, arm: Puma560Arm, q: np.ndarray) -> None:
    """Top view: ``d3``, the waist travel, and the band the arm can serve."""
    base = np.asarray(arm.base, dtype=float)

    ax.add_patch(Rectangle((-CELL_HALF, -CELL_HALF), 2 * CELL_HALF, 2 * CELL_HALF,
                           fill=False, edgecolor="#333333", linewidth=1.3, zorder=2))

    # The band a top-down pick is possible in, at the shortest and the tallest
    # machine in the family -- swept, not assumed: joint travel, the annulus
    # and sigma_min each bind somewhere.
    low_in, low_out = reachable_radii(arm, MACHINE_HEIGHTS[0], SIGMA_MIN)
    high_in, high_out = reachable_radii(arm, MACHINE_HEIGHTS[1], SIGMA_MIN)
    ax.add_patch(Wedge(base, low_out, 0, 360, width=low_out - low_in,
                       facecolor="#2ca02c", alpha=0.10, edgecolor="none", zorder=1))
    for radius, style in ((low_in, ":"), (low_out, "--")):
        ax.add_patch(Circle(base, radius, fill=False, edgecolor="#2ca02c",
                            linestyle=style, linewidth=1.1, zorder=1.5))
    for radius in (high_in, high_out):
        ax.add_patch(Circle(base, radius, fill=False, edgecolor="#1c6b32",
                            linestyle=(0, (1, 2)), linewidth=1.0, alpha=0.8, zorder=1.5))

    envelope = math.hypot(arm.max_reach + arm.tool, arm.d3)
    ax.add_patch(Circle(base, envelope, fill=False, edgecolor="#9aa0a8",
                        linestyle=(0, (6, 4)), linewidth=1.0, zorder=1.5))

    # The waist travel, and the column machines are kept out of.
    ax.add_patch(Wedge(base, 0.33, math.degrees(arm.limits.q_min[0]),
                       math.degrees(arm.limits.q_max[0]), facecolor=ANGLE,
                       alpha=0.10, edgecolor="none", zorder=1.6))
    ax.add_patch(Circle(base, KEEP_OUT, facecolor="#bbbbbb", edgecolor="#555555",
                        alpha=0.85, zorder=2))

    draw_solid(ax, arm, q, "xy", zorder=3.0)

    # d3 puts the arm plane off the waist axis: that is what keeps the
    # shoulder singularity off the column.
    points = arm.joint_positions(q)
    ax.plot([base[0], 0.98], [base[1], base[1]], linestyle=(0, (5, 3)),
            color=GHOST, linewidth=0.9, zorder=4)
    ax.plot([base[0], 0.98], [-arm.d3, -arm.d3], linestyle=(0, (5, 3)),
            color=GHOST, linewidth=0.9, zorder=4)
    dimension(ax, (0.92, 0.0), (0.92, -arm.d3), f"$d_3$ = {arm.d3:.5f} m",
              offset=0.0, text_offset=0.0, fontsize=7.2)
    for height, text, va in ((0.014, "waist axis", "bottom"),
                             (-arm.d3 - 0.014, "plane of the arm", "top")):
        ax.text(0.78, height, text, fontsize=6.8, color="#4a4f57", ha="center",
                va=va, zorder=6,
                bbox=dict(boxstyle="round,pad=0.12", facecolor="white",
                          edgecolor="none", alpha=0.85))
    dimension(ax, (-CELL_HALF, -CELL_HALF), (CELL_HALF, -CELL_HALF),
              f"cell {2 * CELL_HALF:.2f} m square", offset=-0.07, fontsize=7.4,
              colour="#4a4f57")
    ax.annotate(f"machines kept out of a\nr = {KEEP_OUT:.2f} m column",
                xy=KEEP_OUT * np.array([math.cos(2.36), math.sin(2.36)]),
                xytext=(-1.02, 0.50), fontsize=7.0, color="#4a4f57",
                ha="left", va="center", zorder=7,
                arrowprops=dict(arrowstyle="-", color="#4a4f57", linewidth=0.7))
    ax.plot(*points[4, :2], "o", markersize=5.0, color=DIM, zorder=6)
    ax.plot(*base, "+", markersize=9, color=CENTRE, markeredgewidth=1.4, zorder=6)

    angle_arc(ax, base, 0.33, 0.0, math.degrees(arm.limits.q_max[0]),
              f"$q_1$ = {math.degrees(arm.limits.q_max[0]):.0f}\u00b0",
              label_radius=1.14, fontsize=7.6)
    ax.text(0.0, -0.40, f"waist travel\n{math.degrees(arm.limits.q_min[0]):.0f}\u00b0 to "
            f"+{math.degrees(arm.limits.q_max[0]):.0f}\u00b0", fontsize=7.2,
            color=ANGLE, ha="center", va="center", zorder=6)
    ax.annotate(f"cell corner at r = {CELL_HALF * math.sqrt(2.0):.2f} m:\nno posture reaches it",
                xy=(-CELL_HALF, -CELL_HALF), xytext=(-0.63, -0.90), fontsize=7.0,
                color="#4a4f57", ha="left", va="center", zorder=7,
                arrowprops=dict(arrowstyle="-", color="#4a4f57", linewidth=0.7))
    ax.text(0.0, 1.19,
            f"band a top-down pick is possible in, at $\\sigma_{{min}} \\geq$ "
            f"{SIGMA_MIN:g} m/rad:\n"
            f"r = {low_in:.2f}-{low_out:.2f} m onto a {MACHINE_HEIGHTS[0]:.2f} m machine "
            f"(shaded)\nr = {high_in:.2f}-{high_out:.2f} m onto a "
            f"{MACHINE_HEIGHTS[1]:.2f} m machine (dotted)",
            fontsize=7.2, color="#1c6b32", ha="center", va="center")
    ax.text(0.0, 1.00, f"grey dashes: the geometric envelope, {envelope:.3f} m,\n"
            "reached only at shoulder height",
            fontsize=7.0, color="#6a7079", ha="center", va="center")

    ax.set_xlim(-1.06, 1.06)
    ax.set_ylim(-1.06, 1.30)
    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.grid(alpha=0.18, linewidth=0.5)
    ax.set_title("Plan: the 1.4 m cell, the waist travel and the shoulder offset",
                 fontsize=9.5)


# ----------------------------------------------------------------------
# the tables
# ----------------------------------------------------------------------


def _table(ax, columns: Sequence[str], rows: Sequence[Sequence[str]],
           widths: Sequence[float], title: str, fontsize: float = 7.4) -> None:
    ax.set_axis_off()
    ax.set_title(title, fontsize=9.5, pad=6)
    table = ax.table(cellText=[list(r) for r in rows], colLabels=list(columns),
                     colWidths=list(widths), cellLoc="center", loc="upper center")
    table.auto_set_font_size(False)
    table.set_fontsize(fontsize)
    table.scale(1.0, 1.45)
    for (row, _col), cell in table.get_celld().items():
        cell.set_edgecolor("#c8ccd2")
        cell.set_linewidth(0.6)
        if row == 0:
            cell.set_facecolor("#eef1f5")
            cell.set_text_props(weight="bold")
        elif row % 2 == 0:
            cell.set_facecolor("#f8f9fb")


def draw_joint_table(ax, arm: Puma560Arm) -> None:
    """This model's four driven joints against the manufacturer's six."""
    names = ("waist (yaw)", "shoulder (pitch)", "elbow (pitch)", "wrist (pitch)")
    limits = arm.limits
    rows: List[List[str]] = []
    for k, joint in enumerate(PUMA_JOINT_INDEX):
        offset = math.degrees(PUMA_OFFSET[k])
        conversion = (f"$\\theta_{{{joint + 1}}} = q_{k + 1}$" if abs(offset) < 1e-9
                      else f"$\\theta_{{{joint + 1}}} = q_{k + 1} - {offset:.4f}\u00b0$")
        low, high = PUMA_TRAVEL[joint]
        rows.append([
            f"$q_{k + 1}$",
            names[k],
            f"J{joint + 1}",
            conversion,
            f"{math.degrees(limits.q_min[k]):+.2f} to {math.degrees(limits.q_max[k]):+.2f}",
            f"{low:+.0f} to {high:+.0f}",
            f"{math.degrees(limits.velocity[k]):.0f}",
            f"{math.degrees(limits.acceleration[k]):.0f}*",
        ])
    for joint in sorted(LOCKED_PUMA_JOINTS):
        low, high = PUMA_TRAVEL[joint]
        rows.append([
            "--", "tool roll (locked)", f"J{joint + 1}",
            f"$\\theta_{{{joint + 1}}} \\equiv 0\u00b0$", "--",
            f"{low:+.0f} to {high:+.0f}", f"{PUMA_MAX_SPEED_DEG[joint]:.0f}", "--",
        ])
    _table(
        ax,
        ("this model", "function", "Puma joint", "conversion",
         "travel, model (\u00b0)", "travel, Puma (\u00b0)",
         "$v_{max}$ (\u00b0/s)", "$a_{max}$ (\u00b0/s$^2$)"),
        rows,
        (0.085, 0.165, 0.09, 0.185, 0.145, 0.125, 0.10, 0.105),
        "Joints: four driven, two locked -- travel and rates are the manufacturer's, unrounded",
    )
    ax.text(0.5, 0.06,
            "* Unimation never published accelerations; these are representative and "
            "stay parameters (ArmLimits), not constants -- the one soft number in the model.",
            transform=ax.transAxes, fontsize=7.0, color="#4a4f57", ha="center", va="center")


def draw_dh_table(ax, arm: Puma560Arm) -> None:
    """The published standard-DH table this model is checked against."""
    theta = ("$\\theta_1$", "$\\theta_2$", "$\\theta_3$", "0 (locked)",
             "$\\theta_5$", "0 (locked)")
    rows = [
        [f"{j + 1}", theta[j], f"{d:.5f}", f"{a:.4f}",
         f"{math.degrees(alpha):+.0f}\u00b0"]
        for j, (d, a, alpha) in enumerate(PUMA_DH)
    ]
    _table(ax, ("j", "$\\theta_j$", "$d_j$ (m)", "$a_j$ (m)", "$\\alpha_j$"),
           rows, (0.10, 0.25, 0.23, 0.23, 0.19),
           "Standard DH (Unimation Puma 560), $d_1$ = "
           f"{arm.d1:.4f} m carried as the pedestal")
    ax.text(0.5, 0.055,
            "puma_dh_transform() walks this table independently; the self-test agrees\n"
            "with this model's tool pose to 4.4e-16 m over 4 000 random configurations.",
            transform=ax.transAxes, fontsize=7.0, color="#4a4f57", ha="center", va="center")


# ----------------------------------------------------------------------
# the figure
# ----------------------------------------------------------------------


def pick_pose(arm: Puma560Arm) -> np.ndarray:
    """The configuration the elevation is drawn in, solved rather than typed.

    A top-down pick at :data:`PICK_TARGET`, in the arm plane so that ``q1``
    comes out zero and the elevation is a true projection.
    """
    radius, height = PICK_TARGET
    target = np.array([[arm.base[0] + radius, arm.base[1] - arm.d3, height]])
    q, ok = arm.ik_vertical(target, *PICK_BRANCH)
    if not bool(ok[0]) or not bool(arm.within_limits(q)[0]):
        raise ValueError(f"the diagram pose {PICK_TARGET} is not a legal pick")
    return q[0]


def figure_arm_model(path: str, arm: Puma560Arm | None = None,
                     spheres: bool = False) -> str:
    """Write ``arm_model.png``: the whole robot model on one page."""
    arm = arm or Puma560Arm()
    q = pick_pose(arm)

    fig = plt.figure(figsize=(17.0, 11.2))
    grid = fig.add_gridspec(2, 3, width_ratios=[1.35, 1.0, 1.25],
                            height_ratios=[1.62, 1.0], hspace=0.20, wspace=0.16)
    ax_side = fig.add_subplot(grid[0, 0:2])
    ax_plan = fig.add_subplot(grid[0, 2])
    ax_joints = fig.add_subplot(grid[1, 0:2])
    ax_dh = fig.add_subplot(grid[1, 2])

    draw_elevation(ax_side, arm, q, spheres=spheres)
    draw_elbow_detail(ax_side.inset_axes([0.655, 0.735, 0.335, 0.235]), arm)
    draw_plan(ax_plan, arm, q)
    draw_joint_table(ax_joints, arm)
    draw_dh_table(ax_dh, arm)

    fig.suptitle("The robot this study optimises against: a 4-DOF Puma 560 "
                 "(J4 and J6 locked), on its published geometry", fontsize=13.5, y=0.975)
    fig.text(0.5, 0.945,
             f"$a_2$ = {arm.a2:.4f}   $a_3$ = {arm.a3:.4f}   $d_1$ = {arm.d1:.4f}   "
             f"$d_3$ = {arm.d3:.5f}   $d_4$ = {arm.d4:.4f}   "
             f"$L_3$ = {arm.L3:.4f}   tool = {arm.tool:.3f} m   "
             f"|   $\\psi_3$ = {math.degrees(arm.psi3):.4f}\u00b0   "
             f"|   reach {arm.min_reach:.3f}-{arm.max_reach:.3f} m at the wrist   "
             f"|   $\\sigma_{{min}} \\geq$ {SIGMA_MIN:g} m/rad   "
             "|   5 mm clearance to any machine",
             fontsize=9.0, color="#3a3f47", ha="center", va="center")

    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    fig.savefig(path, dpi=DPI, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    return path
