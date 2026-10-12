"""The arithmetic of one repair push, drawn: shadows on the separating axes.

    python -m experiments.make_sat_figure
    python -m experiments.make_sat_figure --seed 2152 --out results/model/repair_sat.png

The companion to :mod:`make_repair_figure`.  That one shows the repair loop
round by round; this one opens up round 1 of the same example and draws the
two calculations every push is made of:

(a) **column push** -- the robot base in the machine's own frame, the nearest
    footprint point (a clamp into the half-sizes), the gap, and the shortfall
    ``R + margin - gap`` the machine is pushed out by, radially;
(b) **shadows** -- each footprint projected onto the four edge normals of the
    pair (the separating axis theorem).  Each axis is drawn as a line beside
    the boxes with both shadows on it;
(c) **push per axis** -- ``forward = hi_A + tol - lo_B``,
    ``backward = hi_B + tol - lo_A``, ``depth = min``; the axis with the
    smallest depth wins;
(d) **the split** -- each machine moves ``depth / 2`` along the winning axis,
    leaving the shadows exactly ``tol`` apart on it.

The numbers are recomputed here with the same formulas as
:func:`geometry.repair_placement`, and checked against the library's
:func:`overlap_depth` and :func:`keep_out_shortfall` before anything is drawn.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from geometry import keep_out_shortfall, overlap_depth  # noqa: E402
from make_repair_figure import DEFAULT_PARENT, MARGIN, mutate_one  # noqa: E402
from problems import build_instances  # noqa: E402

DEFAULT_OUT = os.path.join(results_dir("model"), "repair_sat.png")

BLUE, BLUE_EDGE = "#9bb8d9", "#2f5d8a"
ORANGE, ORANGE_EDGE = "#f4b183", "#b85c1e"
WIN = "#1e6b35"
FRAME_X, FRAME_Y = "#6a1b9a", "#00838f"  # machine-local x and y


def edge_axes(machine):
    """The machine's two edge normals: its local x and y in world frame."""
    t = math.radians(machine.rotation)
    return np.array([math.cos(t), math.sin(t)]), np.array([-math.sin(t), math.cos(t)])


def shadow(corners, axis):
    p = corners @ axis
    return float(p.min()), float(p.max())


def sat_table(a, b, tol):
    """One row per axis: name, axis, both shadows, forward, backward, depth, oriented axis."""
    rows = []
    names = ("M{a} local x", "M{a} local y", "M{b} local x", "M{b} local y")
    axes = (*edge_axes(a["m"]), *edge_axes(b["m"]))
    for name, axis in zip(names, axes):
        lo_a, hi_a = shadow(a["corners"], axis)
        lo_b, hi_b = shadow(b["corners"], axis)
        forward, backward = hi_a + tol - lo_b, hi_b + tol - lo_a
        depth = min(forward, backward)
        rows.append(dict(name=name.format(a=a["n"], b=b["n"]), axis=axis,
                         a=(lo_a, hi_a), b=(lo_b, hi_b), forward=forward,
                         backward=backward, depth=depth,
                         push=axis if forward <= backward else -axis))
    return rows


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--parent", default=DEFAULT_PARENT)
    parser.add_argument("--seed", type=int, default=2152)
    parser.add_argument("--out", default=DEFAULT_OUT)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon

    with open(args.parent, encoding="utf-8") as handle:
        record = json.load(handle)
    instance = next(i for i in build_instances("test") if i.name == record["instance"])
    parent = np.asarray(record["chromosome"], dtype=float).reshape(-1, 3)
    child, k = mutate_one(instance, parent, args.seed)
    machines = [m.copy_at(*row) for m, row in zip(instance.machines, child)]
    R, tol = instance.keep_out_radius, instance.clearance + MARGIN
    base = np.asarray(instance.base_xy, dtype=float)

    # The mutated machine and the neighbour it fouls most deeply.
    mover = machines[k]
    other_i = max((i for i in range(len(machines)) if i != k),
                  key=lambda i: overlap_depth(machines[i], mover, instance.clearance)[0])
    other = machines[other_i]

    # -- (a) column push, as repair_placement step 2 ----------------------
    t = math.radians(mover.rotation)
    c, s = math.cos(t), math.sin(t)
    dx, dy = base - np.array([mover.x, mover.y])
    local = np.array([c * dx + s * dy, -s * dx + c * dy])
    half = np.array([mover.width / 2.0, mover.depth / 2.0])
    nearest_local = np.clip(local, -half, half)
    gap = float(np.hypot(*(local - nearest_local)))
    shortfall = R + MARGIN - gap
    assert abs(shortfall - keep_out_shortfall(mover, base, R + MARGIN)) < 1e-9
    nearest = np.array([mover.x, mover.y]) + np.array(
        [c * nearest_local[0] - s * nearest_local[1], s * nearest_local[0] + c * nearest_local[1]])
    centre0 = np.array([mover.x, mover.y])
    outward = (centre0 - base) / np.linalg.norm(centre0 - base)
    pushed = mover.copy_at(*(centre0 + outward * max(shortfall, 0.0)), mover.rotation)

    # -- (b, c) the pair, after the column push ---------------------------
    # Keep the pair in the library's (i < j) order so the signs match it.
    first, second = sorted([(other_i, other), (k, pushed)], key=lambda p: p[0])
    A = dict(n=first[0] + 1, m=first[1], corners=first[1].corners())
    B = dict(n=second[0] + 1, m=second[1], corners=second[1].corners())
    rows = sat_table(A, B, tol)
    win = min(range(4), key=lambda r: rows[r]["depth"])
    depth, push = rows[win]["depth"], rows[win]["push"]
    lib_depth, _lib_axis = overlap_depth(A["m"], B["m"], tol)
    assert abs(lib_depth - depth) < 1e-9, (lib_depth, depth)

    moved_A = A["m"].copy_at(A["m"].x - 0.5 * depth * push[0], A["m"].y - 0.5 * depth * push[1], A["m"].rotation)
    moved_B = B["m"].copy_at(B["m"].x + 0.5 * depth * push[0], B["m"].y + 0.5 * depth * push[1], B["m"].rotation)

    # ------------------------------------------------------------------
    fig = plt.figure(figsize=(17, 11.5))
    grid = fig.add_gridspec(2, 3, width_ratios=[1.0, 1.35, 1.0], height_ratios=[1.2, 1.0],
                            hspace=0.22, wspace=0.12)
    ax_col = fig.add_subplot(grid[0, 0])
    ax_sat = fig.add_subplot(grid[0, 1])
    ax_split = fig.add_subplot(grid[0, 2])
    ax_bar = fig.add_subplot(grid[1, 0:2])
    ax_text = fig.add_subplot(grid[1, 2])
    ax_text.axis("off")

    def box(ax, m, fc, ec, label, ls="-", alpha=0.8, fill=True):
        ax.add_patch(Polygon(m.corners(), closed=True, fc=fc if fill else "none", ec=ec,
                             lw=1.4, ls=ls, alpha=alpha))
        if label:
            ax.text(m.x, m.y, label, ha="center", va="center", fontsize=9, weight="bold")

    def frame(ax, m, label):
        """The machine's own x/y axes from its centre, with its name and rotation.

        Local x runs along the width, local y along the depth; rotation is
        measured from world +x to local +x, counter-clockwise.
        """
        ux, uy = edge_axes(m)
        centre = np.array([m.x, m.y])
        for u, half_len, name, col in ((ux, m.width / 2, "x", FRAME_X), (uy, m.depth / 2, "y", FRAME_Y)):
            tip = centre + u * half_len * 0.85
            ax.annotate("", xy=tip, xytext=centre,
                        arrowprops=dict(arrowstyle="-|>", color=col, lw=1.8, shrinkA=0, shrinkB=0,
                                        mutation_scale=11))
            ax.text(*(centre + u * half_len * 1.0), name, color=col, fontsize=9, weight="bold",
                    ha="center", va="center")
        ax.plot(*centre, "o", color="0.1", ms=3)
        # Name in the quadrant between -x and -y, clear of both arrows.
        at = centre - 0.45 * (ux * m.width / 2 + uy * m.depth / 2)
        ax.text(*at, f"{label}\n{m.rotation:.1f}°", ha="center", va="center", fontsize=8.5,
                weight="bold", linespacing=1.0)

    def world_key(ax, corner=(0.04, 0.05), size=0.09):
        """World axes key in axes-fraction coordinates; z is out of the page."""
        x0, y0 = corner
        for (dx, dy), name in (((size, 0.0), "world x"), ((0.0, size * 1.3), "world y")):
            ax.annotate("", xy=(x0 + dx, y0 + dy), xytext=(x0, y0), xycoords="axes fraction",
                        arrowprops=dict(arrowstyle="-|>", color="0.25", lw=1.2, mutation_scale=9))
            ax.text(x0 + dx * 1.08 + (0.005 if dx else 0.0), y0 + dy * 1.05 + (0.0 if dx else 0.01),
                    name, transform=ax.transAxes, fontsize=7.5, color="0.25",
                    ha="left" if dx else "center", va="center" if dx else "bottom")
        ax.plot([x0], [y0], "o", transform=ax.transAxes, color="0.25", ms=6, mfc="white")
        ax.plot([x0], [y0], ".", transform=ax.transAxes, color="0.25", ms=3)
        ax.text(x0 - 0.012, y0 - 0.012, "z (up, out of page)", transform=ax.transAxes,
                fontsize=7, color="0.25", ha="left", va="top")

    colour = {A["n"]: (BLUE, BLUE_EDGE), B["n"]: (ORANGE, ORANGE_EDGE)}
    mover_colour = colour[k + 1]

    # (a) -----------------------------------------------------------------
    ax = ax_col
    ax.add_patch(Circle(base, R + MARGIN, fc="none", ec="0.5", lw=0.8, ls=":"))
    ax.add_patch(Circle(base, R, fc="#f2c94c", ec="0.45", lw=1.0, ls="--", alpha=0.5))
    ax.plot(*base, "+", color="0.1", ms=12, mew=1.8)
    box(ax, mover, *mover_colour, None, alpha=0.75)
    box(ax, pushed, "none", mover_colour[1], None, ls="--", fill=False)
    ax.plot(*nearest, "o", color="#a61b1b", ms=5, zorder=5)
    ax.plot([base[0], nearest[0]], [base[1], nearest[1]], color="#a61b1b", lw=1.6)
    mid = 0.5 * (base + nearest)
    ax.annotate(f"gap = {1000 * gap:.1f} mm", mid, xytext=(-8, 10), textcoords="offset points",
                fontsize=8.5, color="#a61b1b", ha="right")
    ax.annotate("", xy=centre0 + outward * shortfall, xytext=centre0,
                arrowprops=dict(arrowstyle="->", lw=1.8, color="0.1", shrinkA=0, shrinkB=0))
    ax.plot(*centre0, ".", color="0.1")
    ax.text(pushed.corners()[:, 0].max() + 0.008, pushed.y,
            f"push\n{1000 * shortfall:.1f} mm\nradially\nout", fontsize=8.5, va="center")
    ax.text(base[0], base[1] + R + 0.012, f"keep-out R = {1000 * R:.0f} mm (+{1000 * MARGIN:.0f} mm margin, dotted)",
            ha="center", fontsize=7.5, color="0.35")
    ax.text(*(np.array([mover.x, mover.y]) + np.array([0.0, -0.11])), f"M{k + 1}",
            ha="center", fontsize=9, weight="bold")
    ax.set_title(f"(a) Off the robot column: M{k + 1}\n"
                 f"shortfall = R + margin - gap = {1000 * R:.0f} + {1000 * MARGIN:.0f} - {1000 * gap:.1f} "
                 f"= {1000 * shortfall:.1f} mm", fontsize=9.5, loc="left")
    view = np.vstack([mover.corners(), pushed.corners(), base + [-R, -R], base + [R, R + 0.03]])
    ax.set_xlim(view[:, 0].min() - 0.03, view[:, 0].max() + 0.09)
    ax.set_ylim(view[:, 1].min() - 0.03, view[:, 1].max() + 0.01)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])

    # (b) -----------------------------------------------------------------
    ax = ax_sat
    box(ax, A["m"], *colour[A["n"]], None)
    box(ax, B["m"], *colour[B["n"]], None)
    frame(ax, A["m"], f"M{A['n']}")
    frame(ax, B["m"], f"M{B['n']}")
    world_key(ax)
    box_box = np.vstack([A["corners"], B["corners"]])
    every = box_box.copy()
    # Each axis is drawn as a line beside the boxes: the point at coordinate
    # s along it is s*u + o*n, whose projection onto u is s -- so a shadow
    # [lo, hi] is drawn exactly where the light would cast it.
    #
    # The four axes come in two near-parallel families when the boxes are
    # nearly square to each other; each family gets its own side of the pair
    # and its members are staggered so they do not draw on top of each other.
    families: list = []
    for r, row in enumerate(rows):
        for fam in families:
            if abs(float(rows[fam[0]]["axis"] @ row["axis"])) > 0.8:
                fam.append(r)
                break
        else:
            families.append([r])
    placement = {r: (f, slot) for f, fam in enumerate(families) for slot, r in enumerate(fam)}
    for r, row in enumerate(rows):
        # Drawn pointing right (or up, if steep), so labels sit at a free end.
        steep = abs(row["axis"][1]) > abs(row["axis"][0])
        flip = (row["axis"][1] < 0) if steep else (row["axis"][0] < 0)
        u = -row["axis"] if flip else row["axis"]
        sign = -1.0 if flip else 1.0
        f, slot = placement[r]
        n = np.array([u[1], -u[0]]) * (1.0 if f == 0 else -1.0)
        o = float((box_box @ n).max()) + 0.04 + 0.075 * slot
        a_lo, a_hi = sorted(sign * np.array(row["a"]))
        b_lo, b_hi = sorted(sign * np.array(row["b"]))
        s_lo = min(a_lo, b_lo) - 0.04
        s_hi = max(a_hi, b_hi) + 0.04
        line = lambda s, u=u, n=n, o=o: s * u + o * n  # noqa: E731
        is_win = r == win
        ax.plot(*np.array([line(s_lo), line(s_hi)]).T, color=WIN if is_win else "0.55",
                lw=1.2 if is_win else 0.8)
        ax.annotate("", xy=line(s_hi + 0.012), xytext=line(s_hi),
                    arrowprops=dict(arrowstyle="->", color=WIN if is_win else "0.55", lw=1.0))
        for (lo, hi), (fc, ec), lift in (((a_lo, a_hi), colour[A["n"]], 0.007),
                                         ((b_lo, b_hi), colour[B["n"]], -0.007)):
            p, q = line(lo) + lift * n, line(hi) + lift * n
            ax.plot(*np.array([p, q]).T, color=ec, lw=5, solid_capstyle="butt", alpha=0.9)
        both_lo, both_hi = max(a_lo, b_lo), min(a_hi, b_hi)
        if both_hi > both_lo:
            p, q = line(both_lo) - 0.018 * n, line(both_hi) - 0.018 * n
            ax.plot(*np.array([p, q]).T, color="#a61b1b", lw=2)
        # Steep axes stand side by side, so their labels lean apart: the
        # outer one (further from the boxes) to its own side, the inner inward.
        outer_left = steep and float(n[0]) < 0
        lean = ("right" if (slot == 1) == outer_left else "left") if steep else "left"
        ax.text(*line(s_hi + 0.022), f"{row['name']}\npush {1000 * row['depth']:.1f} mm",
                fontsize=8, color=WIN if is_win else "0.3", weight="bold" if is_win else "normal",
                ha=lean, va="bottom" if steep else "center")
        if is_win:
            # The light rays: every corner dropped perpendicular onto the axis.
            for corners, ec in ((A["corners"], colour[A["n"]][1]), (B["corners"], colour[B["n"]][1])):
                for corner in corners:
                    foot = line(float(corner @ u))
                    ax.plot(*np.array([corner, foot]).T, color=ec, lw=0.6, ls=":", alpha=0.9)
        for s in (s_lo, s_hi + 0.13):
            every = np.vstack([every, line(s)[None, :]])
    pad = 0.03
    ax.set_xlim(every[:, 0].min() - 0.08, every[:, 0].max() + 0.08)
    ax.set_ylim(every[:, 1].min() - pad, every[:, 1].max() + pad)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title("(b) Project both boxes onto each machine's local x and y axis (its shadow).\n"
                 "purple/teal arrows = each machine's own x/y; red = shadows overlap;\n"
                 "dotted = light rays for the winning axis",
                 fontsize=9.5, loc="left")

    # (c) -----------------------------------------------------------------
    ax = ax_bar
    for r, row in enumerate(rows):
        y = 3 - r
        base_s = min(row["a"][0], row["b"][0])
        for (lo, hi), (fc, ec), dy in ((row["a"], colour[A["n"]], 0.13), (row["b"], colour[B["n"]], -0.13)):
            ax.barh(y + dy, 1000 * (hi - lo), left=1000 * (lo - base_s), height=0.22, color=fc,
                    edgecolor=ec)
        # The side the cheaper push uses, with the required gap drawn on.
        if row["forward"] <= row["backward"]:
            start, end = row["b"][0], row["a"][1] + tol
            gap_from = row["a"][1]
        else:
            start, end = row["a"][0], row["b"][1] + tol
            gap_from = row["b"][1]
        ax.barh(y, 1000 * tol, left=1000 * (gap_from - base_s), height=0.5, color="none",
                edgecolor="0.4", hatch="///", lw=0.6)
        ax.annotate("", xy=(1000 * (end - base_s), y + 0.36), xytext=(1000 * (start - base_s), y + 0.36),
                    arrowprops=dict(arrowstyle="<->", color=WIN if r == win else "0.3", lw=1.2))
        ax.text(1000 * (0.5 * (start + end) - base_s), y + 0.42,
                f"{1000 * row['depth']:.1f} mm", ha="center", va="bottom", fontsize=8.5,
                color=WIN if r == win else "0.2", weight="bold" if r == win else "normal")
        ax.text(-12, y, row["name"] + ("  <- smallest" if r == win else ""), ha="right", va="center",
                fontsize=8.5, color=WIN if r == win else "0.2", weight="bold" if r == win else "normal")
    ax.set_yticks([])
    ax.set_ylim(-0.6, 4.0)
    ax.set_xlabel("position along the axis [mm] (from the leftmost shadow edge)", fontsize=8.5)
    ax.spines[["top", "right", "left"]].set_visible(False)
    ax.set_title(f"(c) Push needed on each axis = shadow overlap + {1000 * tol:.0f} mm gap (hatched)\n"
                 f"blue = M{A['n']} shadow, orange = M{B['n']} shadow; arrow = how far one must slide", fontsize=9.5, loc="left")

    # (d) -----------------------------------------------------------------
    ax = ax_split
    box(ax, A["m"], "none", colour[A["n"]][1], None, ls="--", fill=False)
    box(ax, B["m"], "none", colour[B["n"]][1], None, ls="--", fill=False)
    box(ax, moved_A, *colour[A["n"]], None)
    box(ax, moved_B, *colour[B["n"]], None)
    frame(ax, moved_A, f"M{A['n']}")
    frame(ax, moved_B, f"M{B['n']}")
    for old, new in ((A["m"], moved_A), (B["m"], moved_B)):
        ax.annotate("", xy=(new.x, new.y), xytext=(old.x, old.y),
                    arrowprops=dict(arrowstyle="->", lw=1.6, color="0.1", shrinkA=0, shrinkB=0))
    u = rows[win]["axis"]
    n = np.array([-u[1], u[0]])
    corners_after = np.vstack([moved_A.corners(), moved_B.corners()])
    o = float((corners_after @ n).min()) - 0.04
    line = lambda s: s * u + o * n  # noqa: E731
    sa, sb = shadow(moved_A.corners(), u), shadow(moved_B.corners(), u)
    for (lo, hi), (fc, ec) in ((sa, colour[A["n"]]), (sb, colour[B["n"]])):
        ax.plot(*np.array([line(lo), line(hi)]).T, color=ec, lw=5, solid_capstyle="butt")
    first_hi, second_lo = (sa[1], sb[0]) if sa[1] <= sb[0] else (sb[1], sa[0])
    ax.annotate("", xy=line(second_lo) - 0.012 * n, xytext=line(first_hi) - 0.012 * n,
                arrowprops=dict(arrowstyle="<->", lw=1.0, color=WIN))
    ax.text(*(line(0.5 * (first_hi + second_lo)) - 0.03 * n), f"{1000 * (second_lo - first_hi):.1f} mm",
            ha="center", va="top", fontsize=8.5, color=WIN, weight="bold")
    allp = np.vstack([corners_after, A["corners"], B["corners"], line(min(sa[0], sb[0]))[None],
                      line(max(sa[1], sb[1]))[None], (line(0.5 * (first_hi + second_lo)) - 0.06 * n)[None]])
    ax.set_xlim(allp[:, 0].min() - 0.03, allp[:, 0].max() + 0.03)
    ax.set_ylim(allp[:, 1].min() - 0.02, allp[:, 1].max() + 0.03)
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(f"(d) Split it: each moves {1000 * depth / 2:.1f} mm along the winning\n"
                 f"axis, opposite ways -> shadows exactly {1000 * tol:.0f} mm apart", fontsize=9.5, loc="left")

    # text ------------------------------------------------------------------
    r = rows[win]
    w_b, d_b = B["m"].width, B["m"].depth
    rel = (B["m"].rotation - A["m"].rotation) % 360.0
    formula = (
        "The math (round 1 of the example, metres)\n\n"
        "Shadow of a box on unit axis u:\n"
        "   [min(corner . u), max(corner . u)]\n"
        "   length = w |cos phi| + d |sin phi|\n\n"
        "Push on one axis, tol = clearance + margin\n"
        f"   = {1000 * instance.clearance:.0f} + {1000 * MARGIN:.0f} = {1000 * tol:.0f} mm:\n"
        "   forward  = hi_A + tol - lo_B\n"
        "   backward = hi_B + tol - lo_A\n"
        "   depth    = min(forward, backward)\n"
        "   (depth <= 0 on any axis -> already apart)\n\n"
        f"Winning axis: {r['name']}\n"
        f"   M{A['n']} shadow [{r['a'][0]:.4f}, {r['a'][1]:.4f}]\n"
        f"   M{B['n']} shadow [{r['b'][0]:.4f}, {r['b'][1]:.4f}]\n"
        f"   depth = {min(r['forward'], r['backward']):.4f} m = {1000 * depth:.1f} mm\n\n"
        f"Each box moves depth/2 = {1000 * depth / 2:.1f} mm, opposite ways.\n\n"
        f"Check: M{B['n']} is turned {rel:.1f} deg relative to M{A['n']}, so its\n"
        f"shadow on M{A['n']}'s local x axis is\n"
        f"   {w_b:.4f}|cos {rel:.1f}| + {d_b:.4f}|sin {rel:.1f}| = "
        f"{w_b * abs(math.cos(math.radians(rel))) + d_b * abs(math.sin(math.radians(rel))):.4f}\n\n"
        "What sets the size of a push: overlap depth, box sizes,\n"
        "both rotations, clearance + margin, keep-out radius.\n"
        "Not cycle time, and rotations are never changed."
    )
    ax_text.text(0.0, 1.0, formula, va="top", ha="left", fontsize=8.6, family="monospace")

    fig.suptitle(f"How the placement repair computes a push  ({instance.name}, mutated child, "
                 f"repair round 1: M{k + 1} off the column, then M{A['n']}/M{B['n']} separated)",
                 fontsize=11.5, y=0.985)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    plt.close(fig)

    print(f"column push M{k + 1}: gap {1000 * gap:.1f} mm, shortfall {1000 * shortfall:.1f} mm")
    for i, row in enumerate(rows):
        print(f"  {row['name']:<11s} fwd {1000 * row['forward']:7.1f}  bwd {1000 * row['backward']:7.1f}  "
              f"depth {1000 * row['depth']:6.1f} mm{'  <- smallest' if i == win else ''}")
    print(f"  -> {args.out}")


if __name__ == "__main__":
    main()
