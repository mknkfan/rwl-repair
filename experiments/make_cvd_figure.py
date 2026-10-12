"""What the penalty's ``g_i(x)`` are, drawn on one deliberately illegal layout.

    python -m experiments.make_cvd_figure
    python -m experiments.make_cvd_figure --out results/penalty/cvd_example.png

:mod:`penalty` writes every geometric rule as ``g_i(x) <= 0`` and scores an
illegal layout by ``CVD(x) = sum_i max(0, g_i(x))``.  This figure takes six
machines of test instance 1, places three of them wrong on purpose -- one past
the right wall, one inside the robot's keep-out disc, one overlapping a
neighbour -- and draws:

(a) the whole cell, every violated rule marked;
(b) the wall rule: ``g = max corner x - x_max``;
(c) the keep-out rule: ``g = R - gap`` to the nearest footprint point;
(d) the pair rule: ``g`` = the shortest push, over the four separating axes,
    that leaves the two boxes ``clearance`` apart;
(e) all 45 signed ``g_i``: satisfied rules sit below zero and count nothing,
    only the bars above zero add up to CVD.

The signed values are recomputed here with the same formulas, and the sum of
their positive parts is checked against :func:`penalty.constraint_violations`
before anything is drawn.  For a pair, ``g`` is the smallest per-axis push:
positive when every axis needs one (they overlap), and minus the widest gap
beyond ``clearance`` when some axis already separates them.
"""

from __future__ import annotations

import argparse
import math
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from geometry import _separating_axes, keep_out_shortfall, overlap_depth  # noqa: E402
from penalty import constraint_violations  # noqa: E402
from problems import build_instances  # noqa: E402

DEFAULT_OUT = os.path.join(results_dir("penalty"), "cvd_example.png")

#: (x, y, rotation) per machine: M1 past the right wall, M2 on the robot's
#: column, M3 overlapping M1; M4-M6 legal.
LAYOUT = (
    (0.66, 0.00, 0.0),
    (0.05, 0.10, 0.0),
    (0.58, 0.05, 0.0),
    (-0.45, 0.45, 0.0),
    (-0.45, -0.45, 0.0),
    (0.00, -0.50, 0.0),
)

GREY, GREY_EDGE = "#d9d9d9", "#7f7f7f"
ORANGE, ORANGE_EDGE = "#f4b183", "#b85c1e"
VIOLATION = "#a61b1b"
KIND_COLOUR = {"wall": "#2f5d8a", "keep-out": "#c9a227", "pair": "#6a1b9a"}


def signed_constraints(machines, bounds, base, radius, clearance):
    """Every ``g_i`` as ``(kind, label, value)``, in the order penalty sums them."""
    min_x, max_x, min_y, max_y = bounds
    rows = []
    for m in machines:
        c = m.corners()
        for side, g in (("left", min_x - c[:, 0].min()), ("right", c[:, 0].max() - max_x),
                        ("bottom", min_y - c[:, 1].min()), ("top", c[:, 1].max() - max_y)):
            rows.append(("wall", f"{m.name} {side}", float(g)))
        rows.append(("keep-out", f"{m.name} keep-out", float(keep_out_value(m, base, radius)[0])))
    for i in range(len(machines)):
        for j in range(i + 1, len(machines)):
            a, b = machines[i], machines[j]
            ca, cb = a.corners(), b.corners()
            pushes = []
            for axis in _separating_axes(a, b):
                pa, pb = ca @ axis, cb @ axis
                pushes.append(min(pa.max() + clearance - pb.min(), pb.max() + clearance - pa.min()))
            rows.append(("pair", f"{a.name}-{b.name}", float(min(pushes))))
    return rows


def keep_out_value(m, base, radius):
    """``(R - gap, nearest footprint point)``: the keep-out ``g`` and where it is measured."""
    t = math.radians(m.rotation)
    c, s = math.cos(t), math.sin(t)
    dx, dy = base[0] - m.x, base[1] - m.y
    local = np.array([c * dx + s * dy, -s * dx + c * dy])
    half = np.array([m.width / 2.0, m.depth / 2.0])
    near = np.clip(local, -half, half)
    gap = float(np.hypot(*(local - near)))
    world = np.array([m.x + c * near[0] - s * near[1], m.y + s * near[0] + c * near[1]])
    return radius - gap, world


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", default=DEFAULT_OUT)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.patches import Circle, Polygon, Rectangle

    instance = build_instances("test")[0]
    machines = [m.copy_at(*p) for m, p in zip(instance.machines, LAYOUT)]
    bounds, R, tol = instance.bounds, instance.keep_out_radius, instance.clearance
    base = np.asarray(instance.base_xy, dtype=float)
    min_x, max_x, min_y, max_y = bounds

    rows = signed_constraints(machines, bounds, base, R, tol)
    cvd = sum(max(0.0, g) for _, _, g in rows)
    lib_cvd, lib_n, _lib_pairs = constraint_violations(machines, bounds, base, R, tol)
    assert abs(cvd - lib_cvd) < 1e-9, (cvd, lib_cvd)
    violated = [(k, lab, g) for k, lab, g in rows if g > 0]

    m1, m2, m3 = machines[0], machines[1], machines[2]
    g_wall = m1.corners()[:, 0].max() - max_x
    g_keep, nearest = keep_out_value(m2, base, R)
    g_pair, _ = overlap_depth(m1, m3, tol)
    assert abs(g_keep - keep_out_shortfall(m2, base, R)) < 1e-12
    bad = {0, 1, 2}

    fig = plt.figure(figsize=(17, 11))
    grid = fig.add_gridspec(2, 3, width_ratios=[1.25, 1.0, 1.0], height_ratios=[1.0, 0.85],
                            hspace=0.28, wspace=0.16)
    ax_cell = fig.add_subplot(grid[0, 0])
    ax_wall = fig.add_subplot(grid[0, 1])
    ax_keep = fig.add_subplot(grid[0, 2])
    ax_pair = fig.add_subplot(grid[1, 0])
    ax_bar = fig.add_subplot(grid[1, 1:])

    def draw_machines(ax, only=None, labels=True):
        for i, m in enumerate(machines):
            if only is not None and i not in only:
                continue
            fc, ec = (ORANGE, ORANGE_EDGE) if i in bad else (GREY, GREY_EDGE)
            ax.add_patch(Polygon(m.corners(), closed=True, fc=fc, ec=ec, lw=1.4, alpha=0.75))
            if labels:
                ax.text(m.x, m.y, m.name, ha="center", va="center", fontsize=9, weight="bold")

    def draw_cell(ax):
        ax.add_patch(Rectangle((min_x, min_y), max_x - min_x, max_y - min_y,
                               fc="none", ec="0.2", lw=1.6))
        ax.add_patch(Circle(base, R, fc="#f2c94c", ec="0.45", lw=1.0, ls="--", alpha=0.45))
        ax.plot(*base, "+", color="0.1", ms=11, mew=1.8)

    def tidy(ax, xlim, ylim):
        ax.set_xlim(*xlim)
        ax.set_ylim(*ylim)
        ax.set_aspect("equal")
        ax.tick_params(labelsize=7.5)

    # (a) the whole cell ----------------------------------------------------
    ax = ax_cell
    draw_cell(ax)
    draw_machines(ax)
    ax.text(base[0], base[1] - R - 0.03, f"robot base\nkeep-out R = {1000 * R:.0f} mm",
            ha="center", va="top", fontsize=8, color="0.3")
    callouts = (
        ((m1.corners()[:, 0].max(), m1.y + 0.06), (0.42, 0.55), f"(b) M1 past the wall\ng = +{1000 * g_wall:.1f} mm"),
        ((m2.x - 0.03, m2.y + 0.06), (0.0, 0.42), f"(c) M2 inside keep-out\ng = +{1000 * g_keep:.1f} mm"),
        ((0.62, -0.04), (0.45, -0.42), f"(d) M1-M3 overlap\ng = +{1000 * g_pair:.1f} mm"),
    )
    for xy, text_xy, text in callouts:
        ax.annotate(text, xy=xy, xytext=text_xy, fontsize=8.5, color=VIOLATION, weight="bold",
                    ha="center", arrowprops=dict(arrowstyle="->", color=VIOLATION, lw=1.2))
    ax.text(-0.45, 0.02, "M4, M5, M6: every g < 0\n(all rules satisfied)",
            ha="center", fontsize=8, color="0.35")
    tidy(ax, (min_x - 0.06, max_x + 0.08), (min_y - 0.06, max_y + 0.06))
    ax.set_xlabel("x [m]", fontsize=8.5)
    ax.set_ylabel("y [m]", fontsize=8.5)
    ax.set_title(f"(a) {instance.name}: three machines placed wrong on purpose\n"
                 f"orange = breaks a rule, grey = legal; cell = +/-{max_x:.2f} m",
                 fontsize=9.5, loc="left")

    # (b) wall --------------------------------------------------------------
    ax = ax_wall
    draw_cell(ax)
    draw_machines(ax, only={0, 2})
    c1 = m1.corners()
    ax.axvspan(max_x, max_x + 0.2, color=VIOLATION, alpha=0.08)
    ax.add_patch(Polygon([[max_x, c1[:, 1].min()], [c1[:, 0].max(), c1[:, 1].min()],
                          [c1[:, 0].max(), c1[:, 1].max()], [max_x, c1[:, 1].max()]],
                         closed=True, fc="none", ec=VIOLATION, hatch="///", lw=0))
    y_dim = 0.03
    ax.annotate("", xy=(c1[:, 0].max(), y_dim), xytext=(max_x, y_dim),
                arrowprops=dict(arrowstyle="<->", color=VIOLATION, lw=1.4, shrinkA=0, shrinkB=0))
    ax.text(c1[:, 0].max() + 0.005, y_dim, f"g = +{1000 * g_wall:.1f} mm",
            ha="left", va="center", fontsize=9, color=VIOLATION, weight="bold")
    ax.text(max_x - 0.004, -0.145, f"wall x = {max_x:.2f}", ha="right",
            va="bottom", fontsize=8, color="0.2")
    ax.text(c1[:, 0].max() + 0.005, -0.06, f"M1 right edge\n= {c1[:, 0].max():.4f}", ha="left",
            va="center", fontsize=8, color=ORANGE_EDGE)
    c3 = m3.corners()
    g3 = c3[:, 0].max() - max_x
    y3 = c3[:, 1].max() - 0.02
    ax.annotate("", xy=(c3[:, 0].max(), y3), xytext=(max_x, y3),
                arrowprops=dict(arrowstyle="<->", color="0.4", lw=1.0, shrinkA=0, shrinkB=0))
    ax.text(max_x + 0.005, y3, f"M3: g = {1000 * g3:.1f} mm\n-> max(0, g) = 0",
            ha="left", va="center", fontsize=8, color="0.35")
    tidy(ax, (0.47, 0.80), (-0.15, 0.17))
    ax.set_title("(b) Wall rule, one per side per machine (4n of them)\n"
                 "g = (max corner x) - x_max   [right wall; the other three alike]",
                 fontsize=9.5, loc="left")

    # (c) keep-out ----------------------------------------------------------
    ax = ax_keep
    draw_cell(ax)
    draw_machines(ax, only={1})
    ax.plot(*nearest, "o", color=VIOLATION, ms=5, zorder=5)
    ax.plot([base[0], nearest[0]], [base[1], nearest[1]], color="0.1", lw=1.4)
    gap = R - g_keep
    ax.text(nearest[0] + 0.006, 0.5 * (base[1] + nearest[1]), f"gap = {1000 * gap:.1f} mm",
            fontsize=8.5, va="center")
    # The shortfall: from the nearest point out to the disc edge, along the same ray.
    ray = (nearest - base) / max(np.linalg.norm(nearest - base), 1e-12)
    edge = base + ray * R
    ax.plot([nearest[0], edge[0]], [nearest[1], edge[1]], color=VIOLATION, lw=3)
    ax.text(edge[0] - 0.008, 0.5 * (nearest[1] + edge[1]),
            f"g = R - gap\n= {1000 * R:.0f} - {1000 * gap:.1f}\n= +{1000 * g_keep:.1f} mm",
            ha="right", va="center", fontsize=9, color=VIOLATION, weight="bold")
    ax.text(m2.x + 0.045, m2.y - 0.055, "nearest point of\nM2 to the base", fontsize=7.5,
            color=VIOLATION, ha="left")
    tidy(ax, (-0.22, 0.22), (-0.21, 0.21))
    ax.set_title("(c) Keep-out rule, one per machine (n of them)\n"
                 "g = R - (distance from base to nearest footprint point)",
                 fontsize=9.5, loc="left")

    # (d) pair overlap ------------------------------------------------------
    ax = ax_pair
    draw_machines(ax, only={0, 2})
    ax.axvline(max_x, color="0.2", lw=1.6)
    lo1, hi3 = c1[:, 0].min(), c3[:, 0].max()
    yb = min(c1[:, 1].min(), c3[:, 1].min()) - 0.03
    ax.plot([c1[:, 0].min(), c1[:, 0].max()], [yb + 0.01, yb + 0.01], color=ORANGE_EDGE, lw=5,
            solid_capstyle="butt")
    ax.plot([c3[:, 0].min(), c3[:, 0].max()], [yb - 0.01, yb - 0.01], color=ORANGE_EDGE, lw=5,
            solid_capstyle="butt", alpha=0.6)
    ax.text(c1[:, 0].max() + 0.005, yb + 0.01, "M1 shadow on x", fontsize=7.5, va="center")
    ax.text(c3[:, 0].min() - 0.005, yb - 0.01, "M3 shadow on x", fontsize=7.5, va="center", ha="right")
    overlap = hi3 - lo1
    ax.add_patch(Rectangle((lo1, yb - 0.035), overlap, 0.012, fc=VIOLATION, ec="none"))
    ax.add_patch(Rectangle((hi3, yb - 0.035), tol, 0.012, fc="none", ec="0.4", hatch="///", lw=0.6))
    ax.annotate("", xy=(hi3 + tol, yb - 0.05), xytext=(lo1, yb - 0.05),
                arrowprops=dict(arrowstyle="<->", color=VIOLATION, lw=1.4, shrinkA=0, shrinkB=0))
    ax.text(0.5 * (lo1 + hi3 + tol), yb - 0.058,
            f"g = overlap {1000 * overlap:.1f} + clearance {1000 * tol:.0f} = +{1000 * g_pair:.1f} mm",
            ha="center", va="top", fontsize=9, color=VIOLATION, weight="bold")
    ax.text(lo1 - 0.003, yb - 0.029, "overlap", fontsize=7.5, ha="right", va="center", color=VIOLATION)
    ax.text(hi3 + tol + 0.003, yb - 0.029, "+ clearance", fontsize=7.5, ha="left", va="center",
            color="0.3")
    tidy(ax, (0.40, 0.80), (yb - 0.11, max(c1[:, 1].max(), c3[:, 1].max()) + 0.03))
    ax.set_title("(d) Pair rule, one per pair (n(n-1)/2 of them)\n"
                 "g = smallest push, over the 4 edge axes, leaving the boxes\n"
                 f"{1000 * tol:.0f} mm apart -- here along world x",
                 fontsize=9.5, loc="left")

    # (e) all 45 ------------------------------------------------------------
    ax = ax_bar
    xs = np.arange(len(rows))
    for x, (kind, label, g) in zip(xs, rows):
        ax.bar(x, g, color=VIOLATION if g > 0 else KIND_COLOUR[kind],
               alpha=1.0 if g > 0 else 0.45, width=0.8)
        if g > 0:
            ax.text(x, g + 0.006, f"+{1000 * g:.1f}", ha="center", va="bottom", fontsize=7.5,
                    color=VIOLATION, weight="bold", rotation=90)
    ax.axhline(0.0, color="0.1", lw=1.0)
    ax.set_xticks(xs)
    ax.set_xticklabels([lab for _, lab, _ in rows], rotation=90, fontsize=6.5)
    for tick, (kind, _, g) in zip(ax.get_xticklabels(), rows):
        tick.set_color(VIOLATION if g > 0 else KIND_COLOUR[kind])
        if g > 0:
            tick.set_weight("bold")
    ax.set_ylim(min(g for _, _, g in rows) - 0.05, 0.32)
    ax.set_xlim(-0.8, len(rows) - 0.2)
    ax.set_ylabel("g_i(x) [m]", fontsize=8.5)
    ax.tick_params(axis="y", labelsize=7.5)
    ax.spines[["top", "right"]].set_visible(False)
    ax.text(11, 0.14, "g > 0: violated, counts in CVD", ha="left", va="bottom", fontsize=8,
            color=VIOLATION)
    ax.text(11, 0.05, "g <= 0: satisfied, max(0, g) = 0", ha="left", va="bottom", fontsize=8,
            color="0.3")
    handles = [plt.Rectangle((0, 0), 1, 1, color=KIND_COLOUR[k], alpha=0.45) for k in KIND_COLOUR]
    handles.append(plt.Rectangle((0, 0), 1, 1, color=VIOLATION))
    ax.legend(handles, [f"{k} (4n)" if k == "wall" else f"{k} (n)" if k == "keep-out"
                        else f"{k} (n(n-1)/2)" for k in KIND_COLOUR] + ["violated"],
              fontsize=7.5, loc="upper right", ncol=2, frameon=False)
    terms = " + ".join(f"{g:.4f}" for _, _, g in violated)
    ax.set_title(f"(e) All {len(rows)} signed g_i for this layout\n"
                 f"CVD = sum max(0, g_i) = {terms} = {cvd:.4f} m\n"
                 f"penalty_cvd: F = 1000 + {cvd:.4f}   |   penalty_count: F = 1000 + {cvd:.4f} "
                 f"+ 0.05 x {lib_n} overlapping machines = {1000 + cvd + 0.05 * lib_n:.4f}",
                 fontsize=9.5, loc="left")

    fig.suptitle("The penalty's constraints: each rule is g_i(x) <= 0, and CVD adds up how far "
                 "each broken one is broken (metres)", fontsize=12, y=0.985)
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    fig.savefig(args.out, dpi=150, bbox_inches="tight")
    plt.close(fig)

    table = os.path.join(os.path.dirname(os.path.abspath(args.out)), "cvd_constraints.md")
    write_table(table, rows, instance, machines, cvd, os.path.basename(args.out))

    for kind, label, g in violated:
        print(f"  {kind:<8s} {label:<14s} g = {1000 * g:+.1f} mm")
    print(f"  CVD = {cvd:.4f} m (library {lib_cvd:.4f}), {lib_n} machines overlapping")
    print(f"  -> {args.out}")
    print(f"  -> {table}")


FORMULA = {
    "left": "x_min - (min corner x)",
    "right": "(max corner x) - x_max",
    "bottom": "y_min - (min corner y)",
    "top": "(max corner y) - y_max",
    "keep-out": "R - (distance from base to nearest footprint point)",
    "pair": "smallest push over the 4 edge axes that leaves the boxes `clearance` apart",
}


def write_table(path, rows, instance, machines, cvd, figure_name):
    """The numbered list of every ``g_i``, with this layout's values."""
    n = len(machines)
    min_x, max_x, min_y, max_y = instance.bounds
    lines = [
        "# The penalty's constraints `g_i(x) <= 0`, numbered",
        "",
        f"Written by `experiments/make_cvd_figure.py` alongside [`{figure_name}`]({figure_name}).",
        "",
        "`i` numbers the **rules**, not the violations: every layout is checked against all "
        f"{len(rows)} of them. `g_i(x) <= 0` means rule `i` holds; `g_i(x) > 0` means it is broken "
        "by that many metres. Only broken rules add to",
        "",
        "```",
        "CVD(x) = sum_i max(0, g_i(x))",
        "```",
        "",
        f"For `n` machines there are `4n + n + n(n-1)/2` rules -- {len(rows)} for `n = {n}`. The order "
        "is the order `optimizer/penalty.py::constraint_violations` sums them in: each machine's four "
        "walls and its keep-out, machine by machine, then every pair.",
        "",
        f"Constants ({instance.name}): cell `x, y` in [{min_x:.2f}, {max_x:.2f}] m, robot base at "
        f"({instance.base_xy[0]:.2f}, {instance.base_xy[1]:.2f}), keep-out `R = {instance.keep_out_radius:.2f}` m, "
        f"`clearance = {instance.clearance:.2f}` m.",
        "",
        "The last two columns are the example layout drawn in the figure (M1 past the right wall, "
        "M2 inside the keep-out, M3 overlapping M1). Negative values for a pair are minus the gap "
        "beyond `clearance`; the library itself only reports the positive part.",
        "",
        "| i | kind | rule | g_i(x) | example g_i (mm) | adds to CVD (mm) |",
        "| ---: | --- | --- | --- | ---: | ---: |",
    ]
    for i, (kind, label, g) in enumerate(rows, start=1):
        key = label.split()[-1] if kind == "wall" else kind
        broken = g > 0
        value = f"**{1000 * g:+.1f}**" if broken else f"{1000 * g:+.1f}"
        added = f"**{1000 * g:.1f}**" if broken else "0"
        lines.append(f"| {i} | {kind} | {label} | {FORMULA[key]} | {value} | {added} |")
    lines += [
        f"| | | | | **CVD** | **{1000 * cvd:.1f}** |",
        "",
    ]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


if __name__ == "__main__":
    main()
