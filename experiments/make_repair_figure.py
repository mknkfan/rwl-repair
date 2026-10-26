"""How the placement repair works, on one real mutated child.

    python -m experiments.make_repair_figure
    python -m experiments.make_repair_figure --seed 418 --out results/model/repair_example_418.png

Takes the best layout of a converged ``repair`` run on test_1, mutates one
machine exactly as :meth:`ConfigurableGA.mutate` does (Gaussian jitter of the
centre, sigma = 0.1 of the gene span, rotation re-drawn uniformly), clips the
centres into the gene box as :meth:`apply_bounds` does, and then runs the
library's own :func:`repair_placement` with ``max_iters = 1, 2, ...`` -- the
loop is deterministic, so the k-th call is the state after round k.  Nothing
in the figure is a redrawing of the algorithm; every panel is its output.

Panels: the parent, the mutated child with its faults, the state after each
repair round, and the result.  Red is a fault: a footprint inside the
robot's keep-out disc, or two footprints closer than the clearance.  Arrows
are the move since the previous panel; the dot is the loading port, which
does not turn because the repair never touches rotation.

Prints a trace of the faults per round and the cycle time of the parent and
the repaired child.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from evaluator import CycleTimeEvaluator  # noqa: E402
from geometry import (  # noqa: E402
    Machine, keep_out_shortfall, layout_is_placeable, overlap_depth, repair_placement,
)
from problems import build_instances  # noqa: E402

DEFAULT_OUT = os.path.join(results_dir("model"), "repair_example.png")
DEFAULT_PARENT = os.path.join(
    results_dir("cx08_mut02_r20_g300"), "raw", "repair_test_1_run0.json"
)
MUTATION_SIGMA = 0.10
MARGIN = 0.005  # GAConfig.repair_margin
#: Moves smaller than this (metres) are not drawn: the one-round 1.3 mm nudges
#: of uninvolved machines are real, but a ghost for each is clutter.
MIN_SHOWN = 0.002


def mutate_one(instance, parent: np.ndarray, seed: int) -> Tuple[np.ndarray, int]:
    """One machine moved and turned, as ``ConfigurableGA.mutate`` would."""
    lower, upper = instance.gene_bounds()
    span = (upper - lower).reshape(-1, 3)
    rng = np.random.default_rng(seed)
    child = parent.copy()
    k = int(rng.integers(len(instance.machines)))
    child[k, 0] += rng.normal(0.0, MUTATION_SIGMA * span[k, 0])
    child[k, 1] += rng.normal(0.0, MUTATION_SIGMA * span[k, 1])
    child[k, 2] = rng.random() * 360.0
    box_lo, box_hi = lower.reshape(-1, 3)[:, :2], upper.reshape(-1, 3)[:, :2]
    child[:, :2] = np.clip(child[:, :2], box_lo, box_hi)
    return child, k


def faults(instance, machines: Sequence[Machine]) -> Tuple[Dict[int, float], Dict[Tuple[int, int], float]]:
    """Violations of the placement test itself (no repair margin), in metres."""
    column = {
        i: s for i, m in enumerate(machines)
        if (s := keep_out_shortfall(m, instance.base_xy, instance.keep_out_radius)) > 0.0
    }
    overlap = {}
    for i in range(len(machines)):
        for j in range(i + 1, len(machines)):
            depth, _axis = overlap_depth(machines[i], machines[j], instance.clearance)
            if depth > 0.0:
                overlap[(i, j)] = depth
    return column, overlap


def _describe(instance, machines) -> str:
    column, overlap = faults(instance, machines)
    parts = [f"M{i + 1} in column by {1000 * s:.1f} mm" for i, s in column.items()]
    parts += [f"M{i + 1}/M{j + 1} overlap {1000 * d:.1f} mm" for (i, j), d in overlap.items()]
    return "; ".join(parts) if parts else "no faults"


def draw_panel(ax, instance, machines, previous, title, view) -> None:
    from matplotlib.patches import Circle, Polygon

    column, overlap = faults(instance, machines)
    bad = set(column) | {i for pair in overlap for i in pair}

    min_x, max_x, min_y, max_y = instance.bounds
    ax.add_patch(Polygon([[min_x, min_y], [max_x, min_y], [max_x, max_y], [min_x, max_y]],
                         closed=True, fill=False, lw=1.5, ec="0.2"))
    ax.add_patch(Circle(instance.base_xy, instance.keep_out_radius,
                        fc="#f2c94c" if column else "0.88", ec="0.45", lw=1.0, ls="--", alpha=0.7))
    ax.plot(*instance.base_xy, marker="+", color="0.2", ms=10, mew=1.5)
    ax.annotate("robot\nkeep-out", instance.base_xy, xytext=(0, -26), textcoords="offset points",
                ha="center", va="top", fontsize=7, color="0.35")

    ax.text(max_x - 0.01, view[1][1] - 0.01, "cell\nwall", ha="right", va="top", fontsize=7,
            color="0.35")

    for i, machine in enumerate(machines):
        old = previous[i] if previous is not None else None
        moved = 0.0 if old is None else math.hypot(machine.x - old.x, machine.y - old.y)
        if moved > MIN_SHOWN:
            # Where it was: a dashed ghost, so millimetre moves stay visible.
            ax.add_patch(Polygon(old.corners(), closed=True, fill=False, ec="0.35",
                                 lw=0.9, ls=(0, (3, 2))))
        corners = machine.corners()
        is_bad = i in bad
        ax.add_patch(Polygon(corners, closed=True,
                             fc="#eb5757" if is_bad else "#9bb8d9",
                             ec="#a61b1b" if is_bad else "#2f5d8a", lw=1.2, alpha=0.75))
        port = machine.port()
        ax.plot(port[0], port[1], "o", color="0.15", ms=2.5)
        label = f"M{i + 1}"
        if moved > MIN_SHOWN:
            label += f"\n{1000 * moved:.0f} mm"
        ax.text(machine.x, machine.y, label, ha="center", va="center", fontsize=7,
                weight="bold", color="white" if is_bad else "#123", linespacing=1.0)
        if moved > 0.05:  # only the mutation's jump; repair moves read off the ghosts
            ax.annotate("", xy=(machine.x, machine.y), xytext=(old.x, old.y),
                        arrowprops=dict(arrowstyle="->", color="0.1", lw=1.1,
                                        shrinkA=0, shrinkB=8))

    ax.set_xlim(*view[0])
    ax.set_ylim(*view[1])
    ax.set_aspect("equal")
    ax.set_xticks([])
    ax.set_yticks([])
    ax.set_title(title, fontsize=9, loc="left")
    ax.text(0.02, 0.02, _describe(instance, machines).replace("; ", "\n"),
            transform=ax.transAxes, fontsize=7, va="bottom",
            color="#a61b1b" if bad else "#1e6b35",
            bbox=dict(fc="white", ec="none", alpha=0.85, pad=1.5))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--parent", default=DEFAULT_PARENT,
                        help="run JSON whose best chromosome is mutated")
    parser.add_argument("--seed", type=int, default=2152, help="mutation seed")
    parser.add_argument("--out", default=DEFAULT_OUT)
    args = parser.parse_args()

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    with open(args.parent, encoding="utf-8") as handle:
        record = json.load(handle)
    instance = next(i for i in build_instances("test") if i.name == record["instance"])
    parent = np.asarray(record["chromosome"], dtype=float).reshape(-1, 3)
    child, k = mutate_one(instance, parent, args.seed)

    place = lambda genes: [m.copy_at(*row) for m, row in zip(instance.machines, genes)]  # noqa: E731
    geometry = (instance.bounds, instance.base_xy, instance.keep_out_radius, instance.clearance)
    parent_layout, child_layout = place(parent), place(child)

    rounds: List[List[Machine]] = []
    for n in range(1, 13):
        state = repair_placement(child_layout, *geometry, margin=MARGIN, max_iters=n)
        rounds.append(state)
        if layout_is_placeable(state, *geometry):
            break
    final = repair_placement(child_layout, *geometry, margin=MARGIN)

    evaluator = CycleTimeEvaluator(instance, sigma_min_threshold=0.14, dwell=0.30, retract=0.06)
    parent_out, child_out, final_out = (evaluator.evaluate(lay)
                                        for lay in (parent_layout, child_layout, final))

    print(f"{instance.name}: mutated M{k + 1} from ({parent[k, 0]:.3f}, {parent[k, 1]:.3f}, "
          f"{parent[k, 2]:.1f} deg) to ({child[k, 0]:.3f}, {child[k, 1]:.3f}, {child[k, 2]:.1f} deg)")
    print(f"  parent   : {parent_out.cycle_time:.3f} s")
    print(f"  child    : {_describe(instance, child_layout)}  -> rejected at '{child_out.rejection}'")
    for n, state in enumerate(rounds, 1):
        print(f"  round {n:<2d} : {_describe(instance, state)}")
    moves = [1000 * math.hypot(a.x - b.x, a.y - b.y) for a, b in zip(child_layout, final)]
    print("  moved    : " + ", ".join(f"M{i + 1} {d:.1f} mm" for i, d in enumerate(moves) if d > 0.05))
    print(f"  repaired : {'%.3f s' % final_out.cycle_time if final_out.feasible else 'rejected at ' + repr(final_out.rejection)}")

    every = [parent_layout, child_layout, *rounds]
    xs = np.concatenate([m.corners()[:, 0] for lay in every for m in lay] + [[instance.base_xy[0]]])
    ys = np.concatenate([m.corners()[:, 1] for lay in every for m in lay] + [[instance.base_xy[1]]])
    pad = 0.06
    view = ((min(xs.min(), -instance.keep_out_radius) - pad, xs.max() + pad),
            (min(ys.min(), -instance.keep_out_radius) - pad, max(ys.max(), instance.keep_out_radius) + pad))

    panels = [(parent_layout, None, f"(a) parent: {parent_out.cycle_time:.3f} s"),
              (child_layout, parent_layout, f"(b) child: M{k + 1} mutated, rejected")]
    previous = child_layout
    for n, state in enumerate(rounds, 1):
        tail = ""
        if n == len(rounds):
            tail = (f" -> {final_out.cycle_time:.3f} s" if final_out.feasible
                    else f" -> still rejected ({final_out.rejection})")
        panels.append((state, previous, f"({chr(ord('c') + n - 1)}) after repair round {n}{tail}"))
        previous = state

    cols = 3
    rows = math.ceil(len(panels) / cols)
    fig, axes = plt.subplots(rows, cols, figsize=(4.0 * cols, 3.9 * rows))
    for ax in axes.flat[len(panels):]:
        ax.axis("off")
    for ax, (layout, before, title) in zip(axes.flat, panels):
        draw_panel(ax, instance, layout, before, title, view)
    fig.suptitle(
        "Placement repair: clip into cell -> push off robot column -> separate overlapping pairs, "
        "repeated until placeable (rotations untouched)", fontsize=10)
    fig.tight_layout()
    out = args.out
    os.makedirs(os.path.dirname(os.path.abspath(out)), exist_ok=True)
    fig.savefig(out, dpi=150)
    plt.close(fig)
    print(f"  -> {out}")


if __name__ == "__main__":
    main()
