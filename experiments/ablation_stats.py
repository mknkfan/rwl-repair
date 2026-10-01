"""Paired statistics for the crossover x mutation ablation.

    python -m experiments.ablation_stats
    python -m experiments.ablation_stats --results-dir results/ablation

``run_ablation`` names the cell with the best mean rank, which says which cell
came first and not whether it beat anything.  Every cell of the grid starts
from the same initial population for a given (instance, seed), so the honest
comparison is paired, and this script makes it:

* the mean-rank, mean cycle time and relative-gap grids;
* the main effect of each rate, averaged over the other;
* a Friedman test over all cells -- is there any difference at all;
* a paired Wilcoxon test of the best cell against every other cell,
  Holm-corrected over the family, with the effect size beside it.

The effect size matters as much as the p-value: with 200 matched sets a few
hundredths of a second can come out "significant" and still not matter.
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

from holm import holm  # noqa: E402
from run_repair_ab import wilcoxon_signed_rank  # noqa: E402

DEFAULT_RESULTS_DIR = results_dir("ga_ablation_retract")

Cell = Tuple[float, float]


def average_ranks(values: np.ndarray) -> np.ndarray:
    """Ranks down axis 0, 1 = smallest, ties sharing their mean rank."""
    ranks = np.empty_like(values, dtype=float)
    for col in range(values.shape[1]):
        column = values[:, col]
        order = np.argsort(column, kind="stable")
        ordered = column[order]
        i = 0
        while i < len(ordered):
            j = i
            while j + 1 < len(ordered) and ordered[j + 1] == ordered[i]:
                j += 1
            ranks[order[i:j + 1], col] = 0.5 * (i + j) + 1.0
            i = j + 1
    return ranks


def chi2_sf(x: float, df: int) -> float:
    """Upper tail of the chi-squared distribution, integer ``df``, closed form.

    Even ``df`` is a Poisson tail; odd ``df`` adds an ``erfc`` term.  Summed
    directly as a tail, so a tiny p-value is not lost to ``1 - P``.
    """
    if x <= 0:
        return 1.0
    if df % 2 == 0:
        term, total = 1.0, 1.0
        for j in range(1, df // 2):
            term *= (x / 2.0) / j
            total += term
        return min(1.0, math.exp(-x / 2.0) * total)
    term, total = 1.0, 0.0
    for j in range(df // 2):
        term = 1.0 if j == 0 else term * x / (2 * j + 1)
        total += term
    tail = math.sqrt(2.0 * x / math.pi) * math.exp(-x / 2.0) * total if df > 1 else 0.0
    return min(1.0, math.erfc(math.sqrt(x / 2.0)) + tail)


def friedman(values: np.ndarray) -> Tuple[float, float]:
    """``(chi2, p)`` for ``k`` treatments (rows) over ``n`` blocks (columns)."""
    k, n = values.shape
    ranks = average_ranks(values)
    statistic = 12.0 / (n * k * (k + 1)) * float((ranks.sum(axis=1) ** 2).sum()) \
        - 3.0 * n * (k + 1)
    return statistic, chi2_sf(statistic, k - 1)


def load(path: str) -> Tuple[List[float], List[float], np.ndarray]:
    """``(crossover rates, mutation rates, cycle times)``.

    The times are ``(cells, matched sets)`` with cells in crossover-major
    order, both rate lists descending.  An infeasible run is ``nan``.
    """
    with open(os.path.join(path, "runs.json"), encoding="utf-8") as fh:
        runs = json.load(fh)
    runs = runs if isinstance(runs, list) else runs["runs"]
    crossover = sorted({r["crossover_rate"] for r in runs}, reverse=True)
    mutation = sorted({r["mutation_rate"] for r in runs}, reverse=True)
    sets = sorted({(r["instance"], r["run"]) for r in runs})
    table: Dict[Tuple, float] = {
        (r["crossover_rate"], r["mutation_rate"], r["instance"], r["run"]):
            math.nan if r["cycle_time"] is None else float(r["cycle_time"])
        for r in runs
    }
    times = np.array([
        [table.get((c, m) + key, math.nan) for key in sets]
        for c in crossover for m in mutation
    ])
    return crossover, mutation, times


def print_grid(title: str, values: Sequence[float], crossover: Sequence[float],
               mutation: Sequence[float], spec: str) -> None:
    print(f"\n{title}")
    print("cx \\ mut" + "".join(f"{m:>9g}" for m in mutation))
    for a, c in enumerate(crossover):
        row = values[a * len(mutation):(a + 1) * len(mutation)]
        print(f"{c:<8g}" + "".join(f"{v:>9{spec}}" for v in row))


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--results-dir", default=DEFAULT_RESULTS_DIR,
                        help="an ablation tree written by run_ablation.py")
    args = parser.parse_args()

    crossover, mutation, times = load(args.results_dir)
    cells: List[Cell] = [(c, m) for c in crossover for m in mutation]
    infeasible = np.isnan(times).any(axis=0)
    if infeasible.any():
        print(f"Dropping {int(infeasible.sum())} matched sets with an infeasible run "
              "in some cell: a paired test needs every cell present.")
    times = times[:, ~infeasible]
    print(f"{len(cells)} cells x {times.shape[1]} matched (instance, seed) sets")

    ranks = average_ranks(times)
    mean_rank = ranks.mean(axis=1)
    gap = (times / times.min(axis=0) - 1.0) * 100.0
    print_grid("Mean rank (1 = best)", mean_rank, crossover, mutation, ".2f")
    print_grid("Mean cycle time (s)", times.mean(axis=1), crossover, mutation, ".3f")
    print_grid("Mean % slower than the best cell of the same set", gap.mean(axis=1),
               crossover, mutation, ".2f")

    n_mut = len(mutation)
    print("\nMain effects (mean rank, averaged over the other rate):")
    print("  mutation  " + "  ".join(
        f"{m:g}: {mean_rank[b::n_mut].mean():.2f}" for b, m in enumerate(mutation)))
    print("  crossover " + "  ".join(
        f"{c:g}: {mean_rank[a * n_mut:(a + 1) * n_mut].mean():.2f}"
        for a, c in enumerate(crossover)))

    statistic, p = friedman(times)
    shown = f"{p:.2e}" if p > 0 else "< 1e-300"
    print(f"\nFriedman over all {len(cells)} cells: chi2 = {statistic:.1f}, p = {shown}")

    best = int(np.argmin(mean_rank))
    print(f"\nBest mean rank: crossover {cells[best][0]:g}, mutation {cells[best][1]:g}. "
          "Paired Wilcoxon against every other cell, Holm-corrected:")
    rows, raw = {}, {}
    for j, cell in enumerate(cells):
        if j == best:
            continue
        diff = times[j] - times[best]
        label = f"cx {cell[0]:g} / mut {cell[1]:g}"
        _, p_value = wilcoxon_signed_rank(diff)
        raw[label] = 1.0 if p_value is None else p_value
        rows[label] = (float(diff.mean()), float(np.median(diff)), float((diff > 0).mean()))
    adjusted = holm(raw)
    print(f"{'cell':>18} {'mean diff (s)':>14} {'median (s)':>11} {'best wins':>10} {'Holm p':>10}")
    for label in sorted(rows, key=lambda key: rows[key][0]):
        mean, median, wins = rows[label]
        verdict = "" if adjusted[label] < 0.05 else "  tied"
        print(f"{label:>18} {mean:>+14.3f} {median:>+11.3f} {wins:>10.0%} "
              f"{adjusted[label]:>10.2e}{verdict}")


if __name__ == "__main__":
    main()
