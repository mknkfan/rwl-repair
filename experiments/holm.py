"""Holm-Bonferroni over the arm comparisons, with the resolution problem fixed.

    python -m experiments.holm
    python -m experiments.holm --reference repair --budget equal_cpu

``run_penalty_ab`` reports one p-value per comparison, which is right for a
single claim and wrong for a table of them: the headline "repair beats
everything" is three simultaneous tests and three chances to be unlucky.
Holm-Bonferroni controls the family-wise error rate under *any* dependence,
which matters because these tests share the repair arm and the same 50 matched
sets.  Benjamini-Hochberg would be more powerful and is not needed -- the
p-values are nowhere near the line.

**The resolution problem.** ``sign_flip_test`` samples 200 000 permutations
above n=20, so its smallest possible p-value is ``1/200001``, about 5e-6.  An
overwhelmingly significant comparison and a merely very significant one both
come back "0.0000" and Holm cannot tell them apart.  So this re-runs the
permutation test with a much larger sample, reports the floor explicitly, and
carries the analytic Wilcoxon p-value alongside -- which has no floor, and is
therefore the statistic the correction is applied to.

The two-sided binomial sign test on the win counts is reported as well.  It
throws away the magnitudes and is correspondingly conservative, but it is
closed-form with no approximation and no sampling, so it is the one number in
the table that cannot be argued with.
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from run_penalty_ab import (  # noqa: E402
    ARMS, equal_cpu_view, index_records,
)
from run_repair_ab import collect_records, wilcoxon_signed_rank  # noqa: E402


def sign_test(wins: int, losses: int) -> float:
    """Two-sided exact binomial p for ``wins`` out of ``wins + losses``.

    No approximation and no sampling: the null is that each pair's sign is a
    fair coin, and this is that null's exact tail.  Ties carry no information
    about direction and are dropped, which is the standard handling.
    """
    n = wins + losses
    if n == 0:
        return 1.0
    k = min(wins, losses)
    tail = sum(math.comb(n, i) for i in range(k + 1)) / (2.0 ** n)
    return float(min(1.0, 2.0 * tail))


def permutation_p(
    differences: Sequence[float], samples: int = 10_000_000, seed: int = 0
) -> Tuple[float, float, bool]:
    """``(p, floor, hit_floor)`` for the paired sign-flip test.

    Exact by enumeration up to n=22; sampled above, in chunks, so that ten
    million permutations of fifty pairs never materialise at once.  The floor
    is returned rather than hidden, because a p-value reported at its own
    resolution limit is a statement about the test and not about the data.
    """
    d = np.asarray([x for x in differences if math.isfinite(x)], dtype=float)
    n = d.size
    if n == 0:
        return 1.0, 1.0, False
    observed = abs(float(d.mean()))

    if n <= 22:
        signs = 1.0 - 2.0 * ((np.arange(2 ** n)[:, None] >> np.arange(n)) & 1)
        means = np.abs((signs * d).mean(axis=1))
        p = float((means >= observed - 1e-15).mean())
        return p, 1.0 / (2 ** n), False

    rng = np.random.default_rng(seed)
    hits = 0
    drawn = 0
    while drawn < samples:
        take = min(65_536, samples - drawn)
        signs = rng.choice((-1.0, 1.0), size=(take, n))
        hits += int((np.abs((signs * d).mean(axis=1)) >= observed - 1e-15).sum())
        drawn += take
    floor = 1.0 / (samples + 1)
    return (hits + 1) / (samples + 1), floor, hits == 0


def holm(p_values: Dict[str, float]) -> Dict[str, float]:
    """Holm-Bonferroni adjusted p-values, valid under arbitrary dependence.

    Step-down: sort ascending, multiply the k-th smallest by ``m - k``, then
    enforce monotonicity so an adjusted value can never fall below one earlier
    in the sequence.  Returned as adjusted p-values rather than as accept/reject
    at some alpha, so the reader picks the threshold.
    """
    items = sorted(p_values.items(), key=lambda kv: kv[1])
    m = len(items)
    adjusted: Dict[str, float] = {}
    running = 0.0
    for k, (key, p) in enumerate(items):
        running = max(running, min(1.0, (m - k) * p))
        adjusted[key] = running
    return adjusted


def paired_differences(view, arm: str, reference: str, metric: str) -> List[float]:
    """Signed differences, positive where ``reference`` beat ``arm``.

    Named for what it is: the reference is the arm whose superiority is being
    claimed, so a positive difference supports the claim.
    """
    out = []
    for arms in view.values():
        a, b = arms[reference].get(metric), arms[arm].get(metric)
        if a is None or b is None:
            continue
        a, b = float(a), float(b)
        if not (math.isfinite(a) and math.isfinite(b)):
            continue
        out.append(b - a)          # arm minus reference: positive = reference better
    return out


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--output", default=results_dir("penalty"))
    parser.add_argument("--reference", default="repair")
    parser.add_argument("--budget", default="both",
                        choices=("equal_generations", "equal_cpu", "both"))
    parser.add_argument("--samples", type=int, default=10_000_000)
    parser.add_argument("--family", default="per-budget",
                        choices=("per-budget", "all"),
                        help="correct within each budget, or over both at once")
    args = parser.parse_args()

    records = collect_records(args.output)
    by_key = index_records(records)
    if not by_key:
        print("no complete matched sets on disk")
        return
    views = {"equal_generations": by_key, "equal_cpu": equal_cpu_view(by_key)}
    budgets = list(views) if args.budget == "both" else [args.budget]

    print(f"{len(by_key)} matched sets, {len(ARMS)} arms: {', '.join(ARMS)}")
    print(f"reference = {args.reference}; positive gain means {args.reference} was faster\n")

    raw: Dict[str, Dict[str, float]] = {}
    rows: Dict[str, List[Dict]] = {}

    for budget in budgets:
        view = views[budget]
        rows[budget] = []
        for arm in ARMS:
            if arm == args.reference:
                continue
            d = paired_differences(view, arm, args.reference, "cycle_time")
            if not d:
                continue
            d = np.asarray(d, dtype=float)
            perm, floor, at_floor = permutation_p(d, samples=args.samples)
            _, wilcoxon = wilcoxon_signed_rank(d)
            wins = int((d > 0).sum())
            losses = int((d < 0).sum())
            rows[budget].append({
                "arm": arm, "n": d.size,
                "gain": float(d.mean()), "median": float(np.median(d)),
                "wins": wins, "losses": losses,
                "perm": perm, "perm_floor": floor, "at_floor": at_floor,
                "wilcoxon": wilcoxon, "sign": sign_test(wins, losses),
            })
            raw[f"{budget}:{arm}"] = wilcoxon if wilcoxon is not None else perm

    if args.family == "all":
        adjusted = holm(raw)
    else:
        adjusted = {}
        for budget in budgets:
            subset = {k: v for k, v in raw.items() if k.startswith(f"{budget}:")}
            adjusted.update(holm(subset))

    family = len(raw) if args.family == "all" else len(rows[budgets[0]])
    for budget in budgets:
        print(f"--- {args.reference} vs each arm, {budget.replace('_', ' ')} "
              f"(Holm family size {family}) ---")
        print(f"{'arm':<15} {'n':>3} {'gain s':>8} {'median':>8} {'win/loss':>9} "
              f"{'perm p':>12} {'Wilcoxon p':>12} {'sign p':>10} {'Holm p':>12}")
        for row in rows[budget]:
            perm = (f"<{row['perm_floor']:.1e}" if row["at_floor"]
                    else f"{row['perm']:.2e}")
            adj = adjusted[f"{budget}:{row['arm']}"]
            print(f"{row['arm']:<15} {row['n']:>3} {row['gain']:>+8.4f} "
                  f"{row['median']:>+8.4f} {row['wins']:>4}/{row['losses']:<4} "
                  f"{perm:>12} "
                  f"{'--' if row['wilcoxon'] is None else format(row['wilcoxon'], '.2e'):>12} "
                  f"{row['sign']:>10.2e} {adj:>12.2e}")
        print()

    print(f"perm p from {args.samples:,} sign-flips; '<' marks the sampling floor, "
          "where no permutation out of that many\nreached the observed mean -- the "
          "data cannot distinguish it from smaller still.  Holm is applied to the\n"
          "Wilcoxon p, which is analytic and has no floor.  The sign test is exact "
          "and ignores magnitudes.")


if __name__ == "__main__":
    main()
