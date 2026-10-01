"""The equal-compute control for the placement-repair A/B.

    python -m experiments.equal_cpu_check

``run_repair_ab`` gives both arms the same number of evaluations, and repair
wins on that budget.  The budget is the problem: the placement test throws out
an infeasible layout in about 0.36 ms, while a feasible one runs IK, the
manipulability check and the swept-path collision test over four branches for
about 6.1 ms.  Repair works by turning cheap evaluations into expensive ones,
so at equal evaluation count it quietly draws about 1.7x the compute.

This moves the handicap to the other side: the repair arm is truncated to the
generation count whose CPU matches what the baseline spent over its full 300
(174, from the measured ratio) and compared against the *full-length*
baseline.  Truncation is conservative twice over -- a GA's later generations
are the expensive ones, since the population is by then mostly feasible, so
the first 174 of 300 cost rather less than 58 % of the total.  The realised
CPU is reported so that can be checked rather than taken on trust.

Both trees must already be on disk::

    results/repair/raw            baseline and repair at 300 generations
    results/repair_equalcpu/raw   both arms at 174 generations

Pairing is by (instance, seed), and the seeds match across the two trees
because both use the same ``run_seed_for`` rule.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
from typing import Dict, List, Sequence

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from run_repair_ab import (  # noqa: E402
    bootstrap_ci, collect_records, mcnemar_exact, sign_flip_test,
    wilcoxon_signed_rank,
)

FULL_DIR = results_dir("repair")
EQUAL_DIR = results_dir("repair_equalcpu")


def index_by_arm(records: Sequence[Dict], arm: str) -> Dict:
    return {(r["instance"], r["run"]): r for r in records if r["arm"] == arm}


def paired(baseline: Dict, repair: Dict, metric: str) -> List[float]:
    """Signed differences on one metric, positive where repair did better."""
    out = []
    for key, base in baseline.items():
        other = repair.get(key)
        if other is None:
            continue
        a, b = base.get(metric), other.get(metric)
        if a is None or b is None or not math.isfinite(a) or not math.isfinite(b):
            continue
        out.append(float(a) - float(b))
    return out


def describe(name: str, deltas: Sequence[float], unit: str, spec: str = ".4f") -> Dict:
    d = np.asarray(deltas, dtype=float)
    low, high = bootstrap_ci(d)
    p = sign_flip_test(d)
    _, wp = wilcoxon_signed_rank(d)
    row = {
        "metric": name, "n_pairs": int(d.size),
        "mean_gain": float(d.mean()), "median_gain": float(np.median(d)),
        "ci_low": low, "ci_high": high,
        "repair_wins": int((d > 0).sum()), "baseline_wins": int((d < 0).sum()),
        "permutation_p": p, "wilcoxon_p": wp,
    }
    verdict = ("repair better" if row["mean_gain"] > 0 else "repair worse")
    if p is not None and p > 0.05:
        verdict = "no detectable difference"
    print(f"  {name:<16} {row['mean_gain']:+{spec[1:]}} {unit:<6} "
          f"[{low:{spec}}, {high:{spec}}]  "
          f"wins {row['repair_wins']:>3}/{row['repair_wins'] + row['baseline_wins']:<3} "
          f"p={p:.4g}  -- {verdict}")
    return row


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Equal-compute control for the placement-repair A/B.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--full-dir", default=FULL_DIR,
                        help="study holding the full-length baseline and repair arms")
    parser.add_argument("--equal-dir", default=EQUAL_DIR,
                        help="study holding the CPU-truncated repair arm")
    parser.add_argument("--out", default=None,
                        help="JSON destination (default: <equal-dir>/equal_cpu_check.json)")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    full_dir, equal_dir = args.full_dir, args.equal_dir
    for path in (full_dir, equal_dir):
        if not os.path.isdir(os.path.join(path, "raw")):
            raise SystemExit(f"missing study: {path}")

    full = collect_records(full_dir)
    equal = collect_records(equal_dir)
    baseline = index_by_arm(full, "baseline")          # the full-length baseline
    repair_full = index_by_arm(full, "repair")         # the full-length repair arm
    repair_eq = index_by_arm(equal, "repair")          # the CPU-truncated repair arm

    full_gens = int(np.median([r["generations"] for r in baseline.values()]))
    eq_gens = int(np.median([r["generations"] for r in repair_eq.values()]))
    retract = next(iter(baseline.values())).get("retract", "unrecorded")

    print(f"baseline @{full_gens} gens : {len(baseline)} runs   ({full_dir})")
    print(f"repair   @{full_gens} gens : {len(repair_full)} runs")
    print(f"repair   @{eq_gens} gens : {len(repair_eq)} runs   ({equal_dir})")
    print(f"objective          : retract = {retract}")

    def mean_of(index: Dict, key: str) -> float:
        return float(np.mean([r[key] for r in index.values()]))

    base_cpu = mean_of(baseline, "cpu_time")
    eq_cpu = mean_of(repair_eq, "cpu_time")
    print(f"\nRealised CPU per run: baseline {base_cpu:.1f} s, "
          f"truncated repair {eq_cpu:.1f} s "
          f"(repair is at {100 * eq_cpu / base_cpu:.1f} % of the baseline's compute)")
    if eq_cpu > base_cpu:
        print("  NOTE: the truncated repair still spent MORE cpu than the "
              "baseline, so the control is not yet conservative -- treat a "
              "win below as suggestive only.")
    print(f"Evaluations per run:  baseline {mean_of(baseline, 'evaluations'):.0f}, "
          f"truncated repair {mean_of(repair_eq, 'evaluations'):.0f}")

    print(f"\nEqual compute -- baseline @{full_gens} gens vs repair @{eq_gens} gens")
    print("  (positive gain = repair better)")
    rows = [
        describe("cycle_time", paired(baseline, repair_eq, "cycle_time"), "s"),
        describe("mean_feasible",
                 [-d for d in paired(baseline, repair_eq, "mean_feasible")], ""),
        describe("cpu_time", paired(baseline, repair_eq, "cpu_time"), "s", ".2f"),
    ]

    print(f"\nFor reference -- equal evaluations, baseline vs repair @{full_gens} gens")
    reference = describe("cycle_time", paired(baseline, repair_full, "cycle_time"), "s")

    both = sum(1 for k, r in repair_eq.items()
               if k in baseline and r["feasible"] and baseline[k]["feasible"])
    only_b = sum(1 for k, r in repair_eq.items()
                 if k in baseline and baseline[k]["feasible"] and not r["feasible"])
    only_r = sum(1 for k, r in repair_eq.items()
                 if k in baseline and r["feasible"] and not baseline[k]["feasible"])
    print(f"\nFeasible at all: both {both}, baseline only {only_b}, "
          f"repair only {only_r}, McNemar p = {mcnemar_exact(only_b, only_r)}")

    out = {
        "full_dir": full_dir,
        "equal_dir": equal_dir,
        "retract": retract,
        "baseline_generations": full_gens,
        "repair_generations": eq_gens,
        "baseline_cpu_mean": base_cpu,
        "repair_cpu_mean": eq_cpu,
        "cpu_ratio": eq_cpu / base_cpu,
        "equal_compute": rows,
        "equal_evaluations_reference": reference,
        "feasible": {"both": both, "baseline_only": only_b, "repair_only": only_r},
    }
    path = args.out or os.path.join(equal_dir, "equal_cpu_check.json")
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(out, handle, indent=2)
    print(f"\nwritten -> {path}")


if __name__ == "__main__":
    main()
