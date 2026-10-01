"""The two weights in the penalty arms: one is inert, the other is not.

    python -m experiments.penalty_weights

``penalty.py`` claims two things about the coefficients in

    F(x) = T_inf + w * CVD(x) + m * N(x)      x infeasible

and both are cheap enough to check rather than assert.

**``w`` cannot affect the search in the CVD-only arm.**  The GA selects by
tournament, elitises by ``argsort`` and tracks its best by ``min``; all three
are rank-based, and the ``T_inf`` layer already puts every feasible layout
below every infeasible one.  Scaling ``CVD`` by a positive constant is then a
monotone transform of a quantity only ever compared, so the two runs should be
*bit-identical*, not merely similar.  That is a strong prediction and a
mis-implemented penalty would fail it immediately.

**``m/w`` can.**  Adding the count term reorders the infeasible set: it decides
whether four shallow overlaps rank above or below one deep one.  So the second
half sweeps ``m`` and reports what the choice is worth, rather than defending
one value.

Small on purpose: this is a check on the mechanism, not a comparison of arms.
"""

from __future__ import annotations

import argparse
import os
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import Dict, List, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()

from ga import ConfigurableGA, GAConfig, make_initial_population  # noqa: E402
from penalty import INFEASIBLE_BASE, PenaltyEvaluator  # noqa: E402
from problems import build_instances  # noqa: E402
from run_repair_ab import run_seed_for  # noqa: E402


@dataclass
class Spec:
    """One run, as a picklable description of itself."""

    index: int
    run: int
    generations: int
    weight: float
    count_weight: float


def one_run(spec: Spec) -> Dict:
    """Run one GA.  Executed in a worker process, so it rebuilds its instance."""
    index, run = spec.index, spec.run
    generations, weight, count_weight = spec.generations, spec.weight, spec.count_weight
    instance = build_instances("test")[index]
    seed = run_seed_for(index, run)
    evaluator = PenaltyEvaluator(
        instance, violation_weight=weight, overlap_count_weight=count_weight
    )
    config = GAConfig(label=f"w={weight},m={count_weight}", generations=generations)
    ga = ConfigurableGA(instance, evaluator=evaluator, config=config, seed=seed)
    population, seeded = make_initial_population(
        instance, config.population_size, seed=seed
    )
    started = time.process_time()
    result = ga.optimize(initial_population=population, seeded_fraction=seeded)
    value = float(result.best_fitness)
    return {
        "instance": instance.name,
        "index": index,
        "run": run,
        "weight": weight,
        "count_weight": count_weight,
        "cycle_time": value if value < INFEASIBLE_BASE else None,
        # Rounded only where it is compared for equality across weights: the
        # prediction is that the runs are identical, so the comparison has to
        # be on the raw genes rather than on a printed summary of them.
        "chromosome": [float(v) for v in np.asarray(result.best_chromosome, dtype=float)],
        "cpu": time.process_time() - started,
    }


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--instances", type=int, default=4)
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument("--generations", type=int, default=60)
    parser.add_argument("--weights", type=float, nargs="+", default=[1.0, 1000.0])
    parser.add_argument("--count-weights", type=float, nargs="+",
                        default=[0.0, 0.01, 0.05, 0.25, 1.0])
    parser.add_argument("--workers", type=int,
                        default=max(1, (os.cpu_count() or 2) - 2))
    args = parser.parse_args()

    instances = build_instances("test")[: args.instances]
    pairs = [(i, r) for i in range(len(instances)) for r in range(args.runs)]

    specs = [Spec(i, r, args.generations, w, 0.0) for i, r in pairs for w in args.weights]
    specs += [Spec(i, r, args.generations, 1.0, m)
              for i, r in pairs for m in args.count_weights]

    print(f"{len(specs)} runs over {len(instances)} instances x {args.runs} seeds, "
          f"{args.generations} generations, {args.workers} workers")
    started = time.perf_counter()
    with ProcessPoolExecutor(max_workers=args.workers) as pool:
        results = list(pool.map(one_run, specs))
    print(f"  done in {(time.perf_counter() - started) / 60:.1f} min\n")

    lookup = {(r["index"], r["run"], r["weight"], r["count_weight"]): r for r in results}

    print(f"w-invariance at m=0: {args.weights}")
    identical = 0
    total = 0
    for index, run in pairs:
        first = lookup[(index, run, args.weights[0], 0.0)]
        for w in args.weights[1:]:
            other = lookup[(index, run, w, 0.0)]
            total += 1
            same = (first["cycle_time"] == other["cycle_time"]
                    and first["chromosome"] == other["chromosome"])
            identical += same
            print(f"  {first['instance']} run {run}: w={args.weights[0]:g} -> "
                  f"{first['cycle_time']}, w={w:g} -> {other['cycle_time']}   "
                  f"{'identical' if same else 'DIFFERENT'}")
    print(f"  {identical}/{total} weight pairs produced bit-identical runs\n")

    print(f"m sweep at w=1: {args.count_weights}")
    print(f"  {'m':>8} {'runs':>6} {'mean cycle time (s)':>21} {'best':>8} "
          f"{'vs m=0':>9}")
    reference = None
    for m in args.count_weights:
        values = [lookup[(i, r, 1.0, m)]["cycle_time"] for i, r in pairs]
        values = [v for v in values if v is not None]
        if not values:
            print(f"  {m:>8g} {0:>6} {'--':>21} {'--':>8} {'--':>9}")
            continue
        mean = float(np.mean(values))
        if reference is None:
            reference = mean
        print(f"  {m:>8g} {len(values):>6} {mean:>21.4f} {min(values):>8.4f} "
              f"{mean - reference:>+9.4f}")


if __name__ == "__main__":
    main()
