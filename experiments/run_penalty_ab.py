"""Four ways to handle the geometric constraints, on identical runs.

    python -m experiments.run_penalty_ab
    python -m experiments.run_penalty_ab --runs 5 --generations 150
    python -m experiments.run_penalty_ab --only aggregate

"Repair beats the ``+inf`` wall" is a much weaker claim than "repair beats
anything else you might reasonably do", and a graded penalty is the first
thing anyone would reach for.  So four arms, all the same GA -- same
population, tournament, operator rates, elitism, seed and rejection-sampled
starting population -- differing only in the map from a layout to a score:

``baseline``       ``F = T(x)`` if feasible else ``+inf``.  A death penalty:
                   infeasible layouts survive carrying no information about
                   how badly they missed, so selection sees a flat plateau.

``repair``         the same ``F``, but an unplaceable child is projected back
                   onto the placeable set after mutation, so the plateau is
                   mostly not landed on at all.

``penalty_cvd``    ``F = T(x)`` if feasible, else ``T_inf + w * CVD(x)``, with
                   ``CVD(x) = sum_i max(0, g_i(x))`` over the 45 geometric
                   constraints in metres.  The plateau gets a slope.

``penalty_count``  as above plus ``m * N(x)``, ``N`` being the number of
                   machines caught in at least one overlap -- a discrete term
                   on the theory that four machines overlapping is worse than
                   two overlapping twice as deeply, metres being equal.

``T_inf`` is above every attainable cycle time, so cycle time stays the
primary objective in all four arms and no feasible layout can lose to an
infeasible one.  See :mod:`penalty` for why that makes ``w`` immaterial in
``penalty_cvd`` and ``m/w`` material in ``penalty_count``.

**Two budgets.** Equal generations is what the GA literature usually reports,
and it flatters whichever arm converts cheap evaluations into expensive ones
-- a layout rejected on placement costs about 0.1 ms against 3.3 ms for one
reaching the kinematics.  So every run records its CPU at the end of every
generation and the equal-compute comparison is a *slice* of that trace: per
(instance, seed) the budget is the smallest total CPU any arm spent, and each
arm is read at the last generation it finished inside it.  No arm is rerun at
a guessed generation count, and the realised CPU at the cut is reported.

**Pairing** is by (instance, seed), and it is the whole point: the four arms
start from an identical population and diverge only where the scoring function
differs, so differences are taken within a pair and the statistics over pairs.
That removes the instance-to-instance spread in cycle time, far the largest
source of variance here.  Significance is paired sign-flip permutation plus
Wilcoxon signed-rank, both imported from ``run_repair_ab`` so the two cannot
drift apart.

Run seed ``= 42 + 100 * instance_index + run``, so a pair starts from the same
population as the corresponding run of ``run_ga``.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import os
import platform
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from evaluator import CycleTimeEvaluator  # noqa: E402
from ga import ConfigurableGA, GAConfig, make_initial_population  # noqa: E402
from penalty import INFEASIBLE_BASE, CountingEvaluator, PenaltyEvaluator  # noqa: E402
from problems import build_instances, describe  # noqa: E402
from run_repair_ab import (  # noqa: E402
    bootstrap_ci, collect_records, run_seed_for, sign_flip_test,
    wilcoxon_signed_rank,
)

DEFAULT_RESULTS_DIR = results_dir("penalty")
STAGES = ("run", "aggregate")

#: ``(arm, repair?, violation weight, overlap-count weight)``.  The weights are
#: ignored by the two arms that do not use a penalty evaluator.
ARM_SPECS: Tuple[Tuple[str, bool, float, float], ...] = (
    ("baseline", False, 0.0, 0.0),
    ("repair", True, 0.0, 0.0),
    ("penalty_cvd", False, 1.0, 0.0),
    ("penalty_count", False, 1.0, 0.05),
)
ARMS: Tuple[str, ...] = tuple(spec[0] for spec in ARM_SPECS)
#: The arm every other arm is compared against.
REFERENCE = "repair"

RUN_FIELDS = (
    "arm", "instance", "instance_index", "run", "run_seed",
    "cycle_time", "feasible", "generations", "evaluations",
    "execution_time", "cpu_time", "seeded_fraction",
    "feasible_eval_share", "mean_feasible", "evals_to_first",
    "mean_diversity",
    "violation_weight", "overlap_count_weight",
)


def _jsonable(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def _true_cycle_time(value: float) -> Optional[float]:
    """A score as a cycle time, or ``None`` where it is not one.

    One rule for all four arms: ``baseline`` and ``repair`` mark infeasible
    with ``+inf``, the penalty arms by landing at or above ``INFEASIBLE_BASE``.
    Below that floor the score *is* the cycle time everywhere, because the
    penalty leaves a feasible layout untouched.
    """
    value = float(value)
    if not math.isfinite(value) or value >= INFEASIBLE_BASE:
        return None
    return value


# ----------------------------------------------------------------------
# one run
# ----------------------------------------------------------------------


@dataclass
class Job:
    """One (arm, instance, seed) run.  Picklable, so a worker needs nothing."""

    arm: str
    split: str
    instance_index: int
    run: int
    config: GAConfig
    violation_weight: float
    overlap_count_weight: float
    sigma_min_threshold: float
    dwell: float
    output_dir: str
    check_path: bool = True
    check_arm: bool = True
    arm_clearance: float = 0.005
    retract: Optional[float] = 0.06


def run_json_path(output_dir: str, arm: str, instance_name: str, run: int) -> str:
    return os.path.join(output_dir, "raw", f"{arm}_{instance_name}_run{run}.json")


def build_evaluator(job: Job, instance):
    """The one thing that differs between arms."""
    common = dict(
        sigma_min_threshold=job.sigma_min_threshold, dwell=job.dwell,
        check_path=job.check_path, check_arm=job.check_arm,
        arm_clearance=job.arm_clearance, retract=job.retract,
    )
    if job.arm.startswith("penalty"):
        return PenaltyEvaluator(
            instance,
            violation_weight=job.violation_weight,
            overlap_count_weight=job.overlap_count_weight,
            **common,
        )
    return CountingEvaluator(instance, **common)


def execute(job: Job) -> Dict:
    """Run one GA and write its JSON.  Executed in a worker process."""
    instance = build_instances(job.split)[job.instance_index]
    seed = run_seed_for(job.instance_index, job.run)

    evaluator = build_evaluator(job, instance)
    ga = ConfigurableGA(instance, evaluator=evaluator, config=job.config, seed=seed)

    # The pairing: every arm is handed the identical starting population, so
    # the only thing that can separate them is the scoring function.
    population, seeded = make_initial_population(
        instance, job.config.population_size, seed=seed
    )
    result = ga.optimize(initial_population=population, seeded_fraction=seeded)

    best_so_far = [_true_cycle_time(v) for v in np.asarray(result.best_so_far, dtype=float)]
    evaluations = np.asarray(result.evaluations, dtype=float)
    cpu_elapsed = np.asarray(result.cpu_elapsed, dtype=float)

    cycle_time = _true_cycle_time(result.best_fitness)
    reached = [k for k, v in enumerate(best_so_far) if v is not None]
    evals_to_first = float(evaluations[reached[0]]) if reached else None

    # Integrity check: the reported answer must be the cycle time of the
    # reported layout, evaluated by the *plain* objective.  A penalty arm that
    # returned a penalised score here would look like a very fast layout.
    if cycle_time is not None:
        plain = CycleTimeEvaluator(
            instance, sigma_min_threshold=job.sigma_min_threshold, dwell=job.dwell,
            check_path=job.check_path, check_arm=job.check_arm,
            arm_clearance=job.arm_clearance, retract=job.retract,
        )
        check = plain.evaluate(plain.decode(result.best_chromosome))
        if not check.feasible or abs(check.cycle_time - cycle_time) > 1e-6:
            raise ValueError(
                f"{job.arm} {instance.name} run {job.run}: reported {cycle_time} "
                f"but the chromosome evaluates to {check.cycle_time} "
                f"(feasible={check.feasible})"
            )

    # ``ga.py``'s feasible_fraction is ``isfinite(fitness)``, which a penalty
    # makes identically 1.0 -- so it is recorded only where it means something,
    # and the cross-arm feasibility measure is the evaluation share below.
    feasible_fraction = np.asarray(result.feasible_fraction, dtype=float)
    mean_feasible = (
        None if job.arm.startswith("penalty") else round(float(feasible_fraction.mean()), 6)
    )

    record = {
        "arm": job.arm,
        "instance": instance.name,
        "instance_index": job.instance_index,
        "run": job.run,
        "run_seed": seed,
        "cycle_time": _jsonable(cycle_time),
        "feasible": cycle_time is not None,
        "generations": int(result.generations),
        "evaluations": int(result.total_evaluations),
        "execution_time": round(float(result.execution_time), 4),
        "cpu_time": round(float(result.cpu_time), 4),
        "seeded_fraction": round(float(result.seeded_fraction), 4),
        "feasible_eval_share": round(
            evaluator.n_feasible_evals / max(evaluator.n_evals, 1), 6
        ),
        "mean_feasible": mean_feasible,
        "evals_to_first": evals_to_first,
        "mean_diversity": round(float(np.nanmean(result.diversity)), 6),
        "violation_weight": job.violation_weight,
        "overlap_count_weight": job.overlap_count_weight,
        "best_so_far": [_jsonable(v) for v in best_so_far],
        "evaluations_trace": [int(v) for v in evaluations],
        "cpu_elapsed_trace": [round(float(v), 4) for v in cpu_elapsed],
        "chromosome": [round(float(v), 6) for v in result.best_chromosome],
        "sigma_min_threshold": job.sigma_min_threshold,
        "dwell": job.dwell,
        "check_path": job.check_path,
        "check_arm": job.check_arm,
        "arm_clearance": job.arm_clearance,
        "retract": job.retract,
    }

    path = run_json_path(job.output_dir, job.arm, instance.name, job.run)
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        json.dump(record, handle, indent=2)
    return record


def _progress(done: int, total: int, started: float, every: int = 10) -> None:
    if done % every and done != total:
        return
    elapsed = time.perf_counter() - started
    rate = done / elapsed if elapsed > 0 else 0.0
    remaining = (total - done) / rate if rate > 0 else float("nan")
    print(
        f"  {done:>4}/{total} runs  {elapsed / 60:5.1f} min elapsed"
        f"  ~{remaining / 60:5.1f} min left",
        flush=True,
    )


def run_grid(jobs: Sequence[Job], workers: int, resume: bool) -> List[Dict]:
    pending: List[Job] = []
    records: List[Dict] = []
    for job in jobs:
        instance_name = f"{job.split}_{job.instance_index + 1}"
        path = run_json_path(job.output_dir, job.arm, instance_name, job.run)
        if resume and os.path.exists(path):
            with open(path, encoding="utf-8") as handle:
                records.append(json.load(handle))
        else:
            pending.append(job)

    if records:
        print(f"  resuming: {len(records)} runs already on disk")
    if not pending:
        return records

    started = time.perf_counter()
    done = 0
    if workers <= 1:
        for job in pending:
            records.append(execute(job))
            done += 1
            _progress(done, len(pending), started)
    else:
        with ProcessPoolExecutor(max_workers=workers) as pool:
            for record in pool.map(execute, pending):
                records.append(record)
                done += 1
                _progress(done, len(pending), started)
    return records


# ----------------------------------------------------------------------
# equal compute, by slicing the traces rather than by rerunning
# ----------------------------------------------------------------------


def truncate_to_cpu(record: Dict, budget: float) -> Dict:
    """What this run had achieved by the time it had spent ``budget`` seconds.

    Returns the best-so-far and the realised CPU at the last generation that
    finished inside the budget.  A run whose very first generation already
    costs more than the budget has achieved nothing, which is reported as
    ``None`` rather than silently as its generation-0 value.
    """
    cpu = np.asarray(record["cpu_elapsed_trace"], dtype=float)
    inside = np.flatnonzero(cpu <= budget)
    if inside.size == 0:
        return {"cycle_time": None, "generations": 0, "cpu_time": 0.0}
    last = int(inside[-1])
    best = record["best_so_far"][last]
    return {
        "cycle_time": best,
        "generations": last + 1,
        "cpu_time": float(cpu[last]),
    }


def equal_cpu_view(by_key: Dict[Tuple[str, int], Dict[str, Dict]]) -> Dict[Tuple[str, int], Dict[str, Dict]]:
    """Re-read every arm of every pair at a common per-pair CPU budget.

    The budget is the smallest total CPU any arm of that pair spent, so it is a
    budget every arm demonstrably had.  Taking it per pair rather than globally
    keeps the comparison within-pair, which is where the power is.
    """
    view: Dict[Tuple[str, int], Dict[str, Dict]] = {}
    for key, arms in by_key.items():
        if len(arms) != len(ARMS):
            continue
        budget = min(float(r["cpu_time"]) for r in arms.values())
        view[key] = {
            arm: dict(truncate_to_cpu(record, budget), budget=budget)
            for arm, record in arms.items()
        }
    return view


# ----------------------------------------------------------------------
# aggregation
# ----------------------------------------------------------------------


def index_records(records: Sequence[Dict]) -> Dict[Tuple[str, int], Dict[str, Dict]]:
    """``(instance, run) -> {arm: record}``, keeping only complete pairs."""
    grouped: Dict[Tuple[str, int], Dict[str, Dict]] = {}
    for record in records:
        grouped.setdefault((record["instance"], record["run"]), {})[record["arm"]] = record
    return {key: arms for key, arms in grouped.items() if len(arms) == len(ARMS)}


def _mean(values: Sequence[Optional[float]]) -> Optional[float]:
    kept = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return float(np.mean(kept)) if kept else None


def compare(
    by_key: Dict[Tuple[str, int], Dict[str, Dict]],
    arm: str,
    reference: str,
    metric: str,
    lower_is_better: bool = True,
) -> Dict:
    """Paired summary of one metric, ``arm`` against ``reference``.

    Signed so that a positive difference always means ``arm`` did better.
    Pairs where either side is missing the metric are dropped and counted, so a
    comparison can never quietly be taken over a different set of pairs than
    the one it claims.
    """
    usable: List[Tuple[float, float]] = []
    dropped = 0
    for arms in by_key.values():
        a, b = arms[reference].get(metric), arms[arm].get(metric)
        if a is None or b is None or not math.isfinite(float(a)) or not math.isfinite(float(b)):
            dropped += 1
            continue
        usable.append((float(a), float(b)))

    summary = {"metric": metric, "arm": arm, "reference": reference,
               "n_pairs": len(usable), "dropped": dropped}
    if not usable:
        return summary

    ref = np.array([a for a, _ in usable])
    other = np.array([b for _, b in usable])
    sign = 1.0 if lower_is_better else -1.0
    differences = sign * (ref - other)

    low, high = bootstrap_ci(differences)
    _, wilcoxon_p = wilcoxon_signed_rank(differences)
    summary.update({
        "reference_mean": float(ref.mean()),
        "arm_mean": float(other.mean()),
        "mean_gain": float(differences.mean()),
        "median_gain": float(np.median(differences)),
        "ci_low": low, "ci_high": high,
        "arm_wins": int((differences > 0).sum()),
        "reference_wins": int((differences < 0).sum()),
        "ties": int((differences == 0).sum()),
        "perm_p": sign_flip_test(differences),
        "wilcoxon_p": wilcoxon_p,
    })
    return summary


def arm_table(by_key: Dict[Tuple[str, int], Dict[str, Dict]], metric: str) -> Dict[str, Optional[float]]:
    return {
        arm: _mean([arms[arm].get(metric) for arms in by_key.values()])
        for arm in ARMS
    }


# ----------------------------------------------------------------------
# reporting
# ----------------------------------------------------------------------


def write_csv(path: str, rows: Sequence[Dict], fields: Sequence[str]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def _cell(value: Optional[float], spec: str = ".4f") -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "--"
    return format(value, spec)


def _p(value: Optional[float]) -> str:
    return "--" if value is None else f"{value:.4f}"


FITNESS_TEXT = """\
## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = {base:.0f} s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For six machines there are 45 of them:

| count | constraint | `g_i(x)` |
| ---: | --- | --- |
| 24 | footprint inside the cell | signed excursion of the footprint past each of the four walls |
| 6 | clear of the robot column | `keep_out_radius - distance(footprint, base)` |
| 15 | machines mutually disjoint | `overlap_depth(i, j)`, the minimum translation separating the pair with the clearance included |

`CVD(x) = 0` exactly when `layout_is_placeable(x)` is true; this is checked
rather than assumed. `N(x)` is the number of machines caught in at least one
overlap, an integer in `0..6`.

```
baseline        F(x) = T(x)   if x feasible
                     = +inf   otherwise

repair          F(x) = T(x)   if x feasible          (an unplaceable child is
                     = +inf   otherwise               projected onto the
                                                      placeable set first)

penalty_cvd     F(x) = T(x)                  if x feasible
                     = T_inf + w * CVD(x)    otherwise           w = {w:g}

penalty_count   F(x) = T(x)                            if x feasible
                     = T_inf + w * CVD(x) + m * N(x)   otherwise
                                                       w = {w:g}, m = {m:g}
```

**Why `w` is not a free parameter to be argued over in `penalty_cvd`.** The GA
selects by tournament, elitises by `argsort`, and tracks its best by `min` --
every one of those is rank-based. Because the `T_inf` layer already separates
feasible from infeasible, multiplying `CVD` by any positive `w` leaves all
three unchanged, so `w` cannot affect the search at all. The usual objection to
penalty methods -- that the exchange rate between metres of overlap and seconds
of cycle time is an unstated preference -- does not apply to this formulation,
because the layers never let the two quantities trade.

The ratio `m/w` in `penalty_count` **does** matter: it decides whether four
shallow overlaps rank above or below one deep one. `m = {m:g}` makes one
overlapping machine worth {m:g} m of violation depth, against a median
violation of about 0.13 m over random infeasible layouts -- so the two terms
are of comparable size rather than one dominating.
"""


def write_report(
    path: str,
    config: GAConfig,
    by_key,
    equal_cpu,
    summaries: Dict[str, Dict[str, Dict]],
    equal_cpu_summaries: Dict[str, Dict],
    per_instance_rows: Sequence[Dict],
) -> None:
    n_pairs = len(by_key)
    instances = sorted({key[0] for key in by_key})
    runs = sorted({key[1] for key in by_key})

    lines: List[str] = []
    lines.append("# Repair, the wall, and two penalties")
    lines.append("")
    lines.append(
        f"{n_pairs} matched sets ({len(instances)} instances x {len(runs)} seeds), "
        f"population {config.population_size}, {config.generations} generations, "
        f"{config.evaluation_budget()} evaluations per run."
    )
    lines.append("")
    lines.append(
        "Each set is one (instance, seed). All four arms are handed the identical "
        "rejection-sampled initial population, the same seed and the same operator "
        f"rates (tournament k={config.tournament_k}, uniform crossover "
        f"p={config.crossover_rate}, mutation p={config.mutation_rate} per machine, "
        f"elitism {config.elitism_rate:.0%}, sigma={config.mutation_sigma}), and differ "
        "only in the scoring function. Differences are taken within a set and signed so "
        f"**positive means the arm beat `{REFERENCE}`**."
    )
    lines.append("")
    lines.append(FITNESS_TEXT.format(
        base=INFEASIBLE_BASE,
        w=dict((s[0], s[2]) for s in ARM_SPECS)["penalty_cvd"],
        m=dict((s[0], s[3]) for s in ARM_SPECS)["penalty_count"],
    ))

    # -- headline ------------------------------------------------------
    lines.append("## Cycle time, at equal generations")
    lines.append("")
    lines.append("| arm | runs feasible | mean cycle time (s) | median (s) | best (s) | mean CPU (s) | feasible evaluations |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: |")
    for arm in ARMS:
        values = [arms[arm]["cycle_time"] for arms in by_key.values()]
        kept = [v for v in values if v is not None]
        lines.append(
            f"| `{arm}` | {len(kept)}/{len(values)} | {_cell(_mean(kept))} | "
            f"{_cell(float(np.median(kept)) if kept else None)} | "
            f"{_cell(min(kept) if kept else None)} | "
            f"{_cell(arm_table(by_key, 'cpu_time')[arm], '.1f')} | "
            f"{_cell(arm_table(by_key, 'feasible_eval_share')[arm], '.4f')} |"
        )
    lines.append("")
    lines.append(
        "`feasible evaluations` is the share of all fitness calls that landed on a "
        "genuinely feasible layout -- the one feasibility measure defined identically "
        "in all four arms, since the population-share trace in `ga.py` is "
        "`isfinite(fitness)` and a penalty makes that identically 1.0."
    )
    lines.append("")

    # -- paired, equal generations -------------------------------------
    lines.append(f"## Paired against `{REFERENCE}`, equal generations")
    lines.append("")
    for metric, label, lower in (
        ("cycle_time", "cycle time (s)", True),
        ("feasible_eval_share", "feasible evaluation share", False),
        ("cpu_time", "CPU time (s)", True),
    ):
        lines.append(f"**{label}**")
        lines.append("")
        lines.append("| arm | pairs | arm | " + REFERENCE + " | mean gain | 95% CI | arm wins | "
                     + REFERENCE + " wins | perm p | Wilcoxon p |")
        lines.append("| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |")
        for arm in ARMS:
            if arm == REFERENCE:
                continue
            s = summaries[metric][arm]
            if not s.get("n_pairs"):
                lines.append(f"| `{arm}` | 0 | -- | -- | -- | -- | -- | -- | -- | -- |")
                continue
            ci = f"[{_cell(s['ci_low'])}, {_cell(s['ci_high'])}]"
            lines.append(
                f"| `{arm}` | {s['n_pairs']} | {_cell(s['arm_mean'])} | {_cell(s['reference_mean'])} | "
                f"{_cell(s['mean_gain'])} | {ci} | {s['arm_wins']} | {s['reference_wins']} | "
                f"{_p(s['perm_p'])} | {_p(s['wilcoxon_p'])} |"
            )
        lines.append("")

    # -- against the wall ----------------------------------------------
    lines.append("## Paired against `baseline`: does a graded penalty beat the wall?")
    lines.append("")
    lines.append(
        "The comparison above asks whether anything beats repair. This one asks the "
        "question the penalty arms were actually built to answer -- whether putting a "
        "slope on the infeasible plateau beats leaving it flat. Signed so positive "
        "means the arm beat `baseline`."
    )
    lines.append("")
    for label, view in (("equal generations", by_key), ("equal CPU", equal_cpu)):
        lines.append(f"**{label}**")
        lines.append("")
        lines.append("| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins "
                     "| baseline wins | perm p |")
        lines.append("| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |")
        for arm in ARMS:
            if arm == "baseline":
                continue
            s = compare(view, arm, "baseline", "cycle_time", True)
            if not s.get("n_pairs"):
                continue
            ci = f"[{_cell(s['ci_low'])}, {_cell(s['ci_high'])}]"
            lines.append(
                f"| `{arm}` | {s['n_pairs']} | {_cell(s['arm_mean'])} | "
                f"{_cell(s['reference_mean'])} | {_cell(s['mean_gain'])} | {ci} | "
                f"{s['arm_wins']} | {s['reference_wins']} | {_p(s['perm_p'])} |"
            )
        lines.append("")

    # -- equal cpu -----------------------------------------------------
    lines.append(f"## Paired against `{REFERENCE}`, equal CPU")
    lines.append("")
    lines.append(
        "For each (instance, seed) the budget is the smallest total CPU any of the four "
        "arms spent on that set -- a budget every arm demonstrably had -- and each arm is "
        "read at the last generation it finished inside it. No arm is rerun."
    )
    lines.append("")
    lines.append("| arm | generations at budget | realised CPU (s) | mean cycle time (s) |")
    lines.append("| --- | ---: | ---: | ---: |")
    for arm in ARMS:
        gens = _mean([v[arm]["generations"] for v in equal_cpu.values()])
        cpu = _mean([v[arm]["cpu_time"] for v in equal_cpu.values()])
        cyc = _mean([v[arm]["cycle_time"] for v in equal_cpu.values()])
        lines.append(f"| `{arm}` | {_cell(gens, '.1f')} | {_cell(cpu, '.1f')} | {_cell(cyc)} |")
    lines.append("")
    lines.append("| arm | pairs | arm | " + REFERENCE + " | mean gain | 95% CI | arm wins | "
                 + REFERENCE + " wins | perm p | Wilcoxon p |")
    lines.append("| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |")
    for arm in ARMS:
        if arm == REFERENCE:
            continue
        s = equal_cpu_summaries[arm]
        if not s.get("n_pairs"):
            lines.append(f"| `{arm}` | 0 | -- | -- | -- | -- | -- | -- | -- | -- |")
            continue
        ci = f"[{_cell(s['ci_low'])}, {_cell(s['ci_high'])}]"
        lines.append(
            f"| `{arm}` | {s['n_pairs']} | {_cell(s['arm_mean'])} | {_cell(s['reference_mean'])} | "
            f"{_cell(s['mean_gain'])} | {ci} | {s['arm_wins']} | {s['reference_wins']} | "
            f"{_p(s['perm_p'])} | {_p(s['wilcoxon_p'])} |"
        )
    lines.append("")

    # -- per instance --------------------------------------------------
    lines.append("## Per instance, mean cycle time (s), equal generations")
    lines.append("")
    lines.append("| instance | sets | " + " | ".join(f"`{a}`" for a in ARMS) + " |")
    lines.append("| --- | ---: | " + " | ".join("---:" for _ in ARMS) + " |")
    for row in per_instance_rows:
        cells = " | ".join(_cell(row.get(a)) for a in ARMS)
        lines.append(f"| {row['instance']} | {row['sets']} | {cells} |")
    lines.append("")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


def per_instance(by_key) -> List[Dict]:
    rows: Dict[str, Dict] = {}
    for (instance, _run), arms in by_key.items():
        row = rows.setdefault(instance, {"instance": instance, "sets": 0,
                                         **{a: [] for a in ARMS}})
        row["sets"] += 1
        for arm in ARMS:
            value = arms[arm]["cycle_time"]
            if value is not None:
                row[arm].append(float(value))
    out = []
    for instance in sorted(rows, key=lambda n: int(n.rsplit("_", 1)[1])):
        row = rows[instance]
        out.append({"instance": instance, "sets": row["sets"],
                    **{a: (float(np.mean(row[a])) if row[a] else None) for a in ARMS}})
    return out


# ----------------------------------------------------------------------
# stages
# ----------------------------------------------------------------------


def stage_run(args) -> None:
    instances = build_instances(args.split)
    if args.instances:
        instances = instances[: args.instances]
    config_kwargs = dict(
        population_size=args.population, generations=args.generations,
        crossover_rate=args.crossover_rate, mutation_rate=args.mutation_rate,
        elitism_rate=args.elitism_rate, tournament_k=args.tournament_k,
        mutation_sigma=args.mutation_sigma, record_diversity=True,
    )

    jobs: List[Job] = []
    for arm, repair, weight, count_weight in ARM_SPECS:
        config = GAConfig(label=arm, repair=repair, **config_kwargs)
        for index in range(len(instances)):
            for run in range(args.runs):
                jobs.append(Job(
                    arm=arm, split=args.split, instance_index=index, run=run,
                    config=config, violation_weight=weight,
                    overlap_count_weight=count_weight,
                    sigma_min_threshold=args.sigma_min, dwell=args.dwell,
                    output_dir=args.output, retract=args.retract,
                ))

    print(f"{len(jobs)} runs: {len(ARMS)} arms x {len(instances)} instances "
          f"x {args.runs} seeds, {args.generations} generations, "
          f"{args.workers} workers")
    started = time.perf_counter()
    run_grid(jobs, args.workers, args.resume)
    print(f"  done in {(time.perf_counter() - started) / 60:.1f} min")

    metadata = {
        "split": args.split,
        "instances": [describe(i) | {"name": i.name} for i in instances],
        "arms": [
            {"arm": a, "repair": r, "violation_weight": w, "overlap_count_weight": m}
            for a, r, w, m in ARM_SPECS
        ],
        "infeasible_base": INFEASIBLE_BASE,
        "config": GAConfig(**config_kwargs).to_dict(),
        "runs": args.runs,
        "retract": args.retract,
        "sigma_min_threshold": args.sigma_min,
        "dwell": args.dwell,
        "platform": platform.platform(),
        "python": platform.python_version(),
        "numpy": np.__version__,
    }
    os.makedirs(args.output, exist_ok=True)
    with open(os.path.join(args.output, "metadata.json"), "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)


def stage_aggregate(args) -> None:
    records = collect_records(args.output)
    if not records:
        print("no runs on disk; run the 'run' stage first")
        return
    by_key = index_records(records)
    if not by_key:
        print(f"no (instance, seed) set has all {len(ARMS)} arms on disk")
        return
    print(f"{len(by_key)} complete sets over {len(ARMS)} arms")

    summaries = {
        metric: {
            arm: compare(by_key, arm, REFERENCE, metric, lower)
            for arm in ARMS if arm != REFERENCE
        }
        for metric, lower in (
            ("cycle_time", True), ("feasible_eval_share", False), ("cpu_time", True)
        )
    }

    equal_cpu = equal_cpu_view(by_key)
    equal_cpu_summaries = {
        arm: compare(equal_cpu, arm, REFERENCE, "cycle_time", True)
        for arm in ARMS if arm != REFERENCE
    }
    # Against the wall as well as against repair: "does a penalty beat +inf" is
    # the question the two new arms exist to answer, and it is not the same
    # question as "does anything beat repair".
    versus_baseline = {
        budget: {
            arm: compare(view, arm, "baseline", "cycle_time", True)
            for arm in ARMS if arm != "baseline"
        }
        for budget, view in (("equal_generations", by_key), ("equal_cpu", equal_cpu))
    }

    config = GAConfig(**json.load(
        open(os.path.join(args.output, "metadata.json"), encoding="utf-8")
    )["config"] | {"label": "GA"})

    write_csv(os.path.join(args.output, "runs.csv"), records, RUN_FIELDS)
    rows = per_instance(by_key)
    write_csv(os.path.join(args.output, "per_instance.csv"), rows,
              ["instance", "sets", *ARMS])
    with open(os.path.join(args.output, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump({"equal_generations": summaries,
                   "equal_cpu": equal_cpu_summaries,
                   "versus_baseline": versus_baseline}, handle, indent=2)
    write_report(os.path.join(args.output, "summary.md"), config, by_key,
                 equal_cpu, summaries, equal_cpu_summaries, rows)
    print(f"wrote {os.path.join(args.output, 'summary.md')}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--only", choices=STAGES, action="append", dest="stages")
    parser.add_argument("--split", default="test")
    parser.add_argument("--instances", type=int, default=0,
                        help="use only the first N instances of the split (0 = all)")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--generations", type=int, default=150)
    parser.add_argument("--population", type=int, default=200)
    parser.add_argument("--crossover-rate", type=float, default=0.9)
    parser.add_argument("--mutation-rate", type=float, default=0.1)
    parser.add_argument("--elitism-rate", type=float, default=0.1)
    parser.add_argument("--tournament-k", type=int, default=3)
    parser.add_argument("--mutation-sigma", type=float, default=0.10)
    parser.add_argument("--sigma-min", type=float, default=0.14)
    parser.add_argument("--dwell", type=float, default=0.30)
    parser.add_argument("--retract", type=float, default=0.06)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    parser.add_argument("--output", default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--no-resume", dest="resume", action="store_false")
    parser.set_defaults(resume=True)
    return parser


def main() -> None:
    args = build_parser().parse_args()
    stages = args.stages or list(STAGES)
    if "run" in stages:
        stage_run(args)
    if "aggregate" in stages:
        stage_aggregate(args)


if __name__ == "__main__":
    main()
