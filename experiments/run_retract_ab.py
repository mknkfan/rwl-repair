"""Does the retract raise the population's feasible share?

    python -m experiments.run_retract_ab --workers 22
    python -m experiments.run_retract_ab --runs 2 --generations 60
    python -m experiments.run_retract_ab --only aggregate

The retract brackets every station with a lift pose, so the arm traverses above
everything it could hit.  It is justified by the collision argument -- direct
joint-space moves drag the forearm through the machines in between -- but that
says nothing about whether it makes the *search* easier.  This measures the
one thing that would: ``feasible_fraction``, the share of the population with
finite fitness, traced generation by generation.

**No repair in any arm**, so the only operator acting on feasibility is the
motion model.

The two dimensions
------------------
The retract cannot be varied on its own, because whether the clearance test
runs over the whole cycle or only at the stations decides whether a direct-move
layout can be feasible at all.  So both are varied:

``retract_stations``  retract on,  clearance at the stations only
``direct_stations``   retract off, clearance at the stations only
``retract_path``      retract on,  clearance over the whole cycle
``direct_path``       retract off, clearance over the whole cycle

The *stations* pair is the controlled comparison: both arms are scored by an
identical, satisfiable feasibility test, and differ only in the motion model.
The *path* pair is the configuration the main study actually runs in, where
the direct arm is expected to fail outright -- reported so that failure is
measured rather than asserted.

Every arm is otherwise the same GA at the same seed, handed the identical
rejection-sampled starting population, so a pair diverges only where the
evaluator differs.  Run seed ``= 42 + 100 * instance_index + run``.
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

# Run as a script or as a module: either way the source directories must be
# importable, because the modules import each other by bare name.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from evaluator import CycleTimeEvaluator  # noqa: E402
from ga import ConfigurableGA, GAConfig, make_initial_population  # noqa: E402
from problems import build_instances, describe  # noqa: E402
from run_repair_ab import (  # noqa: E402
    bootstrap_ci, collect_records, run_seed_for, sign_flip_test,
    wilcoxon_signed_rank,
)

DEFAULT_RESULTS_DIR = results_dir("retract_ab")
STAGES = ("run", "aggregate")

#: ``(arm, retract, check_path)``.  ``check_arm`` is on everywhere, so the
#: clearance criterion always applies at the stations and the arms differ only
#: in the motion model and in whether it extends along the moves.
ARM_SPECS: Tuple[Tuple[str, Optional[float], bool], ...] = (
    ("retract_stations", 0.06, False),
    ("direct_stations", None, False),
    ("retract_path", 0.06, True),
    ("direct_path", None, True),
)
ARMS: Tuple[str, ...] = tuple(spec[0] for spec in ARM_SPECS)

#: The two controlled contrasts, ``(arm, reference)``.
CONTRASTS: Tuple[Tuple[str, str], ...] = (
    ("retract_stations", "direct_stations"),
    ("retract_path", "direct_path"),
)

RUN_FIELDS = (
    "arm", "retract", "check_path", "instance", "instance_index", "run",
    "run_seed", "cycle_time", "feasible", "generations", "evaluations",
    "cpu_time", "seeded_fraction", "mean_feasible", "final_feasible",
    "first_feasible_gen", "evals_to_first",
)


def _jsonable(value: Optional[float]) -> Optional[float]:
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


@dataclass
class Job:
    """One (arm, instance, seed) run.  Picklable, so a worker needs nothing."""

    arm: str
    retract: Optional[float]
    check_path: bool
    split: str
    instance_index: int
    run: int
    config: GAConfig
    sigma_min_threshold: float
    dwell: float
    output_dir: str


def run_json_path(output_dir: str, arm: str, instance_name: str, run: int) -> str:
    return os.path.join(output_dir, "raw", f"{arm}_{instance_name}_run{run}.json")


def execute(job: Job) -> Dict:
    """Run one GA and write its JSON.  Executed in a worker process."""
    instance = build_instances(job.split)[job.instance_index]
    seed = run_seed_for(job.instance_index, job.run)

    evaluator = CycleTimeEvaluator(
        instance,
        sigma_min_threshold=job.sigma_min_threshold,
        dwell=job.dwell,
        check_path=job.check_path,
        check_arm=True,
        retract=job.retract,
    )
    ga = ConfigurableGA(instance, evaluator=evaluator, config=job.config, seed=seed)

    # The pairing: every arm starts from the identical population, so only the
    # evaluator can separate them.
    population, seeded = make_initial_population(
        instance, job.config.population_size, seed=seed
    )
    result = ga.optimize(initial_population=population, seeded_fraction=seeded)

    feasible_fraction = np.asarray(result.feasible_fraction, dtype=float)
    best_so_far = np.asarray(result.best_so_far, dtype=float)
    evaluations = np.asarray(result.evaluations, dtype=float)

    reached = np.flatnonzero(np.isfinite(best_so_far))
    first_gen = int(reached[0]) + 1 if reached.size else None
    evals_to_first = float(evaluations[reached[0]]) if reached.size else None

    record = {
        "arm": job.arm,
        "retract": job.retract,
        "check_path": job.check_path,
        "instance": instance.name,
        "instance_index": job.instance_index,
        "run": job.run,
        "run_seed": seed,
        "cycle_time": _jsonable(result.best_fitness),
        "feasible": bool(math.isfinite(result.best_fitness)),
        "generations": int(result.generations),
        "evaluations": int(result.total_evaluations),
        "cpu_time": round(float(result.cpu_time), 4),
        "seeded_fraction": round(float(result.seeded_fraction), 4),
        "mean_feasible": round(float(feasible_fraction.mean()), 6),
        "final_feasible": round(float(feasible_fraction[-1]), 6),
        "first_feasible_gen": first_gen,
        "evals_to_first": evals_to_first,
        "feasible_fraction_trace": [round(float(v), 6) for v in feasible_fraction],
        "best_so_far": [_jsonable(v) for v in best_so_far],
        "chromosome": [round(float(v), 6) for v in result.best_chromosome],
        "sigma_min_threshold": job.sigma_min_threshold,
        "dwell": job.dwell,
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
    print(f"  {done:>4}/{total} runs  {elapsed / 60:5.1f} min elapsed"
          f"  ~{remaining / 60:5.1f} min left", flush=True)


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
# aggregation
# ----------------------------------------------------------------------


def index_records(records: Sequence[Dict]) -> Dict[Tuple[str, int], Dict[str, Dict]]:
    """``(instance, run) -> {arm: record}``, keeping only complete sets."""
    grouped: Dict[Tuple[str, int], Dict[str, Dict]] = {}
    for record in records:
        grouped.setdefault((record["instance"], record["run"]), {})[record["arm"]] = record
    return {key: arms for key, arms in grouped.items() if len(arms) == len(ARMS)}


def mean_curve(by_key, arm: str) -> np.ndarray:
    """Mean ``feasible_fraction`` per generation, over every set."""
    traces = [np.asarray(arms[arm]["feasible_fraction_trace"], dtype=float)
              for arms in by_key.values()]
    length = min(t.size for t in traces)
    return np.stack([t[:length] for t in traces]).mean(axis=0)


def compare(by_key, arm: str, reference: str, metric: str) -> Dict:
    """Paired summary of ``metric``, signed so positive means ``arm`` is higher."""
    usable = []
    for arms in by_key.values():
        a, b = arms[reference].get(metric), arms[arm].get(metric)
        if a is None or b is None:
            continue
        if not math.isfinite(float(a)) or not math.isfinite(float(b)):
            continue
        usable.append((float(a), float(b)))

    summary = {"metric": metric, "arm": arm, "reference": reference,
               "n_pairs": len(usable)}
    if not usable:
        return summary

    ref = np.array([a for a, _ in usable])
    other = np.array([b for _, b in usable])
    differences = other - ref
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
        "perm_p": sign_flip_test(differences),
        "wilcoxon_p": wilcoxon_p,
    })
    return summary


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


def write_report(path: str, config: GAConfig, by_key, summaries, curves) -> None:
    n_pairs = len(by_key)
    instances = sorted({key[0] for key in by_key})
    runs = sorted({key[1] for key in by_key})
    length = min(c.size for c in curves.values())

    lines: List[str] = []
    lines.append("# Does the retract raise the feasible share of the population?")
    lines.append("")
    lines.append(
        f"{n_pairs} matched sets ({len(instances)} instances x {len(runs)} seeds), "
        f"population {config.population_size}, {config.generations} generations, "
        f"{config.evaluation_budget()} evaluations per run. **No repair in any arm.**"
    )
    lines.append("")
    lines.append(
        "Each set is one (instance, seed). All four arms start from the identical "
        "rejection-sampled population at the same seed and share every operator "
        "setting, so an arm differs only in its evaluator: whether the tour lifts "
        "to a retract height, and whether the clearance criterion is applied along "
        "the moves or only at the stations."
    )
    lines.append("")
    lines.append("| arm | retract | clearance | mean feasible share | final | runs feasible | mean CPU (s) |")
    lines.append("| --- | --- | --- | ---: | ---: | ---: | ---: |")
    for arm, retract, check_path in ARM_SPECS:
        vals = [arms[arm] for arms in by_key.values()]
        mean_share = float(np.mean([v["mean_feasible"] for v in vals]))
        final = float(np.mean([v["final_feasible"] for v in vals]))
        feas = sum(1 for v in vals if v["feasible"])
        cpu = float(np.mean([v["cpu_time"] for v in vals]))
        lines.append(
            f"| `{arm}` | {'on' if retract else 'off'} | "
            f"{'whole cycle' if check_path else 'stations only'} | "
            f"{mean_share:.4f} | {final:.4f} | {feas}/{len(vals)} | {cpu:.1f} |"
        )
    lines.append("")

    lines.append("## The controlled contrasts")
    lines.append("")
    lines.append(
        "Signed so **positive means the retract arm held the higher feasible share**."
    )
    lines.append("")
    for metric, label in (("mean_feasible", "mean feasible share over the run"),
                          ("final_feasible", "feasible share at the last generation")):
        lines.append(f"**{label}**")
        lines.append("")
        lines.append("| contrast | pairs | retract | direct | mean gain | 95% CI | "
                     "retract wins | direct wins | perm p | Wilcoxon p |")
        lines.append("| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |")
        for arm, reference in CONTRASTS:
            s = summaries[metric][arm]
            if not s.get("n_pairs"):
                continue
            ci = f"[{_cell(s['ci_low'])}, {_cell(s['ci_high'])}]"
            lines.append(
                f"| `{arm}` vs `{reference}` | {s['n_pairs']} | {_cell(s['arm_mean'])} | "
                f"{_cell(s['reference_mean'])} | {_cell(s['mean_gain'])} | {ci} | "
                f"{s['arm_wins']} | {s['reference_wins']} | "
                f"{_cell(s['perm_p'])} | {_cell(s['wilcoxon_p'])} |"
            )
        lines.append("")

    lines.append("## The curve, generation by generation")
    lines.append("")
    lines.append("Mean over every set of the population's feasible share.")
    lines.append("")
    marks = [g for g in (1, 2, 5, 10, 25, 50, 100, 150, 200, 250, 300) if g <= length]
    lines.append("| generation | " + " | ".join(f"`{a}`" for a in ARMS) + " |")
    lines.append("| ---: | " + " | ".join("---:" for _ in ARMS) + " |")
    for g in marks:
        cells = " | ".join(f"{curves[a][g - 1]:.4f}" for a in ARMS)
        lines.append(f"| {g} | {cells} |")
    lines.append("")

    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines) + "\n")


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
        mutation_sigma=args.mutation_sigma, record_diversity=False,
    )

    jobs: List[Job] = []
    for arm, retract, check_path in ARM_SPECS:
        config = GAConfig(label=arm, repair=False, **config_kwargs)
        for index in range(len(instances)):
            for run in range(args.runs):
                jobs.append(Job(
                    arm=arm, retract=retract, check_path=check_path,
                    split=args.split, instance_index=index, run=run,
                    config=config, sigma_min_threshold=args.sigma_min,
                    dwell=args.dwell, output_dir=args.output,
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
        "arms": [{"arm": a, "retract": r, "check_path": p} for a, r, p in ARM_SPECS],
        "repair": False,
        "config": GAConfig(**config_kwargs).to_dict(),
        "runs": args.runs,
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
        metric: {arm: compare(by_key, arm, reference, metric)
                 for arm, reference in CONTRASTS}
        for metric in ("mean_feasible", "final_feasible")
    }
    curves = {arm: mean_curve(by_key, arm) for arm in ARMS}

    config = GAConfig(**json.load(
        open(os.path.join(args.output, "metadata.json"), encoding="utf-8")
    )["config"] | {"label": "GA"})

    write_csv(os.path.join(args.output, "runs.csv"), records, RUN_FIELDS)
    with open(os.path.join(args.output, "curves.csv"), "w", newline="",
              encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["generation", *ARMS])
        length = min(c.size for c in curves.values())
        for g in range(length):
            writer.writerow([g + 1, *(f"{curves[a][g]:.6f}" for a in ARMS)])
    with open(os.path.join(args.output, "summary.json"), "w", encoding="utf-8") as handle:
        json.dump(summaries, handle, indent=2)
    write_report(os.path.join(args.output, "summary.md"), config, by_key,
                 summaries, curves)
    print(f"wrote {os.path.join(args.output, 'summary.md')}")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--only", choices=STAGES, action="append", dest="stages")
    parser.add_argument("--split", default="test")
    parser.add_argument("--instances", type=int, default=0,
                        help="use only the first N instances of the split (0 = all)")
    parser.add_argument("--runs", type=int, default=5)
    parser.add_argument("--generations", type=int, default=300)
    parser.add_argument("--population", type=int, default=200)
    parser.add_argument("--crossover-rate", type=float, default=0.8)
    parser.add_argument("--mutation-rate", type=float, default=0.2)
    parser.add_argument("--elitism-rate", type=float, default=0.1)
    parser.add_argument("--tournament-k", type=int, default=3)
    parser.add_argument("--mutation-sigma", type=float, default=0.10)
    parser.add_argument("--sigma-min", type=float, default=0.14)
    parser.add_argument("--dwell", type=float, default=0.30)
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
