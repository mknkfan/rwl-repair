"""Does repair still beat the wall when the feasibility thresholds move?

    python -m experiments.run_sensitivity --dry-run        # plan, cost, checks
    python -m experiments.run_sensitivity                  # run + aggregate
    python -m experiments.run_sensitivity --only aggregate
    python -m experiments.run_sensitivity --only pathcheck

Every threshold in the feasibility test is one fixed value, chosen by
judgement.  This changes each in turn, holding the others at their defaults,
and asks one question of every setting: is the paired repair-vs-wall gain
still positive, at equal generations and at equal CPU?

``sigma_min``      conditioning floor at every port [m/rad]  0, 0.07, *0.14*, 0.21
``arm_clearance``  arm-to-machine gap along the cycle [m]     0, *0.005*, 0.010, 0.020
``retract``        lift height above the tallest machine [m]  0.03, *0.06*, 0.10

The default cell is shared by the three sweeps, so 9 cells are run, not 11.
Only the ``baseline`` (+inf wall) and ``repair`` arms are run: the two graded
penalties tie with the wall at every machine count, so they add CPU and no
information here.

Changing a threshold changes the problem, so cycle times are **not**
comparable across cells -- a 0 mm clearance cell is simply easier.  The gain
within a cell is the quantity that is compared.

The GA, the pairing and the per-run JSON are ``run_penalty_ab``'s, unchanged:
run seed ``= 42 + 100 * instance + run``, both arms of a pair start from the
identical rejection-sampled population, and each run is checked against the
plain evaluator before it is written.  The equal-CPU reading slices the
per-generation CPU trace at the smaller of the pair's two totals.

``pathcheck`` is a convergence check on the clearance discretisation rather
than a sweep: it re-evaluates every best layout -- this sweep's and the main
study's ``popgen/p200`` -- with 100 samples per move instead of 12, and
reports any that a denser check rejects, and the smallest whole-cycle gap.

Everything is written under ``--output``::

    <cell>/raw/<arm>_<instance>_run<k>.json   per run, as run_penalty_ab
    summary.csv / summary.md                  one row per cell
    pathcheck.csv / pathcheck.md              the dense re-check
    metadata.json                             grid, GA settings, environment
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
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from evaluator import CycleTimeEvaluator  # noqa: E402
from ga import GAConfig  # noqa: E402
from holm import holm  # noqa: E402
from problems import build_instances, describe  # noqa: E402
from run_penalty_ab import Job, run_grid, truncate_to_cpu  # noqa: E402
from run_repair_ab import (  # noqa: E402
    bootstrap_ci, collect_records, sign_flip_test, wilcoxon_signed_rank,
)

DEFAULT_RESULTS_DIR = results_dir("sensitivity")
STAGES = ("run", "aggregate", "pathcheck")
ARMS = ("baseline", "repair")

#: The paper's settings.  Every cell differs from this in one parameter.
DEFAULTS: Dict[str, float] = {"sigma_min": 0.14, "arm_clearance": 0.005, "retract": 0.06}

#: Parameter -> values swept, the default among them.
SWEEPS: Dict[str, Tuple[float, ...]] = {
    "sigma_min": (0.0, 0.07, 0.14, 0.21),
    "arm_clearance": (0.0, 0.005, 0.010, 0.020),
    "retract": (0.03, 0.06, 0.10),
}

UNITS = {"sigma_min": "m/rad", "arm_clearance": "m", "retract": "m"}

#: Measured CPU seconds per run at P=200, G=300 with the default settings,
#: mean over 100 runs of ``results/popgen/p200`` read at generation 300.  For
#: the estimate only; settings that admit more layouts cost more.
CPU_PER_RUN = {"baseline": 23.8, "repair": 155.7}

#: Realised parallel efficiency of the popgen P=200 sweep on 22 workers
#: (46 020 CPU-s in 36.1 min).
PARALLEL_EFFICIENCY = 0.95

#: Samples per move for the dense clearance re-check.
DENSE_PATH_SAMPLES = 100


# ----------------------------------------------------------------------
# the grid
# ----------------------------------------------------------------------


def cell_name(settings: Dict[str, float]) -> str:
    changed = [k for k in SWEEPS if not math.isclose(settings[k], DEFAULTS[k])]
    if not changed:
        return "default"
    (key,) = changed
    return f"{key}={settings[key]:g}"


def build_cells() -> List[Dict[str, float]]:
    """One settings dict per unique cell, the default first."""
    cells = [dict(DEFAULTS)]
    for key, values in SWEEPS.items():
        for value in values:
            if math.isclose(value, DEFAULTS[key]):
                continue
            cells.append(dict(DEFAULTS, **{key: value}))
    return cells


def evaluator_for(instance, settings: Dict[str, float], **extra) -> CycleTimeEvaluator:
    return CycleTimeEvaluator(
        instance, sigma_min_threshold=settings["sigma_min"],
        arm_clearance=settings["arm_clearance"], retract=settings["retract"],
        **extra,
    )


def build_jobs(args, cells: Sequence[Dict[str, float]]) -> List[Job]:
    """Every (cell, arm, instance, seed), repair runs first.

    Repair runs cost about 6.5x a wall run, so starting them first keeps one
    long run from being the last thing the pool waits on.
    """
    config_kwargs = dict(
        population_size=args.population, generations=args.generations,
        crossover_rate=args.crossover_rate, mutation_rate=args.mutation_rate,
        elitism_rate=args.elitism_rate, tournament_k=args.tournament_k,
        mutation_sigma=args.mutation_sigma, record_diversity=True,
    )
    n_instances = args.instances or len(build_instances(args.split))
    jobs: List[Job] = []
    for arm in ("repair", "baseline"):
        config = GAConfig(label=arm, repair=arm == "repair", **config_kwargs)
        for settings in cells:
            for index in range(n_instances):
                for run in range(args.runs):
                    jobs.append(Job(
                        arm=arm, split=args.split, instance_index=index, run=run,
                        config=config, violation_weight=0.0, overlap_count_weight=0.0,
                        sigma_min_threshold=settings["sigma_min"], dwell=args.dwell,
                        output_dir=os.path.join(args.output, cell_name(settings)),
                        arm_clearance=settings["arm_clearance"],
                        retract=settings["retract"],
                    ))
    return jobs


def selected_cells(args) -> List[Dict[str, float]]:
    cells = build_cells()
    if args.cells:
        wanted = set(args.cells)
        cells = [c for c in cells if cell_name(c) in wanted]
        missing = wanted - {cell_name(c) for c in cells}
        if missing:
            raise SystemExit(f"unknown cells: {sorted(missing)}; "
                             f"known: {[cell_name(c) for c in build_cells()]}")
    return cells


# ----------------------------------------------------------------------
# stages
# ----------------------------------------------------------------------


def check_instances(args, cells: Sequence[Dict[str, float]]) -> List[str]:
    """Build every (cell, instance) evaluator; a misconfigured one raises.

    The home pose must pass the conditioning floor in some posture, so a
    strict ``sigma_min`` can make an instance unusable before the GA starts.
    Better to find that here than in a worker 40 minutes in.
    """
    instances = build_instances(args.split)[: args.instances or None]
    problems = []
    for settings in cells:
        for instance in instances:
            try:
                evaluator_for(instance, settings)
            except ValueError as error:
                problems.append(f"{cell_name(settings)} / {instance.name}: {error}")
    return problems


def stage_plan(args) -> None:
    cells = selected_cells(args)
    jobs = build_jobs(args, cells)
    n_instances = args.instances or len(build_instances(args.split))
    scale = (args.generations / 300.0) * (args.population / 200.0)
    cpu = sum(CPU_PER_RUN[job.arm] for job in jobs) * scale
    hours = cpu / (args.workers * PARALLEL_EFFICIENCY) / 3600.0

    print(f"{len(cells)} cells x {len(ARMS)} arms x {n_instances} instances x "
          f"{args.runs} seeds = {len(jobs)} runs")
    print(f"P={args.population}, G={args.generations}, crossover {args.crossover_rate}, "
          f"mutation {args.mutation_rate}, {args.workers} workers")
    for settings in cells:
        print(f"  {cell_name(settings):<22} sigma_min={settings['sigma_min']:<5g} "
              f"arm_clearance={settings['arm_clearance']:<6g} retract={settings['retract']:g}")
    print(f"estimated CPU {cpu / 3600:.1f} h; wall clock about {hours:.1f} h "
          f"on {args.workers} workers (default-setting cost; cells that admit "
          f"more layouts run longer)")


def stage_run(args) -> None:
    cells = selected_cells(args)
    problems = check_instances(args, cells)
    if problems:
        raise SystemExit("unusable instances:\n  " + "\n  ".join(problems))
    jobs = build_jobs(args, cells)
    stage_plan(args)
    started = time.perf_counter()
    run_grid(jobs, args.workers, args.resume)
    print(f"  done in {(time.perf_counter() - started) / 60:.1f} min")

    metadata = {
        "defaults": DEFAULTS,
        "sweeps": SWEEPS,
        "cells": [dict(s, name=cell_name(s)) for s in cells],
        "arms": list(ARMS),
        "split": args.split,
        "instances": [describe(i) | {"name": i.name}
                      for i in build_instances(args.split)[: args.instances or None]],
        "runs": args.runs,
        "dwell": args.dwell,
        "config": GAConfig(
            population_size=args.population, generations=args.generations,
            crossover_rate=args.crossover_rate, mutation_rate=args.mutation_rate,
            elitism_rate=args.elitism_rate, tournament_k=args.tournament_k,
            mutation_sigma=args.mutation_sigma,
        ).to_dict(),
        "platform": platform.platform(),
        "python": platform.python_version(),
        "numpy": np.__version__,
    }
    os.makedirs(args.output, exist_ok=True)
    with open(os.path.join(args.output, "metadata.json"), "w", encoding="utf-8") as handle:
        json.dump(metadata, handle, indent=2)


# -- aggregate ----------------------------------------------------------


def pairs_of(records: Sequence[Dict]) -> List[Dict[str, Dict]]:
    grouped: Dict[Tuple[str, int], Dict[str, Dict]] = {}
    for record in records:
        grouped.setdefault((record["instance"], record["run"]), {})[record["arm"]] = record
    return [arms for _, arms in sorted(grouped.items()) if set(arms) == set(ARMS)]


def paired_gain(pairs: Sequence[Dict[str, Dict]]) -> Dict:
    """Wall minus repair, so positive means repair found the shorter cycle.

    A pair where either arm found no feasible layout is dropped and counted.
    """
    d = [p["baseline"]["cycle_time"] - p["repair"]["cycle_time"]
         for p in pairs
         if p["baseline"]["cycle_time"] is not None and p["repair"]["cycle_time"] is not None]
    out = {"n": len(d), "dropped": len(pairs) - len(d)}
    if not d:
        return out
    d = np.asarray(d, dtype=float)
    low, high = bootstrap_ci(d)
    out.update(
        gain=float(d.mean()), ci_low=low, ci_high=high,
        wins=int((d > 0).sum()), losses=int((d < 0).sum()),
        perm_p=sign_flip_test(d), wilcoxon_p=wilcoxon_signed_rank(d)[1],
    )
    return out


def at_equal_cpu(pairs: Sequence[Dict[str, Dict]]) -> Tuple[List[Dict[str, Dict]], float]:
    """Each pair re-read at the smaller of its two CPU totals."""
    out, budgets = [], []
    for arms in pairs:
        budget = min(float(r["cpu_time"]) for r in arms.values())
        budgets.append(budget)
        out.append({arm: truncate_to_cpu(r, budget) for arm, r in arms.items()})
    return out, float(np.mean(budgets)) if budgets else float("nan")


def _mean(values) -> Optional[float]:
    kept = [float(v) for v in values if v is not None and math.isfinite(float(v))]
    return float(np.mean(kept)) if kept else None


def summarise_cell(settings: Dict[str, float], pairs: Sequence[Dict[str, Dict]]) -> Dict:
    row: Dict = {"cell": cell_name(settings), **settings, "pairs": len(pairs)}
    for arm in ARMS:
        runs = [p[arm] for p in pairs]
        row[f"{arm}_T"] = _mean(r["cycle_time"] for r in runs)
        row[f"{arm}_no_layout"] = sum(r["cycle_time"] is None for r in runs)
        row[f"{arm}_feasible_share"] = _mean(r["feasible_eval_share"] for r in runs)
        row[f"{arm}_cpu"] = _mean(r["cpu_time"] for r in runs)
    for prefix, view in (("gen", pairs), ("cpu", at_equal_cpu(pairs)[0])):
        for key, value in paired_gain(view).items():
            row[f"{prefix}_{key}"] = value
    row["cpu_budget"] = at_equal_cpu(pairs)[1]
    return row


def stage_aggregate(args) -> List[Dict]:
    rows = []
    for settings in build_cells():
        directory = os.path.join(args.output, cell_name(settings))
        pairs = pairs_of(collect_records(directory))
        if pairs:
            rows.append(summarise_cell(settings, pairs))
    if not rows:
        print("nothing to aggregate")
        return rows

    # One family: every cell at both budgets.  Wilcoxon, because the
    # permutation p has a floor that Holm cannot resolve (see holm.py).
    family = {f"{r['cell']}|{b}": r[f"{b}_wilcoxon_p"]
              for r in rows for b in ("gen", "cpu") if r.get(f"{b}_wilcoxon_p") is not None}
    adjusted = holm(family)
    for r in rows:
        for b in ("gen", "cpu"):
            r[f"{b}_holm_p"] = adjusted.get(f"{r['cell']}|{b}")

    fields = list(rows[0].keys())
    for r in rows:
        fields += [k for k in r if k not in fields]
    _write_csv(os.path.join(args.output, "summary.csv"), rows, fields)
    _write_summary_md(os.path.join(args.output, "summary.md"), rows)
    print(f"  wrote {os.path.join(args.output, 'summary.md')}")
    return rows


def _f(value, spec=".3f") -> str:
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "--"
    return format(value, spec)


def _write_csv(path: str, rows: Sequence[Dict], fields: Sequence[str]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(fields), extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)


def _write_summary_md(path: str, rows: Sequence[Dict]) -> None:
    by_cell = {r["cell"]: r for r in rows}
    lines = [
        "# Sensitivity of the repair gain to the feasibility thresholds",
        "",
        "One parameter changed at a time from the defaults "
        f"(sigma_min {DEFAULTS['sigma_min']} m/rad, arm clearance "
        f"{DEFAULTS['arm_clearance'] * 1000:g} mm, retract {DEFAULTS['retract']} m).",
        "Gain = wall minus repair cycle time, paired by (instance, seed); positive "
        "favours repair. Cycle times are not comparable across rows: each row is a "
        "different problem.",
        "Holm correction over every row at both budgets, on the Wilcoxon p.",
        "",
    ]
    for key, values in SWEEPS.items():
        lines += [
            f"## {key} [{UNITS[key]}]",
            "",
            "| value | pairs | feasible share wall / repair | no layout wall / repair "
            "| T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p "
            "| gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |",
            "| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |",
        ]
        for value in values:
            name = cell_name(dict(DEFAULTS, **{key: value}))
            r = by_cell.get(name)
            if r is None:
                lines.append(f"| {value:g} | not run |" + " |" * 11)
                continue
            mark = " (default)" if name == "default" else ""
            lines.append(
                f"| {value:g}{mark} | {r['pairs']} "
                f"| {_f(r['baseline_feasible_share'])} / {_f(r['repair_feasible_share'])} "
                f"| {r['baseline_no_layout']} / {r['repair_no_layout']} "
                f"| {_f(r['baseline_T'])} | {_f(r['repair_T'])} "
                f"| {_f(r.get('gen_gain'))} [{_f(r.get('gen_ci_low'))}, {_f(r.get('gen_ci_high'))}] "
                f"| {r.get('gen_wins', '--')}/{r.get('gen_n', '--')} | {_f(r.get('gen_holm_p'), '.4f')} "
                f"| {_f(r.get('cpu_gain'))} [{_f(r.get('cpu_ci_low'))}, {_f(r.get('cpu_ci_high'))}] "
                f"| {r.get('cpu_wins', '--')}/{r.get('cpu_n', '--')} | {_f(r.get('cpu_holm_p'), '.4f')} "
                f"| {_f(r['baseline_cpu'], '.1f')} / {_f(r['repair_cpu'], '.1f')} |"
            )
        lines.append("")
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


# -- pathcheck ------------------------------------------------------------


def pathcheck_record(record: Dict, instances: Dict[str, object]) -> Optional[Dict]:
    """Re-evaluate a run's best layout with dense path sampling."""
    if record.get("cycle_time") is None:
        return None
    instance = instances[record["instance"]]
    settings = {
        "sigma_min": float(record.get("sigma_min_threshold", DEFAULTS["sigma_min"])),
        "arm_clearance": float(record.get("arm_clearance", DEFAULTS["arm_clearance"])),
        "retract": float(record.get("retract", DEFAULTS["retract"])),
    }
    dense = evaluator_for(instance, settings, dwell=float(record.get("dwell", 0.30)),
                          path_samples=DENSE_PATH_SAMPLES)
    outcome = dense.evaluate(dense.decode(np.asarray(record["chromosome"], dtype=float)))

    min_gap = None
    if outcome.feasible:
        tour = outcome.configurations
        fractions = np.linspace(0.0, 1.0, DENSE_PATH_SAMPLES + 2)
        samples = tour[:-1, None, :] + fractions[None, :, None] * (tour[1:] - tour[:-1])[:, None, :]
        gaps = dense.arm_clearances(samples.reshape(-1, tour.shape[1]), outcome.machines)
        min_gap = float(np.min(gaps))
    return {
        "arm": record["arm"], "instance": record["instance"], "run": record["run"],
        "arm_clearance": settings["arm_clearance"],
        "cycle_time_12": record["cycle_time"],
        "feasible_dense": outcome.feasible,
        "rejection_dense": outcome.rejection,
        "cycle_time_dense": outcome.cycle_time if outcome.feasible else None,
        "min_gap_dense": min_gap,
    }


def stage_pathcheck(args) -> None:
    sources = [(cell_name(s), os.path.join(args.output, cell_name(s))) for s in build_cells()]
    sources.append(("popgen/p200", results_dir(os.path.join("popgen", "p200"))))
    instances = {i.name: i for i in build_instances(args.split)}

    rows, table = [], []
    for label, directory in sources:
        checked = [c for c in (pathcheck_record(r, instances) for r in collect_records(directory))
                   if c is not None]
        if not checked:
            continue
        for c in checked:
            c["source"] = label
        rows += checked
        failed = [c for c in checked if not c["feasible_dense"]]
        # The stored chromosome is rounded to 6 decimals, which alone moves the
        # cycle time by a few microseconds; a posture change moves it by far
        # more than a millisecond.
        changed = [c for c in checked if c["feasible_dense"]
                   and abs(c["cycle_time_dense"] - c["cycle_time_12"]) > 1e-3]
        gaps = [c["min_gap_dense"] for c in checked if c["min_gap_dense"] is not None]
        table.append(
            f"| {label} | {len(checked)} | {len(failed)} | {len(changed)} "
            f"| {_f(min(gaps) * 1000 if gaps else None, '.2f')} "
            f"| {_f(checked[0]['arm_clearance'] * 1000, '.0f')} |"
        )

    if not rows:
        print("no runs to check")
        return
    _write_csv(os.path.join(args.output, "pathcheck.csv"), rows, list(rows[0].keys()) + ["source"])
    lines = [
        "# Dense clearance re-check of every best layout",
        "",
        f"Each run's best layout re-evaluated with {DENSE_PATH_SAMPLES} samples per move "
        "instead of 12. 'Rejected' layouts were accepted at 12 samples and fail at "
        f"{DENSE_PATH_SAMPLES}; 'time changed' means a different posture won.",
        "",
        "| source | layouts | rejected when dense | time changed | smallest gap [mm] | required gap [mm] |",
        "| --- | ---: | ---: | ---: | ---: | ---: |",
        *table,
        "",
    ]
    path = os.path.join(args.output, "pathcheck.md")
    os.makedirs(args.output, exist_ok=True)
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))
    print(f"  wrote {path}")


# ----------------------------------------------------------------------
# entry point
# ----------------------------------------------------------------------


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--only", choices=STAGES, action="append", dest="stages")
    parser.add_argument("--dry-run", action="store_true",
                        help="print the plan and cost, check every instance, run nothing")
    parser.add_argument("--cells", nargs="+",
                        help="run only these cells, e.g. default sigma_min=0.21")
    parser.add_argument("--split", default="test")
    parser.add_argument("--instances", type=int, default=0, help="first N instances (0 = all)")
    parser.add_argument("--runs", type=int, default=5, help="seeds per instance")
    parser.add_argument("--generations", type=int, default=300)
    parser.add_argument("--population", type=int, default=200)
    parser.add_argument("--crossover-rate", type=float, default=0.8)
    parser.add_argument("--mutation-rate", type=float, default=0.2)
    parser.add_argument("--elitism-rate", type=float, default=0.1)
    parser.add_argument("--tournament-k", type=int, default=3)
    parser.add_argument("--mutation-sigma", type=float, default=0.10)
    parser.add_argument("--dwell", type=float, default=0.30)
    parser.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) - 2))
    parser.add_argument("--output", default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--no-resume", dest="resume", action="store_false")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    if args.dry_run:
        stage_plan(args)
        problems = check_instances(args, selected_cells(args))
        print("instance check: " + ("all usable" if not problems else
                                    "UNUSABLE\n  " + "\n  ".join(problems)))
        return
    stages = args.stages or list(STAGES)
    if "run" in stages:
        stage_run(args)
    if "aggregate" in stages:
        stage_aggregate(args)
    if "pathcheck" in stages:
        stage_pathcheck(args)


if __name__ == "__main__":
    main()
