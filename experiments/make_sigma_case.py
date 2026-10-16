"""A layout rejected on ``sigma_min`` alone, played through as a cycle.

    python -m experiments.make_sigma_case
    python -m experiments.make_sigma_case --parent results/popgen/p100/raw/baseline_test_1_run0.json --seed 132

Takes the best layout of a popgen run, mutates one machine exactly as
:meth:`ConfigurableGA.mutate` does (the same draw as ``make_repair_figure``),
and keeps the child only if the conditioning test is the *only* thing it
fails: the evaluator rejects it at ``'sigma_min'``, and the same evaluator
with ``sigma_min_threshold = 0`` accepts it.  That second evaluation is the
cycle the robot would run if the test were switched off, and it is the one
drawn -- every other check (placement, reach, joint travel, clearance at the
stations and along the path) still passes on it.

The default, test_9 seed 19, moves M4 51 mm outward and turns it: its port
lands 0.77 m from the base, the elbow straightens to about -4 deg, and
``sigma_min`` there drops to ~0.014 m/rad, a tenth of the 0.14 threshold, in
every one of the four postures.  The parent passes with 0.149 at that station.

Writes ``cycle.html`` (the usual viewer, plus a ``sigma_min``-over-time strip
with the stations marked, the offending machine and port highlighted, and a
banner while the arm dwells at the station that fails) and ``case.json`` with
the numbers.
"""

from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from evaluator import CycleTimeEvaluator  # noqa: E402
from make_repair_figure import mutate_one  # noqa: E402
from problems import build_instances  # noqa: E402
from robot import BRANCHES, BRANCH_NAMES  # noqa: E402
import visualize  # noqa: E402

DEFAULT_PARENT = os.path.join(results_dir("popgen"), "p100", "raw", "baseline_test_9_run0.json")
DEFAULT_OUT = os.path.join(results_dir("popgen"), "sigma_case")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--parent", default=DEFAULT_PARENT,
                        help="run JSON whose best chromosome is mutated")
    parser.add_argument("--seed", type=int, default=19, help="mutation seed")
    parser.add_argument("--out", default=DEFAULT_OUT, help="folder for cycle.html and case.json")
    args = parser.parse_args()

    with open(args.parent, encoding="utf-8") as handle:
        record = json.load(handle)
    instance = next(i for i in build_instances("test") if i.name == record["instance"])
    threshold = float(record.get("sigma_min_threshold", 0.14))
    settings = dict(dwell=record.get("dwell", 0.30), retract=record.get("retract", 0.06),
                    check_path=record.get("check_path", True),
                    check_arm=record.get("check_arm", True),
                    arm_clearance=record.get("arm_clearance", 0.005))
    evaluator = CycleTimeEvaluator(instance, sigma_min_threshold=threshold, **settings)
    unconditioned = CycleTimeEvaluator(instance, sigma_min_threshold=0.0, **settings)

    parent = np.asarray(record["chromosome"], dtype=float).reshape(-1, 3)
    child, k = mutate_one(instance, parent, args.seed)
    layout = evaluator.decode(child.ravel())

    parent_out = evaluator.evaluate(evaluator.decode(parent.ravel()))
    child_out = evaluator.evaluate(layout)
    run_out = unconditioned.evaluate(layout)
    if child_out.rejection != "sigma_min" or not run_out.feasible:
        sys.exit(f"seed {args.seed}: child rejected at {child_out.rejection!r}, "
                 f"{'feasible' if run_out.feasible else 'infeasible'} without the sigma test "
                 "-- not a sigma-only case, try another seed")

    sigma = run_out.sigma_min                       # per station, process order
    failing = [int(s) for s in np.flatnonzero(sigma < threshold)]
    bad_machines = [int(instance.sequence[s]) for s in failing]
    stations = run_out.stations
    worst = int(np.argmin(sigma))

    # Every posture, not just the one drawn: the point is that none escapes.
    ports = run_out.waypoints
    per_branch = {}
    for branch in BRANCHES:
        q, ok = evaluator.arm.ik_vertical(ports, *branch)
        per_branch[BRANCH_NAMES[branch]] = (
            round(float(evaluator.arm.sigma_min(q)[worst]), 4) if ok.all() else None)

    print(f"{instance.name}: mutated M{k + 1} from ({parent[k, 0]:.3f}, {parent[k, 1]:.3f}, "
          f"{parent[k, 2]:.1f} deg) to ({child[k, 0]:.3f}, {child[k, 1]:.3f}, {child[k, 2]:.1f} deg)")
    print(f"  parent : {parent_out.cycle_time:.3f} s, sigma_min per station {parent_out.sigma_min.round(4)}")
    print(f"  child  : rejected at {child_out.rejection!r}; with the test off it runs "
          f"{run_out.cycle_time:.3f} s ({run_out.branch_name})")
    print(f"  sigma  : {sigma.round(4)} (threshold {threshold:g})")
    print(f"  worst  : S{worst + 1} = M{instance.sequence[worst] + 1}, sigma_min {sigma[worst]:.4f}, "
          f"q = {np.degrees(stations[worst]).round(1)} deg, reach "
          f"{np.hypot(*(ports[worst, :2] - instance.base_xy)):.3f} m")
    print(f"  postures at S{worst + 1}: {per_branch}")

    station_q = np.asarray(stations)
    elbow = float(np.degrees(stations[worst][2]))

    def overlay(t: np.ndarray, q: np.ndarray) -> dict:
        # Which station each sampled instant is held at: the arrival sample and
        # the two dwell samples reproduce the station configuration exactly.
        gap = np.abs(q[:, None, :] - station_q[None, :, :]).max(axis=2)
        held = np.where(gap.min(axis=1) < 1e-9, gap.argmin(axis=1), -1)
        return {
            "sigma": evaluator.arm.sigma_min(q).round(5).tolist(),
            "sigmaThreshold": threshold,
            "station": held.astype(int).tolist(),
            "badPorts": bad_machines,
            "badReason": f"elbow at {elbow:.1f}°, nearly straight (lock-out)"
                         if abs(elbow) < 20 else "near-singular",
            "note": (f"REJECTED (sigma_min): M{k + 1} mutated, S{worst + 1} at "
                     f"{sigma[worst]:.4f} < {threshold:g} m/rad; shown with the test off"),
        }

    os.makedirs(args.out, exist_ok=True)
    visualize.export_viewer(instance, run_out, unconditioned,
                            os.path.join(args.out, "cycle.html"), extras=overlay)

    case = {
        "parent": os.path.relpath(args.parent, results_dir("")).replace(os.sep, "/"),
        "instance": instance.name,
        "mutation_seed": args.seed,
        "mutated_machine": k + 1,
        "gene_before": parent[k].round(6).tolist(),
        "gene_after": child[k].round(6).tolist(),
        "sigma_min_threshold": threshold,
        "rejection": child_out.rejection,
        "parent_cycle_time": round(float(parent_out.cycle_time), 4),
        "parent_sigma_min_per_station": parent_out.sigma_min.round(6).tolist(),
        "unconditioned_cycle_time": round(float(run_out.cycle_time), 4),
        "posture": run_out.branch_name,
        "sigma_min_per_station": sigma.round(6).tolist(),
        "failing_stations": [s + 1 for s in failing],
        "failing_machines": [m + 1 for m in bad_machines],
        "worst_station_q_deg": np.degrees(stations[worst]).round(3).tolist(),
        "worst_station_sigma_by_posture": per_branch,
        "chromosome": child.ravel().round(6).tolist(),
    }
    with open(os.path.join(args.out, "case.json"), "w", encoding="utf-8") as handle:
        json.dump(case, handle, indent=2)
    print(f"  -> {os.path.join(args.out, 'case.json')}")


if __name__ == "__main__":
    main()
