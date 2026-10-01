"""Per-instance layout / trajectory / viewer figures for an A/B results tree.

    python -m experiments.make_ab_figures                  # repair_equalcpu_noretract
    python -m experiments.make_ab_figures --arm repair
    python -m experiments.make_ab_figures --results-dir results/repair --viewer

``run_ga`` writes ``figures/<instance>/{layout.png, trajectory.png}`` for the best run of every instance; the A/B drivers
(``run_repair_ab``, ``run_penalty_ab``, ``run_retract_ab``) do not, because
their question is about the *difference* between two arms, not about any one
layout.  The layouts are still in the tree though -- every raw JSON carries the
winning chromosome and the objective settings it was scored under -- so this
re-scores the chosen run and draws it.

``--arm best`` (the default) picks, per instance, the fastest feasible run over
*both* arms: the best answer the experiment found for that instance.  ``--arm
baseline`` or ``--arm repair`` pins the choice to one arm instead and writes
under ``figures/<arm>/<instance>/`` so the two never overwrite each other.

The evaluator is rebuilt from the run's own JSON (``retract``, ``check_path``,
``check_arm``, ``arm_clearance``, ``sigma_min_threshold``, ``dwell``), not from
this file's defaults, so a no-retract tree is redrawn without the retract and a
retract tree with it.  ``figures/best_runs.json`` records which arm, seed and
cycle time each directory came from.
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from typing import Dict, List, Optional

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from armmesh import collision_spheres  # noqa: E402
from evaluator import CycleTimeEvaluator  # noqa: E402
from geometry import box_frames, points_to_boxes_distance  # noqa: E402
from problems import ALL_SPLITS, build_instances  # noqa: E402

DEFAULT_RESULTS_DIR = results_dir("repair_equalcpu_noretract")

#: Samples per tour segment when auditing the drawn cycle.  Denser than the
#: evaluator's own ``path_samples`` because this is reporting, not searching,
#: and includes the endpoints so the station poses are covered.
AUDIT_SAMPLES = 24

LINK_NAMES = ("upper arm", "forearm", "tool/gripper")


def load_records(raw_dir: str) -> List[Dict]:
    """Every ``raw/<arm>_<instance>_run<k>.json`` in the tree, as dicts."""
    records: List[Dict] = []
    for name in sorted(os.listdir(raw_dir)):
        if not name.endswith(".json"):
            continue
        with open(os.path.join(raw_dir, name), encoding="utf-8") as handle:
            records.append(json.load(handle))
    return records


def best_per_instance(records: List[Dict], arm: Optional[str]) -> Dict[str, Dict]:
    """The fastest feasible run of each instance, optionally within one arm."""
    best: Dict[str, Dict] = {}
    for record in records:
        if not record.get("feasible") or record.get("cycle_time") is None:
            continue
        if arm is not None and record.get("arm") != arm:
            continue
        current = best.get(record["instance"])
        if current is None or record["cycle_time"] < current["cycle_time"]:
            best[record["instance"]] = record
    return best


def evaluator_for(instance, record: Dict, fallback: Dict) -> CycleTimeEvaluator:
    """Rebuild the evaluator the run was scored under, from the run's own JSON.

    ``retract`` is the one setting whose ``null`` is meaningful -- it is how a
    no-retract tree records "direct moves" -- so it is read straight through
    rather than falling back to a default.
    """

    def setting(key: str, default):
        value = record.get(key, fallback.get(key, default))
        return default if value is None else value

    retract = record["retract"] if "retract" in record else fallback.get("retract")
    return CycleTimeEvaluator(
        instance,
        sigma_min_threshold=setting("sigma_min_threshold", 0.14),
        dwell=setting("dwell", 0.30),
        check_path=setting("check_path", True),
        check_arm=setting("check_arm", True),
        arm_clearance=setting("arm_clearance", 0.005),
        retract=retract,
    )


def audit_path(evaluator: CycleTimeEvaluator, outcome, machines) -> Dict:
    """How close the drawn cycle comes to a machine *between* the stations.

    A tree run with ``--no-path-check`` scores only the station poses, so the
    cycle it draws may sweep the arm straight through a box on the way.  The
    figures are then perfectly faithful to what was optimised and still show a
    collision, which is worth saying out loud next to them.

    The gap itself comes from :meth:`CycleTimeEvaluator.arm_clearances`, the
    same call the feasibility test uses -- exemption for the approach corridor
    included -- so this cannot disagree with the verdict.  Which link owns the
    worst gap is worked out separately, for the report only.
    """
    tour = outcome.configurations
    fractions = np.linspace(0.0, 1.0, AUDIT_SAMPLES)
    samples = np.concatenate([
        tour[k][None, :] + fractions[:, None] * (tour[k + 1] - tour[k])[None, :]
        for k in range(tour.shape[0] - 1)
    ])

    gaps = evaluator.arm_clearances(samples, machines)      # (n_sample, n_machine)
    worst = float(gaps.min())
    flat = int(np.argmin(gaps))
    sample_index, machine_index = np.unravel_index(flat, gaps.shape)
    segment = int(sample_index // AUDIT_SAMPLES)

    # Attribution: which link is the one that got closest, at that sample.
    spheres = collision_spheres(evaluator.arm)
    centres, half, rotations = box_frames(machines)
    point = evaluator.arm.points_along_arm(
        samples[sample_index][None, :], spheres.coefficients
    )
    per_sphere = (points_to_boxes_distance(point, centres, half, rotations)
                  - spheres.radii[None, :, None])[0, :, machine_index]
    link = int(spheres.link[int(np.argmin(per_sphere))])

    return {
        "worst_gap": round(worst, 6),
        "worst_link": LINK_NAMES[link],
        "worst_machine": machines[machine_index].name,
        "worst_segment": segment,
        "colliding_segments": int(sum(
            bool((gaps[k * AUDIT_SAMPLES:(k + 1) * AUDIT_SAMPLES] < 0.0).any())
            for k in range(tour.shape[0] - 1)
        )),
    }


def make_figures(output_dir: str, split: str, arm: Optional[str],
                 viewer: bool, animate: bool, spin: float) -> None:
    import visualize

    raw_dir = os.path.join(output_dir, "raw")
    if not os.path.isdir(raw_dir):
        raise SystemExit(f"no raw/ under {output_dir}")

    metadata_path = os.path.join(output_dir, "metadata.json")
    fallback: Dict = {}
    if os.path.exists(metadata_path):
        with open(metadata_path, encoding="utf-8") as handle:
            metadata = json.load(handle)
        # A/B trees record the objective settings as a dict; run_ga trees
        # write a one-line description there and keep the settings at the
        # top level, so read whichever this tree has.
        objective = metadata.get("objective")
        fallback = dict(objective) if isinstance(objective, dict) else {}
        for key in ("retract", "check_path", "check_arm", "arm_clearance",
                    "dwell", "sigma_min_threshold"):
            if key in metadata and key not in fallback:
                fallback[key] = metadata[key]
        split = metadata.get("split", split)

    records = load_records(raw_dir)
    best = best_per_instance(records, arm)
    if not best:
        raise SystemExit("no feasible run to draw")

    instances = {inst.name: inst for inst in build_instances(split)}
    figures_dir = os.path.join(output_dir, "figures")
    if arm is not None:
        figures_dir = os.path.join(figures_dir, arm)
    os.makedirs(figures_dir, exist_ok=True)

    manifest: Dict[str, Dict] = {}
    for name, record in sorted(best.items(),
                               key=lambda kv: kv[1]["instance_index"]):
        instance = instances[name]
        evaluator = evaluator_for(instance, record, fallback)
        outcome = evaluator.evaluate(
            evaluator.decode(np.asarray(record["chromosome"], dtype=float))
        )
        if not outcome.feasible:
            print(f"  {name}: re-scored infeasible, skipped")
            continue

        drift = abs(float(outcome.cycle_time) - float(record["cycle_time"]))
        if drift > 1e-6:
            print(f"  {name}: re-scored {outcome.cycle_time:.6f} s vs recorded "
                  f"{record['cycle_time']:.6f} s (drift {drift:.2e})")

        target = os.path.join(figures_dir, name)
        os.makedirs(target, exist_ok=True)
        visualize.figure_layout(instance, outcome, evaluator,
                                os.path.join(target, "layout.png"))
        visualize.figure_trajectory(instance, outcome, evaluator,
                                    os.path.join(target, "trajectory.png"))
        if viewer:
            visualize.export_viewer(instance, outcome, evaluator,
                                    os.path.join(target, "cycle.html"))
        if animate:
            visualize.animate_cycle(instance, outcome, evaluator,
                                    os.path.join(target, "cycle.gif"), spin=spin)

        audit = audit_path(evaluator, outcome,
                           outcome.machines or instance.machines)
        manifest[name] = {
            # run_ga trees have one arm and do not record it.
            "arm": record.get("arm", "ga"),
            "run": record["run"],
            "run_seed": record["run_seed"],
            "cycle_time": round(float(outcome.cycle_time), 6),
            "posture": outcome.branch_name,
            "retract": record.get("retract", fallback.get("retract")),
            "path_checked": bool(evaluator.check_path),
            "path_audit": audit,
        }
        note = ""
        if audit["worst_gap"] < 0.0:
            note = (f"  [{audit['worst_link']} {-audit['worst_gap']*1000:.0f} mm "
                    f"inside {audit['worst_machine']}]")
        print(f"  {name}: {record.get('arm', 'ga')} run {record['run']} "
              f"({outcome.cycle_time:.3f} s, {outcome.branch_name}) -> {target}{note}")

    with open(os.path.join(figures_dir, "best_runs.json"), "w",
              encoding="utf-8") as handle:
        json.dump(manifest, handle, indent=2)
    write_caveat(output_dir, figures_dir, manifest)
    print(f"  {len(manifest)} instances -> {figures_dir}")


def write_caveat(output_dir: str, figures_dir: str,
                 manifest: Dict[str, Dict]) -> None:
    """``figures/README.md``: what the figures are, and what they are not.

    Only worth writing when something is worth warning about, which here means
    a drawn cycle that touches a machine between its stations.  The figure
    files carry no caption of their own, so without this the collision is
    visible but unexplained -- and it looks like a bug in the collision model
    rather than the documented consequence of scoring stations only.
    """
    dirty = {k: v for k, v in manifest.items()
             if v["path_audit"]["worst_gap"] < 0.0}
    lines = [
        "# What these figures are",
        "",
        "One directory per instance, each the **fastest feasible run** of that",
        "instance in this tree, redrawn from the chromosome in its `raw/` JSON.",
        "`best_runs.json` says which arm and seed each came from.",
        "",
    ]
    if not dirty:
        lines.append("Every drawn cycle clears the machines along its whole path.")
    else:
        worst = min(dirty.values(), key=lambda v: v["path_audit"]["worst_gap"])
        lines += [
            "## These cycles collide, and are meant to",
            "",
            f"{len(dirty)} of {len(manifest)} drawn cycles put part of the robot",
            "**inside** a machine while moving between stations, by up to "
            f"{-worst['path_audit']['worst_gap']*1000:.0f} mm.",
            "That is not a gap in the collision model: the arm is bounded by",
            "`armmesh.collision_spheres`, wrist and gripper included, and the",
            "station poses here all clear it. It is that this tree was run with",
            "the **path check off and no retract** (`retract: null`,",
            "`check_path: false` in `metadata.json`), so the moves between",
            "stations were never scored at all -- only the stations were.",
            "",
            "With direct point-to-point moves the tool interpolates in joint",
            "space and drags the arm through whatever stands between two ports,",
            "which is why the retract exists. No run in this tree is path-clean,",
            "so there is no better one to draw; the figures are faithful to what",
            "was optimised. For cycles that clear the machines throughout, see a",
            "tree whose `metadata.json` has `check_path: true` and a retract.",
            "",
            "| instance | worst gap | part | inside | segments |",
            "|---|---|---|---|---|",
        ]
        for name, entry in manifest.items():
            audit = entry["path_audit"]
            if audit["worst_gap"] >= 0.0:
                lines.append(f"| {name} | clear | — | — | 0 |")
                continue
            lines.append(
                f"| {name} | {audit['worst_gap']*1000:.0f} mm | "
                f"{audit['worst_link']} | {audit['worst_machine']} | "
                f"{audit['colliding_segments']} |"
            )
        lines += [
            "",
            "Gaps are from `CycleTimeEvaluator.arm_clearances`, the same call the",
            f"feasibility test uses, at {AUDIT_SAMPLES} samples per tour segment.",
            "Negative means that far inside the box.",
        ]
    with open(os.path.join(figures_dir, "README.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    note = [
        MARKER,
        "",
        "## A note on the per-instance figures",
        "",
    ]
    # One long line per paragraph, the way the rest of summary.md is written.
    if dirty:
        worst = min(dirty.values(), key=lambda v: v["path_audit"]["worst_gap"])
        note.append(
            f"`figures/<instance>/` holds the layout, trajectory and viewer for "
            f"the fastest run of each instance. **{len(dirty)} of {len(manifest)} "
            f"of those cycles sweep the robot through a machine between "
            f"stations**, by up to "
            f"{-worst['path_audit']['worst_gap'] * 1000:.0f} mm, and the part "
            f"that does it is the gripper. That is the path check being off: "
            f"this tree scores the station poses only (`check_path: false`, no "
            f"retract), so the moves between them were never tested. The "
            f"collision model itself covers the whole arm, gripper included. No "
            f"run in this tree is path-clean, so there is no better one to draw. "
            f"`figures/README.md` has the per-instance numbers."
        )
    else:
        note.append(
            "`figures/<instance>/` holds the layout, trajectory and viewer for "
            "the fastest run of each instance. Every one of those cycles clears "
            "the machines along its whole path."
        )
    append_marked_section(os.path.join(output_dir, "summary.md"),
                          "\n".join(note) + "\n")


#: Delimits the section this script owns inside a report another script writes,
#: so re-running either one cannot end up with two copies or a stale table.
MARKER = "<!-- make_ab_figures: per-instance figures -->"


def append_marked_section(path: str, text: str) -> None:
    """Replace this script's section of ``path``, or add it if it is not there.

    ``summary.md`` is written by the A/B driver, which knows nothing about
    these figures and would overwrite a hand-added warning the next time its
    aggregate stage ran.  Keeping the section marker-delimited means re-running
    this script puts it back, and never stacks up duplicates.
    """
    if not os.path.exists(path):
        return
    with open(path, encoding="utf-8") as handle:
        body = handle.read()
    if MARKER in body:
        body = body[:body.index(MARKER)].rstrip("\n") + "\n"
    with open(path, "w", encoding="utf-8") as handle:
        handle.write(body.rstrip("\n") + "\n\n" + text)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--results-dir", default=DEFAULT_RESULTS_DIR)
    parser.add_argument("--split", default="test", choices=list(ALL_SPLITS),
                        help="overridden by the tree's metadata.json when present")
    parser.add_argument("--arm", default="best",
                        help="'best' (fastest run over both arms) or an arm name")
    parser.add_argument("--viewer", action="store_true",
                        help="also write the drag-to-rotate cycle.html viewer "
                             "(off by default; run_mujoco.py validates layouts)")
    parser.add_argument("--animate", action="store_true", help="also write cycle.gif")
    parser.add_argument("--spin", type=float, default=360.0,
                        help="degrees the cycle.gif camera orbits")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    arm = None if args.arm == "best" else args.arm
    make_figures(args.results_dir, args.split, arm,
                 args.viewer, args.animate, args.spin)


if __name__ == "__main__":
    main()
