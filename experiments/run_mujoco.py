"""Put a GA layout into MuJoCo and check it under physics.

Picks a run from a results tree (by default the fastest feasible one of the
chosen instance), rebuilds the evaluator it was scored under, re-scores it,
and then asks the MuJoCo model of :mod:`mujoco_cell` whether

* the solid arm clears the machines along the whole cycle,
* the motors can supply the torque the planned motion needs, and
* a computed-torque controller follows the plan under gravity, friction,
  reflected motor inertia, contact and torque saturation.

Writes ``results/mujoco/<tree>_<instance>/``: ``scene.xml`` (open it in
MuJoCo's ``simulate`` app, or ``python -m mujoco.viewer --mjcf scene.xml``),
``report.md`` / ``report.json``, ``torques.png``, ``tracking.png`` and
``cycle.gif``.

    python -m experiments.run_mujoco                       # best run of test_1 in results/retract
    python -m experiments.run_mujoco --instance test_3 --view
    python -m experiments.run_mujoco --results retract_ab --instance test_2 --run 4
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

from make_ab_figures import best_per_instance, evaluator_for, load_records  # noqa: E402
from mujoco_cell import (  # noqa: E402
    JOINT_NAMES,
    build_cell,
    mesh_clearance,
    render_frames,
    required_torques,
    sample_reference,
    track_cycle,
)
from problems import build_instances  # noqa: E402


def pick_record(tree: str, instance: str, run: Optional[int]) -> tuple:
    """``(record, fallback)`` -- the chosen run's JSON and the tree's defaults."""
    output_dir = results_dir(tree)
    records = load_records(os.path.join(output_dir, "raw"))
    fallback: Dict = {}
    metadata_path = os.path.join(output_dir, "metadata.json")
    if os.path.exists(metadata_path):
        with open(metadata_path, encoding="utf-8") as handle:
            metadata = json.load(handle)
        objective = metadata.get("objective")
        fallback = dict(objective) if isinstance(objective, dict) else {}
        for key in ("retract", "check_path", "check_arm", "arm_clearance", "dwell",
                    "sigma_min_threshold", "split"):
            if key in metadata and key not in fallback:
                fallback[key] = metadata[key]

    if run is None:
        record = best_per_instance(records, None).get(instance)
        if record is None:
            raise SystemExit(f"no feasible run of {instance} in {output_dir}")
        return record, fallback
    for record in records:
        if record["instance"] == instance and record["run"] == run and record.get("feasible"):
            return record, fallback
    raise SystemExit(f"no feasible run {run} of {instance} in {output_dir}")


def write_plots(target: str, reference, demand, tracking) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    driven = (0, 1, 2, 4)
    fig, axes = plt.subplots(len(driven), 1, figsize=(9, 8), sharex=True)
    for ax, j in zip(axes, driven):
        ax.plot(reference.t, demand.tau[:, j], lw=1.2, label="planned motion needs")
        ax.plot(tracking.t, tracking.tau[:, j], lw=0.8, alpha=0.8, label="controller applied")
        for sign in (1, -1):
            ax.axhline(sign * demand.limit[j], color="crimson", ls="--", lw=0.9)
        ax.set_ylabel(f"{JOINT_NAMES[j]}\n[N m]")
        ax.grid(alpha=0.3)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[0].set_title("Joint torque along the cycle (dashed: torque limit)")
    axes[-1].set_xlabel("time [s]")
    fig.tight_layout()
    fig.savefig(os.path.join(target, "torques.png"), dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(9, 5.5), sharex=True)
    axes[0].plot(tracking.t, tracking.tip_error * 1000.0, lw=1.0)
    axes[0].set_ylabel("tool tip error [mm]")
    axes[0].grid(alpha=0.3)
    axes[0].set_title("Tracking the planned cycle under physics")
    for j in driven:
        axes[1].plot(tracking.t, np.degrees(tracking.q[:, j] - reference.q[:, j]),
                     lw=0.9, label=JOINT_NAMES[j])
    axes[1].set_ylabel("joint error [deg]")
    axes[1].set_xlabel("time [s]")
    axes[1].legend(fontsize=8, ncol=4)
    axes[1].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(os.path.join(target, "tracking.png"), dpi=130)
    plt.close(fig)


def write_gif(cell, q_path: np.ndarray, dt: float, path: str, fps: int = 25) -> None:
    from PIL import Image

    every = max(1, int(round(1.0 / (fps * dt))))
    frames = [Image.fromarray(f) for f in render_frames(cell, q_path, every)]
    frames[0].save(path, save_all=True, append_images=frames[1:],
                   duration=int(1000 / fps), loop=0, optimize=True)


def view(cell, q_path: np.ndarray, dt: float) -> None:
    """Replay the simulated motion in MuJoCo's interactive viewer, looping."""
    import time

    import mujoco
    import mujoco.viewer

    with mujoco.viewer.launch_passive(cell.model, cell.data) as viewer:
        while viewer.is_running():
            start = time.perf_counter()
            for i in range(q_path.shape[0]):
                if not viewer.is_running():
                    return
                cell.data.qpos[:] = q_path[i]
                mujoco.mj_kinematics(cell.model, cell.data)
                viewer.sync()
                lag = start + (i + 1) * dt - time.perf_counter()
                if lag > 0:
                    time.sleep(lag)


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--results", default="retract", help="results tree under results/")
    parser.add_argument("--instance", default="test_1")
    parser.add_argument("--run", type=int, default=None, help="run index; default the fastest")
    parser.add_argument("--dt", type=float, default=0.001, help="physics and control step [s]")
    parser.add_argument("--bandwidth", type=float, default=25.0,
                        help="controller natural frequency [rad/s]")
    parser.add_argument("--friction-comp", type=float, default=1.0,
                        help="Coulomb feed-forward, 1 = perfectly identified, 0 = none")
    parser.add_argument("--no-gif", action="store_true")
    parser.add_argument("--view", action="store_true", help="open the interactive viewer")
    args = parser.parse_args(argv)

    record, fallback = pick_record(args.results, args.instance, args.run)
    split = record.get("split", fallback.get("split", "test"))
    instance = {inst.name: inst for inst in build_instances(split)}[args.instance]
    evaluator = evaluator_for(instance, record, fallback)
    outcome = evaluator.evaluate(evaluator.decode(np.asarray(record["chromosome"], dtype=float)))
    if not outcome.feasible:
        raise SystemExit(f"{args.instance} run {record['run']} re-scores infeasible "
                         f"({outcome.rejection}); nothing to simulate")
    print(f"{args.instance} run {record['run']}: {outcome.cycle_time:.3f} s planned, "
          f"posture {outcome.branch_name}, retract {evaluator.retract}")

    arm = evaluator.arm
    cell = build_cell(arm, outcome.machines, outcome.waypoints)
    reference = sample_reference(
        arm, outcome.configurations, evaluator.velocity_limit, evaluator.acceleration_limit,
        evaluator.dwell, outcome.station_index, dt=args.dt,
    )

    target = os.path.join(results_dir("mujoco"), f"{args.results}_{args.instance}")
    os.makedirs(target, exist_ok=True)
    with open(os.path.join(target, "scene.xml"), "w", encoding="utf-8") as handle:
        handle.write(cell.xml)

    # 1. clearance: the solid against the evaluator's bounding spheres.
    clearance = mesh_clearance(cell, reference)
    q4 = arm.from_puma_angles(reference.q[::10])
    sphere_gap = float(evaluator.arm_clearances(q4, outcome.machines).min())
    print(f"  clearance   solid {clearance.worst_gap * 1000:6.1f} mm "
          f"({clearance.worst_part} to {clearance.worst_machine} at {clearance.worst_time:.2f} s); "
          f"evaluator's spheres say {sphere_gap * 1000:.1f} mm")

    # 2. torque.
    demand = required_torques(cell, reference)
    for j in (0, 1, 2, 4):
        print(f"  torque      {JOINT_NAMES[j]:<12} peak {demand.peak[j]:6.1f} N m "
              f"of {demand.limit[j]:5.1f}  ({demand.utilisation[j]:5.0%})")

    # 3. tracking under physics.
    tracking = track_cycle(cell, reference, natural_frequency=args.bandwidth,
                           friction_compensation=args.friction_comp)
    print(f"  tracking    tool tip error max {tracking.tip_error.max() * 1000:.2f} mm, "
          f"at stations max {tracking.station_tip_error.max() * 1000:.2f} mm; "
          f"torque saturated {tracking.saturated_fraction.max():.1%} of steps")
    print(f"  contacts    {len(tracking.collisions)} collision steps, "
          f"{tracking.approach_contacts} gripper-on-served-machine contacts")

    write_plots(target, reference, demand, tracking)
    if not args.no_gif:
        write_gif(cell, tracking.q, reference.dt, os.path.join(target, "cycle.gif"))

    collided = sorted({(part, machine) for _, part, machine, _ in tracking.collisions})
    report = {
        "results_tree": args.results,
        "instance": args.instance,
        "run": record["run"],
        "cycle_time_planned": round(float(outcome.cycle_time), 4),
        "posture": outcome.branch_name,
        "retract": evaluator.retract,
        "clearance": {
            "solid_worst_gap_mm": round(clearance.worst_gap * 1000, 2),
            "solid_worst_part": clearance.worst_part,
            "solid_worst_machine": clearance.worst_machine,
            "solid_worst_time_s": round(clearance.worst_time, 3),
            "sphere_worst_gap_mm": round(sphere_gap * 1000, 2),
            "per_part_mm": {k: round(v * 1000, 2) for k, v in clearance.per_part.items()},
        },
        "torque": {
            JOINT_NAMES[j]: {
                "peak_Nm": round(float(demand.peak[j]), 2),
                "limit_Nm": float(demand.limit[j]),
                "utilisation": round(float(demand.utilisation[j]), 3),
                "at_s": round(float(demand.peak_time[j]), 3),
            }
            for j in range(6)
        },
        "tracking": {
            "bandwidth_rad_s": args.bandwidth,
            "friction_compensation": args.friction_comp,
            "tip_error_max_mm": round(float(tracking.tip_error.max()) * 1000, 3),
            "tip_error_rms_mm": round(float(np.sqrt(np.mean(tracking.tip_error**2))) * 1000, 3),
            "station_tip_error_mm": [round(float(e) * 1000, 3) for e in tracking.station_tip_error],
            "joint_error_max_deg": {JOINT_NAMES[j]: round(float(np.degrees(tracking.joint_error_max[j])), 4)
                                    for j in range(6)},
            "saturated_fraction": {JOINT_NAMES[j]: round(float(tracking.saturated_fraction[j]), 4)
                                   for j in range(6)},
            "collision_steps": len(tracking.collisions),
            "collided_pairs": [list(p) for p in collided],
            "approach_contact_steps": tracking.approach_contacts,
        },
    }
    with open(os.path.join(target, "report.json"), "w", encoding="utf-8") as handle:
        json.dump(report, handle, indent=2)
    write_markdown(target, report)
    print(f"  -> {target}")

    if args.view:
        view(cell, tracking.q, reference.dt)


def write_markdown(target: str, report: Dict) -> None:
    c, tr = report["clearance"], report["tracking"]
    lines = [
        f"# MuJoCo check: `{report['instance']}` run {report['run']} "
        f"from `results/{report['results_tree']}`",
        "",
        f"Planned cycle {report['cycle_time_planned']} s, posture {report['posture']}, "
        f"retract {report['retract']} m.",
        "",
        "## Clearance, solid arm vs machines",
        "",
        f"Worst gap along the cycle: **{c['solid_worst_gap_mm']} mm** "
        f"({c['solid_worst_part']} to {c['solid_worst_machine']}, t = {c['solid_worst_time_s']} s). "
        f"The evaluator's bounding spheres give {c['sphere_worst_gap_mm']} mm on the same samples; "
        "the spheres enclose the solid, so theirs should be the smaller number.",
        "",
        "## Torque the planned motion needs",
        "",
        "Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and "
        "Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified "
        "against a Unimation document**.",
        "",
        "| joint | peak [N m] | limit [N m] | use | at [s] |",
        "|---|---:|---:|---:|---:|",
    ]
    for name, row in report["torque"].items():
        lines.append(f"| {name} | {row['peak_Nm']} | {row['limit_Nm']} | "
                     f"{row['utilisation']:.0%} | {row['at_s']} |")
    lines += [
        "",
        "## Tracking under physics",
        "",
        f"Computed-torque control at {tr['bandwidth_rad_s']} rad/s, Coulomb feed-forward "
        f"x{tr['friction_compensation']}, torques clipped at the limits.",
        "",
        f"- Tool tip error: max **{tr['tip_error_max_mm']} mm**, RMS {tr['tip_error_rms_mm']} mm; "
        f"at the stations max {max(tr['station_tip_error_mm']):.3f} mm.",
        f"- Collision steps outside the approach corridor: **{tr['collision_steps']}**"
        + (f" ({', '.join(f'{p} / {m}' for p, m in tr['collided_pairs'])})"
           if tr["collided_pairs"] else "") + ".",
        f"- Gripper contact with the served machine's top: {tr['approach_contact_steps']} steps "
        "(expected: the tool tip is planned onto the top surface).",
        "",
        "![torques](torques.png)",
        "",
        "![tracking](tracking.png)",
        "",
    ]
    with open(os.path.join(target, "report.md"), "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


if __name__ == "__main__":
    main()
