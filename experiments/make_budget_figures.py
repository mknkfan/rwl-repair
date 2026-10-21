"""Constraint handlers compared at three budgets: generations, evaluations, CPU.

    python -m experiments.make_budget_figures                 # every study below
    python -m experiments.make_budget_figures --study main    # one study

Reads the per-generation traces in each run's raw JSON (``best_so_far``,
``evaluations_trace``, ``cpu_elapsed_trace``) and draws, per study:

* top row: mean best cycle time of each arm against generations, evaluations
  and CPU seconds;
* bottom row: the paired advantage of repair over each other arm (other arm's
  cycle time minus repair's, same instance and seed), with a 95 % confidence
  band.  Positive means repair is ahead.

Reading the CPU panel.  At a CPU budget t, a run's value is its best-so-far at
the last generation that finished within t.  Each arm's line stops at that
arm's median total CPU (the point where half its runs have used up their 300
generations), so no arm is shown past the time it actually ran.  The paired
advantage at CPU t stops at the earlier of the two arms' stopping points.

Outputs, next to each study's results:

    <study>/figures/budget_comparison.png            one study, 2 x 3 panels
    <study>/figures/budget_gain_overview.png         sweeps only: repair's
        advantage over the wall, one line per sweep value
"""

from __future__ import annotations

import argparse
import glob
import json
import os
import sys
from collections import defaultdict
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np
import warnings

warnings.filterwarnings("ignore", category=RuntimeWarning)

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import results_dir  # noqa: E402

import matplotlib  # noqa: E402

matplotlib.use("Agg")
import matplotlib.pyplot as plt  # noqa: E402

# -- style ------------------------------------------------------------------
# Categorical slots 1-4 of the reference palette, in their validated order.
# Penalties are dashed so the two near-identical penalty lines and the wall
# are told apart by more than hue.
ARM_STYLE = {
    "repair":        dict(color="#2a78d6", ls="-",  label="Repair"),
    "baseline":      dict(color="#eb6834", ls="-",  label="Hard reject (+inf)"),
    "penalty_cvd":   dict(color="#1baf7a", ls="--", label="Penalty (CVD)"),
    "penalty_count": dict(color="#eda100", ls=":",  label="Penalty (CVD + count)"),
}
ARM_ORDER = ("repair", "baseline", "penalty_cvd", "penalty_count")
INK, INK2, MUTED = "#0b0b0b", "#52514e", "#898781"
GRID, AXIS = "#e1e0d9", "#c3c2b7"
# Ordinal blue ramp for sweep values, steps 250 -> 650 (lightest still 2:1).
RAMP = ["#86b6ef", "#5598e7", "#2a78d6", "#1c5cab", "#104281"]

plt.rcParams.update({
    "font.family": "sans-serif",
    "font.size": 8.5,
    "axes.edgecolor": AXIS,
    "axes.labelcolor": INK2,
    "axes.titlesize": 9,
    "axes.titleweight": "bold",
    "axes.titlecolor": INK,
    "xtick.color": MUTED,
    "ytick.color": MUTED,
    "axes.grid": True,
    "grid.color": GRID,
    "grid.linewidth": 0.6,
    "axes.spines.top": False,
    "axes.spines.right": False,
    "legend.frameon": False,
    "savefig.dpi": 220,
    "savefig.bbox": "tight",
})

CPU_POINTS = 400
Z95 = 1.96


# -- loading ----------------------------------------------------------------

def load_study(root: str) -> Dict[str, Dict[Tuple[str, int], dict]]:
    """arm -> (instance, run) -> trace arrays."""
    by_arm: Dict[str, Dict[Tuple[str, int], dict]] = defaultdict(dict)
    for path in glob.glob(os.path.join(root, "raw", "*.json")):
        with open(path) as fh:
            rec = json.load(fh)
        by_arm[rec["arm"]][(rec["instance"], rec["run"])] = {
            "best": np.asarray([np.nan if v is None else v for v in rec["best_so_far"]], float),
            "evals": np.asarray(rec["evaluations_trace"], float),
            "cpu": np.asarray(rec["cpu_elapsed_trace"], float),
        }
    if not by_arm:
        raise SystemExit(f"no raw runs under {root}")
    keys = set.intersection(*(set(v) for v in by_arm.values()))
    return {arm: {k: runs[k] for k in sorted(keys)} for arm, runs in by_arm.items()}


def best_at_cpu(run: dict, grid: np.ndarray) -> np.ndarray:
    """Best-so-far at the last generation finished within each budget."""
    idx = np.searchsorted(run["cpu"], grid, side="right") - 1
    out = np.full(grid.shape, np.nan)
    ok = idx >= 0
    out[ok] = run["best"][idx[ok]]
    return out


def stop_time(runs: Dict) -> float:
    return float(np.median([r["cpu"][-1] for r in runs.values()]))


def mean_ci(diff: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """Mean and 95 % half-width over rows (pairs), ignoring NaN."""
    n = np.sum(~np.isnan(diff), axis=0)
    m = np.nanmean(diff, axis=0)
    sd = np.nanstd(diff, axis=0, ddof=1)
    return m, Z95 * sd / np.sqrt(np.maximum(n, 1))


# -- one study, 2 x 3 -------------------------------------------------------

def budget_figure(study: Dict, title: str, out_base: str, note: Optional[str] = None) -> None:
    arms = [a for a in ARM_ORDER if a in study]
    keys = list(study["repair"].keys())
    n_pairs = len(keys)
    gens = np.arange(1, len(study["repair"][keys[0]]["best"]) + 1)
    evals = np.mean([r["evals"] for r in study["repair"].values()], axis=0)
    stops = {a: stop_time(study[a]) for a in arms}
    cpu_grid = np.linspace(0.0, max(stops.values()), CPU_POINTS)

    best_gen = {a: np.stack([study[a][k]["best"] for k in keys]) for a in arms}
    best_cpu = {a: np.stack([best_at_cpu(study[a][k], cpu_grid) for k in keys]) for a in arms}

    fig, axes = plt.subplots(2, 3, figsize=(10.5, 5.6), sharex="col")
    xs = [(gens, "Generation"), (evals / 1000.0, "Evaluations (thousands)"), (cpu_grid, "CPU time per run [s]")]

    # y window: from a bit below the best final mean to the wall's mean after 5 % of the run
    finals = [np.nanmean(best_gen[a][:, -1]) for a in arms]
    top = np.nanmean(best_gen["baseline"][:, max(1, len(gens) // 20)])
    ylo, yhi = min(finals) - 0.08, top

    for col, (x, xlabel) in enumerate(xs):
        ax, axg = axes[0, col], axes[1, col]
        for a in arms:
            st = ARM_STYLE[a]
            if col < 2:
                y, xx = np.nanmean(best_gen[a], axis=0), x
            else:
                keep = cpu_grid <= stops[a]
                y, xx = np.nanmean(best_cpu[a][:, keep], axis=0), cpu_grid[keep]
            ax.plot(xx, y, color=st["color"], ls=st["ls"], lw=1.8, label=st["label"])
            if col == 2:
                ax.plot(xx[-1], y[-1], "o", ms=4, color=st["color"], mec="white", mew=1)
        ax.set_ylim(ylo, yhi)

        for a in arms:
            if a == "repair":
                continue
            st = ARM_STYLE[a]
            if col < 2:
                diff, xx = best_gen[a] - best_gen["repair"], x
            else:
                keep = cpu_grid <= min(stops[a], stops["repair"])
                diff, xx = best_cpu[a][:, keep] - best_cpu["repair"][:, keep], cpu_grid[keep]
            m, h = mean_ci(diff)
            axg.fill_between(xx, m - h, m + h, color=st["color"], alpha=0.15, lw=0)
            axg.plot(xx, m, color=st["color"], ls=st["ls"], lw=1.8, label=f"vs {st['label']}")
        axg.axhline(0, color=AXIS, lw=1)
        axg.set_xlabel(xlabel)

    axes[0, 0].set_title("Equal generations", loc="left")
    axes[0, 1].set_title("Equal evaluations", loc="left")
    axes[0, 2].set_title("Equal CPU time", loc="left")
    axes[0, 0].set_ylabel("Mean best cycle time [s]")
    axes[1, 0].set_ylabel("Repair ahead by [s]\n(other arm − repair, paired)")
    for ax in axes[0, 1:]:
        ax.tick_params(labelleft=True)

    axes[0, 0].legend(loc="upper right", fontsize=8)
    axes[1, 0].legend(loc="upper right", fontsize=8)
    sub = f"{n_pairs} paired runs (instance × seed); bands are 95 % CIs of the paired mean."
    sub += " CPU lines stop at each arm's median total CPU."
    if note:
        sub += "\n" + note
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.text(0.01, 0.935, sub, ha="left", va="top", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0, 1, 0.9 if note else 0.92))
    save(fig, out_base)


# -- sweeps: repair's advantage over the wall, one line per value ------------

def gain_overview(groups: Sequence[Tuple[str, List[Tuple[str, Dict, bool]]]], title: str,
                  out_base: str, note: Optional[str] = None) -> None:
    """groups: (row label, [(value label, study, is_default)])."""
    fig, axes = plt.subplots(len(groups), 3, figsize=(12.0, 2.5 * len(groups) + 0.9),
                             sharex="col", squeeze=False)
    for row, (row_label, cells) in enumerate(groups):
        ramp = RAMP if len(cells) >= 4 else RAMP[1::2] if len(cells) == 3 else RAMP[1:4:2]
        for (label, study, is_default), colour in zip(cells, ramp[-len(cells):] if len(cells) <= len(ramp) else RAMP):
            keys = list(study["repair"].keys())
            wall = np.stack([study["baseline"][k]["best"] for k in keys])
            rep = np.stack([study["repair"][k]["best"] for k in keys])
            gens = np.arange(1, wall.shape[1] + 1)
            evals = np.mean([r["evals"] for r in study["repair"].values()], axis=0) / 1000.0
            stop = min(stop_time(study["baseline"]), stop_time(study["repair"]))
            grid = np.linspace(0, stop, CPU_POINTS)
            wall_c = np.stack([best_at_cpu(study["baseline"][k], grid) for k in keys])
            rep_c = np.stack([best_at_cpu(study["repair"][k], grid) for k in keys])
            lw = 2.4 if is_default else 1.5
            name = f"{label} (default)" if is_default else label
            for col, (x, d) in enumerate([(gens, wall - rep), (evals, wall - rep), (grid, wall_c - rep_c)]):
                m, h = mean_ci(d)
                ax = axes[row, col]
                if is_default:
                    ax.fill_between(x, m - h, m + h, color=colour, alpha=0.15, lw=0)
                ax.plot(x, m, color=colour, lw=lw, label=name)
        for col in range(3):
            axes[row, col].axhline(0, color=AXIS, lw=1)
        axes[row, 0].set_ylabel("Repair ahead of wall [s]")
        axes[row, 2].legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), fontsize=7.5,
                            title=row_label, title_fontsize=7.5, alignment="left")
    axes[0, 0].set_title("Equal generations", loc="left")
    axes[0, 1].set_title("Equal evaluations", loc="left")
    axes[0, 2].set_title("Equal CPU time", loc="left")
    axes[-1, 0].set_xlabel("Generation")
    axes[-1, 1].set_xlabel("Evaluations (thousands)")
    axes[-1, 2].set_xlabel("CPU time per run [s]")
    sub = ("Paired advantage of repair over the hard-reject wall (wall − repair; positive = repair ahead). "
           "Band: 95 % CI, default only. CPU lines stop at the earlier arm's median total CPU.")
    if note:
        sub += "\n" + note
    fig.suptitle(title, x=0.01, ha="left", fontsize=11, fontweight="bold", color=INK)
    fig.text(0.01, 1 - 0.45 / fig.get_figheight(), sub, ha="left", va="top", fontsize=8, color=INK2)
    fig.tight_layout(rect=(0, 0, 1, 1 - (0.85 if note else 0.7) / fig.get_figheight()))
    save(fig, out_base)


def save(fig, out_base: str) -> None:
    os.makedirs(os.path.dirname(out_base), exist_ok=True)
    for ext in ("png",):
        fig.savefig(f"{out_base}.{ext}")
    plt.close(fig)
    print("wrote", out_base + ".png")


# -- studies ----------------------------------------------------------------

def run_main() -> None:
    root = results_dir("cx08_mut02_r20_g300")
    budget_figure(load_study(root), "Constraint handlers at three budgets (6 machines)",
                  os.path.join(root, "figures", "budget_comparison"))


def run_machines() -> None:
    base = results_dir("cx08_mut02_r20_g300_machines")
    note = ("CPU panels use the ORIGINAL (unoptimised) repair code on Windows: repair's CPU cost is overstated; "
            "the generation and evaluation panels are unaffected.")
    cells = []
    for m in sorted(glob.glob(os.path.join(base, "m*"))):
        name = os.path.basename(m)
        study = load_study(m)
        budget_figure(study, f"Constraint handlers at three budgets ({name[1:]} machines)",
                      os.path.join(m, "figures", "budget_comparison"), note=note)
        cells.append((f"{name[1:]} machines", study, name == "m6"))
    gain_overview([("Machines", cells)], "Repair vs wall by machine count",
                  os.path.join(base, "figures", "budget_gain_overview"), note=note)


SWEEPS = (
    ("sigma_min [m/rad]", "sigma_min", ("0", "0.07", "0.14", "0.21"), "0.14"),
    ("arm clearance [m]", "arm_clearance", ("0", "0.005", "0.01", "0.02"), "0.005"),
    ("retract [m]", "retract", ("0.03", "0.06", "0.1"), "0.06"),
)


def run_sensitivity() -> None:
    base = results_dir("sensitivity_repair")
    loaded: Dict[str, Dict] = {}
    for cell in sorted(os.listdir(base)):
        if os.path.isdir(os.path.join(base, cell, "raw")):
            loaded[cell] = load_study(os.path.join(base, cell))
            budget_figure(loaded[cell], f"Repair vs hard reject at three budgets ({cell})",
                          os.path.join(base, cell, "figures", "budget_comparison"))
    groups = []
    for row_label, param, values, default in SWEEPS:
        cells = []
        for v in values:
            cell = "default" if v == default else f"{param}={v}"
            cells.append((f"{param} = {v}", loaded[cell], v == default))
        groups.append((row_label, cells))
    gain_overview(groups, "Sensitivity of the repair gain to the feasibility thresholds",
                  os.path.join(base, "figures", "budget_gain_overview"),
                  note="retract = 0.03 m: every best layout fails the dense (100-sample) clearance re-check; "
                       "read that line with caution.")


STUDIES = {"main": run_main, "machines": run_machines, "sensitivity": run_sensitivity}


def main(argv: Optional[Sequence[str]] = None) -> None:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--study", choices=sorted(STUDIES), action="append",
                   help="which study to draw (repeatable; default all)")
    args = p.parse_args(argv)
    for name in args.study or STUDIES:
        STUDIES[name]()


if __name__ == "__main__":
    main()
