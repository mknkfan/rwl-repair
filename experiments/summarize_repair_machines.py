"""One table across machine counts for the repair A/B sweep.

    python -m experiments.summarize_repair_machines
    python -m experiments.summarize_repair_machines --sweep-dir results/repair_retract_machines

``run_repair_ab --machines N`` writes one paired study per machine count, each
with its own report.  The question the sweep asks -- does the repair's effect
change as the cell fills up? -- is across those studies, so this reads every
``m<N>/raw/`` under the sweep folder and puts them side by side.

Pairs are rebuilt with :func:`run_repair_ab.pair_up`, so the check that both
arms of a pair started from the same population (matching SHA-256
fingerprints) applies here too.  The cycle-time p-values are Holm-corrected
across the machine counts: five tests at once, one family.

Cycle times are **not** comparable between rows -- more machines is more
stations per cycle -- so the comparison that means something is the paired
gain within a row, and how it moves from row to row.

Writes ``summary.md``, ``summary.csv`` and ``repair_by_machines.png`` into the
sweep folder.
"""

from __future__ import annotations

import argparse
import csv
import os
import re
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from run_repair_ab import collect_records, compare, feasibility_table, pair_up  # noqa: E402

DEFAULT_SWEEP_DIR = results_dir("repair_retract_machines")

FIELDS = (
    "n_machines", "n_pairs", "both_feasible", "only_baseline", "only_repair", "neither",
    "mcnemar_p", "baseline_cycle_mean", "repair_cycle_mean", "mean_gain_s", "median_gain_s",
    "gain_ci_low", "gain_ci_high", "repair_wins", "baseline_wins", "ties",
    "permutation_p", "holm_p", "wilcoxon_p", "baseline_mean_feasible", "repair_mean_feasible",
    "baseline_diversity", "repair_diversity", "baseline_cpu_s", "repair_cpu_s",
)


def holm(p_values: Sequence[Optional[float]]) -> List[Optional[float]]:
    """Holm-Bonferroni adjusted p-values; ``None`` entries are skipped."""
    present = [(p, i) for i, p in enumerate(p_values) if p is not None]
    present.sort()
    adjusted: List[Optional[float]] = [None] * len(p_values)
    running = 0.0
    m = len(present)
    for rank, (p, i) in enumerate(present):
        running = max(running, min(1.0, (m - rank) * p))
        adjusted[i] = running
    return adjusted


def study_rows(sweep_dir: str) -> List[Dict]:
    rows = []
    for name in sorted(os.listdir(sweep_dir)):
        match = re.fullmatch(r"m(\d+)", name)
        if not match or not os.path.isdir(os.path.join(sweep_dir, name, "raw")):
            continue
        records = collect_records(os.path.join(sweep_dir, name))
        if not records:
            continue
        pairs = pair_up(records)
        cycle = compare(pairs, "cycle_time")
        feas = compare(pairs, "mean_feasible", lower_is_better=False)
        diversity = compare(pairs, "mean_diversity", lower_is_better=False)
        cpu = compare(pairs, "cpu_time")
        table = feasibility_table(pairs)
        rows.append({
            "n_machines": int(match.group(1)),
            "n_pairs": len(pairs),
            "both_feasible": table["both"],
            "only_baseline": table["only_baseline"],
            "only_repair": table["only_repair"],
            "neither": table["neither"],
            "mcnemar_p": table["mcnemar_p"],
            "baseline_cycle_mean": cycle.get("baseline_mean"),
            "repair_cycle_mean": cycle.get("repair_mean"),
            "mean_gain_s": cycle.get("mean_gain"),
            "median_gain_s": cycle.get("median_gain"),
            "gain_ci_low": cycle.get("ci_low"),
            "gain_ci_high": cycle.get("ci_high"),
            "repair_wins": cycle.get("repair_wins"),
            "baseline_wins": cycle.get("baseline_wins"),
            "ties": cycle.get("ties"),
            "permutation_p": cycle.get("permutation_p"),
            "wilcoxon_p": cycle.get("wilcoxon_p"),
            "baseline_mean_feasible": feas.get("baseline_mean"),
            "repair_mean_feasible": feas.get("repair_mean"),
            "baseline_diversity": diversity.get("baseline_mean"),
            "repair_diversity": diversity.get("repair_mean"),
            "baseline_cpu_s": cpu.get("baseline_mean"),
            "repair_cpu_s": cpu.get("repair_mean"),
        })
    rows.sort(key=lambda r: r["n_machines"])
    for row, adjusted in zip(rows, holm([r["permutation_p"] for r in rows])):
        row["holm_p"] = adjusted
    return rows


def _f(value, spec: str) -> str:
    return "-" if value is None else format(value, spec)


def _p(value: Optional[float]) -> str:
    """A p-value, without printing a sampled one as an exact zero."""
    if value is None:
        return "-"
    return "< 0.0001" if value < 1e-4 else f"{value:.4f}"


def write_markdown(path: str, rows: Sequence[Dict]) -> None:
    lines = [
        "# Placement repair vs none, by machine count (retract on)",
        "",
        "One paired study per row: 10 test instances x 10 seeds, both arms started "
        "from the identical population (fingerprint-checked). **Gain** is baseline "
        "minus repair cycle time over pairs where both found a layout, so positive "
        "means repair was faster. Holm corrects the permutation p-values across the rows.",
        "",
        "Cycle times are not comparable between rows (more machines, more stations); "
        "the gain within a row is.",
        "",
        "| machines | pairs | both feasible | only base / only repair | baseline [s] | repair [s] "
        "| gain [s] | 95% CI | repair wins | p (Holm) |",
        "|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|",
    ]
    for r in rows:
        lines.append(
            f"| {r['n_machines']} | {r['n_pairs']} | {r['both_feasible']} | "
            f"{r['only_baseline']} / {r['only_repair']} | {_f(r['baseline_cycle_mean'], '.3f')} | "
            f"{_f(r['repair_cycle_mean'], '.3f')} | {_f(r['mean_gain_s'], '+.3f')} | "
            f"[{_f(r['gain_ci_low'], '+.3f')}, {_f(r['gain_ci_high'], '+.3f')}] | "
            f"{_f(r['repair_wins'], 'd')}/{(r['repair_wins'] or 0) + (r['baseline_wins'] or 0) + (r['ties'] or 0)} | "
            f"{_p(r['holm_p'])} |"
        )
    lines += [
        "",
        "| machines | population feasible share, base -> repair | diversity, base -> repair "
        "| CPU per run [s], base -> repair |",
        "|---:|---|---|---|",
    ]
    for r in rows:
        lines.append(
            f"| {r['n_machines']} | {_f(r['baseline_mean_feasible'], '.3f')} -> "
            f"{_f(r['repair_mean_feasible'], '.3f')} | {_f(r['baseline_diversity'], '.2f')} -> "
            f"{_f(r['repair_diversity'], '.2f')} | {_f(r['baseline_cpu_s'], '.0f')} -> "
            f"{_f(r['repair_cpu_s'], '.0f')} |"
        )
    lines += ["", "![repair by machine count](repair_by_machines.png)", ""]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def write_figure(path: str, rows: Sequence[Dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    n = np.array([r["n_machines"] for r in rows])
    fig, axes = plt.subplots(1, 2, figsize=(10, 4))

    gain = np.array([np.nan if r["mean_gain_s"] is None else r["mean_gain_s"] for r in rows])
    low = np.array([np.nan if r["gain_ci_low"] is None else r["gain_ci_low"] for r in rows])
    high = np.array([np.nan if r["gain_ci_high"] is None else r["gain_ci_high"] for r in rows])
    axes[0].axhline(0.0, color="0.5", lw=0.8)
    axes[0].errorbar(n, gain, yerr=[gain - low, high - gain], fmt="o-", capsize=4)
    axes[0].set_xlabel("machines per cell")
    axes[0].set_ylabel("cycle-time gain from repair [s]\n(baseline - repair, paired)")
    axes[0].set_title("Repair's effect on cycle time (95% CI)")
    axes[0].set_xticks(n)
    axes[0].grid(alpha=0.3)

    # What repair acts on: how much of the population is feasible, averaged
    # over the run.  (Whether a run found a layout at all is in the table --
    # it saturates at every count here, so it makes a flat, empty chart.)
    base = [r["baseline_mean_feasible"] for r in rows]
    rep = [r["repair_mean_feasible"] for r in rows]
    axes[1].plot(n, base, "o-", label="no repair")
    axes[1].plot(n, rep, "s-", label="repair")
    axes[1].set_ylim(0, 1)
    axes[1].set_xlabel("machines per cell")
    axes[1].set_ylabel("feasible share of the population\n(mean over the run)")
    axes[1].set_title("What repair acts on")
    axes[1].set_xticks(n)
    axes[1].legend(loc="lower left")
    axes[1].grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main(argv: Optional[List[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--sweep-dir", default=DEFAULT_SWEEP_DIR)
    args = parser.parse_args(argv)

    rows = study_rows(args.sweep_dir)
    if not rows:
        raise SystemExit(f"no m<N>/raw/ studies under {args.sweep_dir}")
    with open(os.path.join(args.sweep_dir, "summary.csv"), "w", newline="", encoding="utf-8") as h:
        writer = csv.DictWriter(h, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    write_markdown(os.path.join(args.sweep_dir, "summary.md"), rows)
    write_figure(os.path.join(args.sweep_dir, "repair_by_machines.png"), rows)
    for r in rows:
        print(f"  m{r['n_machines']}: {r['n_pairs']} pairs, gain {_f(r['mean_gain_s'], '+.3f')} s "
              f"[{_f(r['gain_ci_low'], '+.3f')}, {_f(r['gain_ci_high'], '+.3f')}], "
              f"Holm p {_p(r['holm_p'])}")
    print(f"  -> {os.path.join(args.sweep_dir, 'summary.md')}")


if __name__ == "__main__":
    main()
