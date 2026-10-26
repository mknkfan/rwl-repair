"""One table across machine counts for the four-arm penalty A/B sweep.

    python -m experiments.summarize_penalty_machines --sweep-dir results/cx08_mut02_r20_g300_machines

``run_penalty_ab --machines N`` writes one study per machine count, each with
its own report and ``summary.json``.  The question the sweep asks -- does the
ranking of repair, the ``+inf`` wall and the two penalties change as the cell
fills up? -- is across those studies, so this reads every ``m<N>/summary.json``
under the sweep folder and puts them side by side.

Each table is one family for Holm: every (machine count, arm) comparison in it
is corrected together, on the permutation p-values.

Cycle times are **not** comparable between rows -- more machines is more
stations per cycle -- so the comparison that means something is the paired
gain within a row, and how it moves from row to row.

Writes ``summary.md``, ``summary.csv`` and ``penalty_by_machines.png`` into the
sweep folder.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys
from typing import Dict, List, Optional, Sequence

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from summarize_repair_machines import _f, _p, holm  # noqa: E402

ARMS = ("baseline", "repair", "penalty_cvd", "penalty_count")

#: (key, title, reference, arms compared, where in summary.json)
FAMILIES = (
    ("vs_repair_gen", "Against `repair`, equal generations", "repair",
     ("baseline", "penalty_cvd", "penalty_count"), ("equal_generations", "cycle_time")),
    ("vs_baseline_gen", "Penalty against the `+inf` wall, equal generations", "baseline",
     ("penalty_cvd", "penalty_count"), ("versus_baseline", "equal_generations")),
    ("vs_repair_cpu", "Against `repair`, equal CPU", "repair",
     ("baseline", "penalty_cvd", "penalty_count"), ("equal_cpu",)),
    ("vs_baseline_cpu", "Penalty against the `+inf` wall, equal CPU", "baseline",
     ("penalty_cvd", "penalty_count"), ("versus_baseline", "equal_cpu")),
)

FIELDS = ("family", "n_machines", "arm", "reference", "n_pairs", "arm_mean",
          "reference_mean", "mean_gain", "ci_low", "ci_high", "arm_wins",
          "reference_wins", "perm_p", "holm_p", "wilcoxon_p")


def load_studies(sweep_dir: str) -> Dict[int, Dict]:
    studies = {}
    for name in sorted(os.listdir(sweep_dir)):
        match = re.fullmatch(r"m(\d+)", name)
        path = os.path.join(sweep_dir, name, "summary.json")
        if match and os.path.isfile(path):
            with open(path, encoding="utf-8") as handle:
                studies[int(match.group(1))] = json.load(handle)
    return dict(sorted(studies.items()))


def family_rows(studies: Dict[int, Dict]) -> Dict[str, List[Dict]]:
    out = {}
    for key, _title, reference, arms, where in FAMILIES:
        rows = []
        for n, summary in studies.items():
            block = summary
            for step in where:
                block = block[step]
            for arm in arms:
                s = block[arm]
                rows.append({
                    "family": key, "n_machines": n, "arm": arm, "reference": reference,
                    "n_pairs": s["n_pairs"], "arm_mean": s["arm_mean"],
                    "reference_mean": s["reference_mean"], "mean_gain": s["mean_gain"],
                    "ci_low": s["ci_low"], "ci_high": s["ci_high"],
                    "arm_wins": s["arm_wins"], "reference_wins": s["reference_wins"],
                    "perm_p": s["perm_p"], "wilcoxon_p": s["wilcoxon_p"],
                })
        for row, adjusted in zip(rows, holm([r["perm_p"] for r in rows])):
            row["holm_p"] = adjusted
        out[key] = rows
    return out


def arm_means(studies: Dict[int, Dict]) -> Dict[int, Dict[str, float]]:
    """Mean cycle time per arm, at equal generations, from the vs-repair block."""
    means = {}
    for n, summary in studies.items():
        block = summary["equal_generations"]["cycle_time"]
        row = {arm: block[arm]["arm_mean"] for arm in ARMS if arm != "repair"}
        row["repair"] = next(iter(block.values()))["reference_mean"]
        means[n] = row
    return means


def write_markdown(path: str, studies: Dict[int, Dict], families: Dict[str, List[Dict]]) -> None:
    pairs = {n: s["equal_generations"]["cycle_time"]["baseline"]["n_pairs"]
             for n, s in studies.items()}
    lines = [
        "# Repair, the wall, and two penalties, by machine count",
        "",
        "One `run_penalty_ab` study per machine count (pairs per study: "
        + ", ".join(f"{n} machines = {p}" for n, p in pairs.items())
        + "). All four arms start from the identical population per (instance, seed). "
        "**Gain** is reference minus arm cycle time, so positive means the arm was "
        "faster than the reference. Holm corrects the permutation p-values within each "
        "table (all rows of the table are one family).",
        "",
        "Cycle times are not comparable between rows (more machines, more stations); "
        "the gain within a row is.",
        "",
        "## Mean cycle time (s), equal generations",
        "",
        "| machines | " + " | ".join(f"`{a}`" for a in ARMS) + " |",
        "|---:|" + "---:|" * len(ARMS),
    ]
    for n, row in arm_means(studies).items():
        best = min(row.values())
        cells = [(f"**{row[a]:.3f}**" if row[a] == best else f"{row[a]:.3f}") for a in ARMS]
        lines.append(f"| {n} | " + " | ".join(cells) + " |")

    for key, title, reference, _arms, _where in FAMILIES:
        lines += [
            "",
            f"## {title}",
            "",
            f"| machines | arm | arm [s] | `{reference}` [s] | gain [s] | 95% CI "
            f"| arm wins | p (Holm) |",
            "|---:|---|---:|---:|---:|---|---:|---:|",
        ]
        for r in families[key]:
            lines.append(
                f"| {r['n_machines']} | `{r['arm']}` | {_f(r['arm_mean'], '.3f')} | "
                f"{_f(r['reference_mean'], '.3f')} | {_f(r['mean_gain'], '+.3f')} | "
                f"[{_f(r['ci_low'], '+.3f')}, {_f(r['ci_high'], '+.3f')}] | "
                f"{r['arm_wins']}/{r['n_pairs']} | {_p(r['holm_p'])} |"
            )
    lines += ["", "![penalty by machine count](penalty_by_machines.png)", ""]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def write_figure(path: str, families: Dict[str, List[Dict]]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(1, 2, figsize=(11, 4.2), sharey=True)
    for ax, key, title in ((axes[0], "vs_repair_gen", "Equal generations"),
                           (axes[1], "vs_repair_cpu", "Equal CPU")):
        rows = families[key]
        ax.axhline(0.0, color="0.5", lw=0.8)
        arms = sorted({r["arm"] for r in rows}, key=ARMS.index)
        for offset, (arm, marker) in zip((-0.12, 0.0, 0.12), zip(arms, "o^s")):
            sel = [r for r in rows if r["arm"] == arm]
            n = np.array([r["n_machines"] for r in sel], dtype=float)
            gain, low, high = (np.array([np.nan if r[k] is None else r[k] for r in sel])
                               for k in ("mean_gain", "ci_low", "ci_high"))
            ax.errorbar(n + offset, gain, yerr=[gain - low, high - gain], fmt=f"{marker}-",
                        capsize=3, label=arm)
        ax.set_xticks(sorted({r["n_machines"] for r in rows}))
        ax.set_xlabel("machines per cell")
        ax.set_title(f"{title}: gain over repair (95% CI)")
        ax.grid(alpha=0.3)
    axes[0].set_ylabel("cycle-time gain over repair [s]\n(repair - arm, paired; <0 = repair better)")
    axes[0].legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--sweep-dir", required=True)
    args = parser.parse_args(argv)

    studies = load_studies(args.sweep_dir)
    if not studies:
        raise SystemExit(f"no m<N>/summary.json studies under {args.sweep_dir}")
    families = family_rows(studies)
    with open(os.path.join(args.sweep_dir, "summary.csv"), "w", newline="", encoding="utf-8") as h:
        writer = csv.DictWriter(h, fieldnames=FIELDS)
        writer.writeheader()
        for rows in families.values():
            writer.writerows(rows)
    write_markdown(os.path.join(args.sweep_dir, "summary.md"), studies, families)
    write_figure(os.path.join(args.sweep_dir, "penalty_by_machines.png"), families)
    print(f"  {len(studies)} machine counts: {', '.join(map(str, studies))}")
    print(f"  -> {os.path.join(args.sweep_dir, 'summary.md')}")


if __name__ == "__main__":
    main()
