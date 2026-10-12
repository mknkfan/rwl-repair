"""Population size x generations for the four-arm penalty study, from traces.

    python -m experiments.summarize_popgen --sweep-dir results/popgen

The sweep is one ``run_penalty_ab`` study per population size, each run to the
largest generation count of interest::

    for P in 400 200 100 50; do
        python -m experiments.run_penalty_ab --only run --population $P \\
            --generations 500 --runs 10 --crossover-rate 0.8 --mutation-rate 0.2 \\
            --output results/popgen/p$P
    done

Generations need no runs of their own.  Nothing in ``ConfigurableGA`` depends on
the generation count it was configured with -- no annealing, and the
evaluation-budget stop never fires before the last generation -- so a run read
at generation ``g`` of its ``best_so_far`` trace *is* the ``g``-generation run,
same RNG stream and all.  Every cut below is such a read.

Three budgets, because they answer different questions:

* **equal generations** -- what the GA literature reports, and what a setting
  in a config file means;
* **equal evaluations** -- ``P * G`` held fixed, which is the honest way to
  choose between a big population and a long run;
* **equal CPU** -- as equal evaluations, but charging repair for its own cost
  and the wall for its cheap rejections.

Pairing is by (instance, seed) throughout.  Different population sizes start
from different populations, so between-``P`` differences are paired on the
instance and seed only; that still removes the instance spread, which is most
of the variance.

Writes ``summary.md``, ``grid.csv`` and ``popgen.png`` into the sweep folder.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import re
import sys
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()

from run_repair_ab import bootstrap_ci, sign_flip_test  # noqa: E402
from summarize_repair_machines import _f, _p, holm  # noqa: E402

ARMS = ("baseline", "repair", "penalty_cvd", "penalty_count")
GENERATIONS = (25, 50, 100, 150, 200, 300, 400, 500)
EVAL_BUDGETS = (5_000, 10_000, 20_000, 40_000, 80_000)
CPU_BUDGETS = (5.0, 10.0, 20.0, 40.0, 80.0)
#: Within this many seconds of the best cell counts as "as good as the best".
TOLERANCE = 0.02

Key = Tuple[str, int]


def load(sweep_dir: str) -> Dict[int, Dict[Key, Dict[str, Dict]]]:
    """``P -> (instance, run) -> arm -> record``, complete sets only."""
    out: Dict[int, Dict[Key, Dict[str, Dict]]] = {}
    for name in sorted(os.listdir(sweep_dir)):
        match = re.fullmatch(r"p(\d+)", name)
        raw = os.path.join(sweep_dir, name, "raw")
        if not match or not os.path.isdir(raw):
            continue
        grouped: Dict[Key, Dict[str, Dict]] = {}
        for file in os.listdir(raw):
            if file.endswith(".json"):
                with open(os.path.join(raw, file), encoding="utf-8") as handle:
                    r = json.load(handle)
                grouped.setdefault((r["instance"], r["run"]), {})[r["arm"]] = r
        complete = {k: v for k, v in grouped.items() if len(v) == len(ARMS)}
        if complete:
            out[int(match.group(1))] = complete
    return dict(sorted(out.items()))


def cut_at(record: Dict, index: int) -> Dict:
    """The run as it stood at the end of generation ``index`` (0-based)."""
    if index < 0:
        return {"cycle_time": None, "cpu": 0.0, "evals": 0, "generations": 0}
    return {
        "cycle_time": record["best_so_far"][index],
        "cpu": float(record["cpu_elapsed_trace"][index]),
        "evals": int(record["evaluations_trace"][index]),
        "generations": index + 1,
    }


def cut_generations(record: Dict, g: int) -> Optional[Dict]:
    return cut_at(record, g - 1) if g <= len(record["best_so_far"]) else None


def _cut_trace(record: Dict, trace: str, budget: float) -> Optional[Dict]:
    """Last generation inside ``budget``; ``None`` if the run never reached it."""
    values = np.asarray(record[trace], dtype=float)
    if values[-1] < budget:
        return None
    return cut_at(record, int(np.flatnonzero(values <= budget)[-1]) if values[0] <= budget else -1)


def cut_evals(record: Dict, budget: float) -> Optional[Dict]:
    return _cut_trace(record, "evaluations_trace", budget)


def cut_cpu(record: Dict, budget: float) -> Optional[Dict]:
    return _cut_trace(record, "cpu_elapsed_trace", budget)


# ----------------------------------------------------------------------
# tables
# ----------------------------------------------------------------------


def cell(study: Dict[Key, Dict[str, Dict]], arm: str, cutter, budget) -> Optional[Dict]:
    """Mean outcome of one arm at one budget over the sets of one study."""
    cuts = [cutter(arms[arm], budget) for arms in study.values()]
    if any(c is None for c in cuts):
        return None
    times = [c["cycle_time"] for c in cuts]
    found = [t for t in times if t is not None]
    return {
        "n": len(cuts),
        "feasible": len(found) / len(cuts),
        "cycle_time": float(np.mean(found)) if found else None,
        "cpu": float(np.mean([c["cpu"] for c in cuts])),
        "evals": float(np.mean([c["evals"] for c in cuts])),
        "generations": float(np.mean([c["generations"] for c in cuts])),
        "per_key": {k: c["cycle_time"] for k, c in zip(study, cuts)},
    }


def build_grid(data: Dict[int, Dict]) -> List[Dict]:
    rows = []
    for budget_kind, cutter, budgets in (
        ("generations", cut_generations, GENERATIONS),
        ("evaluations", cut_evals, EVAL_BUDGETS),
        ("cpu", cut_cpu, CPU_BUDGETS),
    ):
        for P, study in data.items():
            for b in budgets:
                for arm in ARMS:
                    c = cell(study, arm, cutter, b)
                    if c is not None:
                        rows.append({"budget_kind": budget_kind, "budget": b,
                                     "population": P, "arm": arm, **c})
    return rows


def paired(a: Dict[Key, Optional[float]], b: Dict[Key, Optional[float]]) -> Optional[Dict]:
    """``a`` against ``b`` over shared keys; positive gain means ``a`` faster."""
    keys = [k for k in a if k in b and a[k] is not None and b[k] is not None]
    if not keys:
        return None
    d = np.array([b[k] - a[k] for k in keys])
    low, high = bootstrap_ci(d)
    return {"n": len(keys), "gain": float(d.mean()), "ci_low": low, "ci_high": high,
            "wins": int((d > 0).sum()), "p": sign_flip_test(d)}


def lookup(grid: Sequence[Dict], kind: str, budget, P: int, arm: str) -> Optional[Dict]:
    for r in grid:
        if (r["budget_kind"], r["budget"], r["population"], r["arm"]) == (kind, budget, P, arm):
            return r
    return None


# ----------------------------------------------------------------------
# report
# ----------------------------------------------------------------------


def _table(grid, kind, budgets, populations, arm, value="cycle_time", spec=".3f",
           bold_best=True) -> List[str]:
    label = {"generations": "G", "evaluations": "evals", "cpu": "CPU s"}[kind]
    lines = [f"| P \\ {label} | " + " | ".join(f"{b:g}" for b in budgets) + " |",
             "|---:|" + "---:|" * len(budgets)]
    vals = {(P, b): (lookup(grid, kind, b, P, arm) or {}).get(value)
            for P in populations for b in budgets}
    finite = [v for v in vals.values() if v is not None]
    best = min(finite) if finite and bold_best else None
    for P in populations:
        cells = []
        for b in budgets:
            v = vals[(P, b)]
            text = _f(v, spec) if v is not None else "--"
            cells.append(f"**{text}**" if best is not None and v == best else text)
        lines.append(f"| {P} | " + " | ".join(cells) + " |")
    return lines


def write_markdown(path: str, data: Dict[int, Dict], grid: Sequence[Dict]) -> None:
    populations = list(data)
    n_sets = {P: len(s) for P, s in data.items()}
    gmax = min(len(next(iter(next(iter(s.values())).values()))["best_so_far"])
               for s in data.values())
    gens = [g for g in GENERATIONS if g <= gmax]
    lines = [
        "# Population size x generations, four constraint handlers",
        "",
        "One `run_penalty_ab` study per population size, run to "
        f"{gmax} generations and read at every cut from the per-generation traces "
        "(nothing in the GA depends on the configured generation count, so the cut "
        "*is* the shorter run). Sets per population: "
        + ", ".join(f"P={P}: {n}" for P, n in n_sets.items())
        + ". Retract model on, crossover 0.8, mutation 0.2, elitism 10%, "
        "tournament k=3. Cells are mean best cycle time (s) over feasible runs; "
        "**bold** is the best cell of the table.",
        "",
    ]
    infeasible = [r for r in grid if r["feasible"] < 1.0]
    if infeasible:
        lines += ["Cells where not every run found a feasible layout: "
                  + ", ".join(f"{r['arm']} P={r['population']} {r['budget_kind']}={r['budget']:g} "
                              f"({r['feasible']:.0%})" for r in infeasible[:20]), ""]

    for kind, budgets, title in (
        ("generations", gens, "Equal generations"),
        ("evaluations", EVAL_BUDGETS, "Equal evaluations (P x G fixed)"),
        ("cpu", CPU_BUDGETS, "Equal CPU per run"),
    ):
        lines += [f"## {title}", ""]
        for arm in ARMS:
            lines += [f"**`{arm}`**", ""] + _table(grid, kind, budgets, populations, arm) + [""]

    lines += ["## CPU per run (s), equal generations", ""]
    for arm in ARMS:
        lines += [f"**`{arm}`**", ""] + _table(grid, "generations", gens, populations, arm,
                                               value="cpu", spec=".1f", bold_best=False) + [""]

    # Does the arm ranking depend on (P, G)?  Repair against each other arm at
    # every equal-generation cell, Holm over the whole table.
    lines += ["## Is repair's lead stable across the grid? (equal generations)", "",
              "Gain = other arm minus repair, paired by (instance, seed); positive "
              "means repair faster. Holm over every cell of the table.", ""]
    tests = []
    for P in populations:
        for g in gens:
            ref = lookup(grid, "generations", g, P, "repair")
            for arm in ("baseline", "penalty_cvd", "penalty_count"):
                other = lookup(grid, "generations", g, P, arm)
                if ref and other:
                    t = paired(ref["per_key"], other["per_key"])
                    if t:
                        tests.append({"P": P, "G": g, "arm": arm, **t})
    for t, adj in zip(tests, holm([t["p"] for t in tests])):
        t["holm"] = adj
    lines += ["| P | G | vs | gain [s] | 95% CI | repair wins | p (Holm) |",
              "|---:|---:|---|---:|---|---:|---:|"]
    for t in tests:
        lines.append(f"| {t['P']} | {t['G']} | `{t['arm']}` | {t['gain']:+.3f} | "
                     f"[{t['ci_low']:+.3f}, {t['ci_high']:+.3f}] | {t['wins']}/{t['n']} | "
                     f"{_p(t['holm'])} |")

    # Between population sizes at a fixed evaluation budget, against P=200.
    if 200 in populations:
        lines += ["", "## Population size at equal evaluations, against P=200", "",
                  "Gain = P=200 minus P, paired by (instance, seed); positive means "
                  "the population size beat P=200 for the same number of evaluations.", "",
                  "| arm | evals | P | gain [s] | 95% CI | wins | p |",
                  "|---|---:|---:|---:|---|---:|---:|"]
        for arm in ARMS:
            for b in EVAL_BUDGETS:
                ref = lookup(grid, "evaluations", b, 200, arm)
                for P in populations:
                    other = lookup(grid, "evaluations", b, P, arm)
                    if P == 200 or not ref or not other:
                        continue
                    t = paired(other["per_key"], ref["per_key"])
                    if t:
                        lines.append(f"| `{arm}` | {b:,} | {P} | {t['gain']:+.3f} | "
                                     f"[{t['ci_low']:+.3f}, {t['ci_high']:+.3f}] | "
                                     f"{t['wins']}/{t['n']} | {_p(t['p'])} |")

    # Smallest cost within TOLERANCE of the best equal-generation cell.
    lines += ["", f"## Cheapest setting within {TOLERANCE} s of each arm's best", "",
              "| arm | best (P, G) | best [s] | cheapest within tolerance | its time [s] | its CPU [s] |",
              "|---|---|---:|---|---:|---:|"]
    for arm in ARMS:
        cells = [r for r in grid if r["budget_kind"] == "generations" and r["arm"] == arm
                 and r["budget"] in gens and r["cycle_time"] is not None and r["feasible"] == 1.0]
        if not cells:
            continue
        best = min(cells, key=lambda r: r["cycle_time"])
        ok = [r for r in cells if r["cycle_time"] <= best["cycle_time"] + TOLERANCE]
        cheap = min(ok, key=lambda r: r["cpu"])
        lines.append(f"| `{arm}` | ({best['population']}, {best['budget']}) | "
                     f"{best['cycle_time']:.3f} | ({cheap['population']}, {cheap['budget']}) | "
                     f"{cheap['cycle_time']:.3f} | {cheap['cpu']:.1f} |")

    lines += ["", "![population x generations](popgen.png)", ""]
    with open(path, "w", encoding="utf-8") as handle:
        handle.write("\n".join(lines))


def write_figure(path: str, data: Dict[int, Dict]) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, len(ARMS), figsize=(4 * len(ARMS), 7.4), sharey=True)
    for col, arm in enumerate(ARMS):
        for P, study in data.items():
            for row, (trace, xlabel) in enumerate((("evaluations_trace", "evaluations"),
                                                  ("cpu_elapsed_trace", "CPU per run [s]"))):
                n = min(len(arms[arm]["best_so_far"]) for arms in study.values())
                x = np.mean([arms[arm][trace][:n] for arms in study.values()], axis=0)
                y = np.array([[np.nan if v is None else v for v in arms[arm]["best_so_far"][:n]]
                              for arms in study.values()])
                y = np.nanmean(y, axis=0)
                axes[row, col].plot(x, y, label=f"P={P}")
                axes[row, col].set_xscale("log")
                axes[row, col].set_xlabel(xlabel)
                axes[row, col].grid(alpha=0.3, which="both")
        axes[0, col].set_title(arm)
    for ax in axes[:, 0]:
        ax.set_ylabel("mean best cycle time [s]")
    lo = min(np.nanmin(line.get_ydata()) for ax in axes.flat for line in ax.get_lines())
    axes[0, 0].set_ylim(lo - 0.05, lo + 1.2)
    axes[0, 0].legend()
    fig.tight_layout()
    fig.savefig(path, dpi=140)
    plt.close(fig)


def main(argv: Optional[Sequence[str]] = None) -> None:
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    parser.add_argument("--sweep-dir", required=True)
    args = parser.parse_args(argv)

    data = load(args.sweep_dir)
    if not data:
        raise SystemExit(f"no p<P>/raw studies under {args.sweep_dir}")
    grid = build_grid(data)
    with open(os.path.join(args.sweep_dir, "grid.csv"), "w", newline="", encoding="utf-8") as h:
        fields = ("budget_kind", "budget", "population", "arm", "n", "feasible",
                  "cycle_time", "cpu", "evals", "generations")
        writer = csv.DictWriter(h, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(grid)
    write_markdown(os.path.join(args.sweep_dir, "summary.md"), data, grid)
    write_figure(os.path.join(args.sweep_dir, "popgen.png"), data)
    print(f"  populations: {', '.join(map(str, data))}")
    print(f"  -> {os.path.join(args.sweep_dir, 'summary.md')}")


if __name__ == "__main__":
    main()
