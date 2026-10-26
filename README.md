# Robotic Arm Study: Layout Optimization Against Cycle Time

A genetic algorithm that places machines in a robot workcell to minimise the
**cycle time of a Puma 560**. A layout only counts if the arm can reach every
machine's loading port, stays away from singular poses, and keeps clear of
every machine over the whole cycle.

The study's question is how to handle the geometric constraints (machines
inside the cell, off the robot's column, not overlapping). Four ways are
compared on identical runs: a hard `+∞` wall, a **placement repair**
operator, and two graded penalties. The repair operator is the method under
test.

One cycle is `home → every loading port in process order → home`. The arm
lifts above the cell between stations, so each station is descend, dwell,
retract, traverse. Each move is a coordinated joint-space motion with a
trapezoidal velocity profile, limited by the Puma's published joint speeds.

## Settings used in the study

The headline results (`results/cx08_mut02_r20_g300/`) and the machine-count,
population/generation and sensitivity studies all use these values.

Crossover 0.8 / mutation 0.2 were picked by the operator ablation (lowest
mean cycle time and best mean rank of 30 combinations) and are the defaults
in every script. The earlier `repair_vs_baseline/`, `retract_vs_noretract/`
and `retract_baseline/` trees were run with the old 0.9 / 0.1, so pass
`--crossover-rate 0.9 --mutation-rate 0.1` to reproduce those.
`run_penalty_ab.py` defaults to 5 runs and 150 generations, so pass
`--runs 20 --generations 300` for the main study.

**GA**

| setting | study value | CLI default |
|---|---|---|
| population | 200 | 200 |
| generations | 300 | 300 (`run_penalty_ab`: 150) |
| crossover rate (per pair, uniform) | 0.8 | 0.8 |
| mutation rate (per machine) | 0.2 | 0.2 |
| mutation width σ | 0.10 × each position gene's range | 0.10 |
| elitism | 10 % (20 elites, 180 offspring per generation) | 10 % |
| tournament size k | 3 | 3 |
| initial-population seeding | up to 200 rejection-sampling draws per member | 200 |
| runs | 20 seeds × 10 test instances = 200 paired sets | `run_ga` 10, `run_penalty_ab` 5 |
| run seed | `42 + 100 × instance_index + run` | same |
| repair margin | 0.005 m | 0.005 m |

**Fitness calls per run: 54,200** = 200 (initial) + 300 × 180 offspring. The
`max_evaluations = 60000` in the metadata is `population × generations`, an
upper bound that never binds because elites are not re-scored. Calls answered
from the fitness cache still count.

**Robot and objective**

| setting | value |
|---|---|
| `sigma_min` floor (manipulability, every port) | 0.14 m/rad |
| dwell per station | 0.30 s |
| retract height above the tallest machine | 0.06 m |
| arm-to-machine clearance (stations and along every move) | 0.005 m |
| clearance samples per move | 12 |
| joint speeds (waist, shoulder, elbow, wrist pitch) | 100, 95, 100, 130 °/s (Puma published values) |
| joint accelerations | 6, 6, 6, 10 rad/s² (assumed; Unimation never published them) |
| home tool position | (0.40, 0.0, 0.45) m |

**Penalty arms**: `T_inf = 1000 s`, CVD weight `w = 1`, overlap-count weight
`m = 0.05` (only for `penalty_count`).

## Problem instances

Every instance is a 1.4 m × 1.4 m floor (`x, y ∈ [-0.7, 0.7]`) with the robot
in the middle on a 0.67 m pedestal (reach 0.864 m from the shoulder).

| | value |
|---|---|
| machines per instance | 6 (4–8 in the machine-count study) |
| machine footprint | 0.10–0.18 m × 0.10–0.18 m, height 0.10–0.40 m (uniform random) |
| loading port | on the top face, offset 0.3 × width from the centre, so rotation moves it |
| robot keep-out disc | radius 0.18 m around the base |
| gap between machines | ≥ 0.02 m |
| process order | a random permutation per instance |
| splits | `test` 10 instances (seeds 1000–1009), `train` 15 (2000–2014), `generalization` 6 (4000–4005) |

All reported results are on the `test` split. Instance names are the split
plus the index plus one, so `--instances 2` on `test` is `test_3`.

## How a layout is scored

The chromosome is `[x, y, rotation]` per machine (18 genes for 6 machines).
Positions are bounded so every rotation of a machine stays inside the cell;
rotations wrap at 360°.

A layout is **feasible** only if it passes all of these, in this order:

1. **Placement**: every footprint inside the cell, clear of the keep-out disc,
   and 0.02 m from every other machine.
2. **Reach**: the tool reaches every port with a vertical approach and can lift
   to the retract height above it, inside every joint's travel.
3. **Conditioning**: `sigma_min` of the position Jacobian ≥ 0.14 m/rad at every
   port (not at the lift poses).
4. **Clearance**: the arm (bounded by spheres fitted to its solid model) stays
   0.005 m from every machine, at the stations and at 12 samples along every
   move. The tool may enter the corridor directly above the machine it serves.
5. **One posture**: a single IK branch (shoulder left/right × elbow up/down)
   must pass 2–4 for the whole cycle. The fastest passing branch is used.

The fitness is the cycle time in seconds (moves + dwells), minimised.

## The GA

A plain generational GA with no local search, so differences between arms
come from the constraint handling, not from the search.

* **Initial population**: each member is drawn uniformly inside the gene
  bounds until it passes the placement test (up to 200 tries). Only about 5 %
  of uniform layouts place without overlap, so a uniform population would have
  almost no finite fitness. All four arms of a set receive the identical
  starting population.
* **Selection**: tournament of k = 3, drawn without replacement.
* **Crossover**: with probability 0.8, uniform crossover gene by gene (each
  gene from either parent with probability 0.5). Two children per pair.
* **Mutation**: per machine with probability 0.2: add Gaussian noise
  (σ = 0.10 × range) to `x` and `y`, and re-draw the rotation uniformly in
  [0°, 360°). Positions are then clipped to the gene bounds.
* **Elitism**: the best 10 % are copied unchanged and not re-scored.

## The repair operator

`--repair` turns it on (`ConfigurableGA.repair` →
`layout.geometry.repair_placement`). It runs on **every offspring** right
after mutation and clipping, before scoring. Elites and the initial
population are not repaired.

What it does:

* If the layout already passes the placement test, it is returned unchanged.
* Otherwise it moves machine **positions only**. Rotations, and so the side
  of the machine the port is on, are never changed.
* Up to 12 passes of three moves, stopping early when a pass moves nothing:
  1. **Clip** each centre into the cell, inset by the machine's half-diagonal
     plus the margin.
  2. **Keep-out**: a machine intruding into the robot's disc (0.18 m +
     margin) is pushed radially outward by its shortfall.
  3. **Overlap**: for each pair closer than the 0.02 m gap + margin, find the
     separating axis needing the smallest push (separating-axis test over the
     4 edge normals) and move both machines apart along it, half the depth
     each.
* A final clip so nothing ends outside the cell.
* The margin (0.005 m) pushes machines slightly *past* each constraint, so a
  repaired layout is not left exactly on a boundary.

Properties:

* **Lamarckian**: the repaired genes replace the child's genes in the
  population.
* **Objective-blind**: it never evaluates a cycle time and is not counted as
  a fitness call. Its CPU time is counted, which matters for the equal-CPU
  comparisons.
* **Not guaranteed**: on a crowded layout 12 passes may not be enough. The
  layout is then scored as usual and gets `+∞` if it still fails.
* It only fixes placement. A repaired layout still has to pass reach,
  conditioning, clearance and posture.

## The four constraint-handling arms

`run_penalty_ab.py` runs all four with the same seeds and starting
populations. Only the scoring differs.

| arm | infeasible layout scores |
|---|---|
| `baseline` | `+∞` (hard wall) |
| `repair` | `+∞`, but placement violations are repaired before scoring |
| `penalty_cvd` | `T_inf + w·CVD(x)`, where CVD is the summed violation of the 45 geometric constraints in metres |
| `penalty_count` | `T_inf + w·CVD(x) + m·N(x)`, `N` = number of machines in at least one overlap |

A layout that is placeable but fails a robot constraint scores exactly
`T_inf` in the penalty arms. Since selection is rank-based and `T_inf` is
above every cycle time, `w` has no effect in `penalty_cvd`; the ratio `m/w`
does matter in `penalty_count`.

Comparisons are paired by (instance, seed) and reported at equal generations
and at equal CPU, with paired sign-flip permutation tests, Wilcoxon
signed-rank tests, bootstrap 95 % CIs and Holm correction.

## Requirements

* Python 3 (tested on 3.12 and 3.14)
* `numpy` (tested on 2.5) and `matplotlib`. Nothing else is needed for the
  GA, the figures or the self-test.
* `mujoco` (optional), only for the physics check in `run_mujoco.py`

```bash
pip install numpy matplotlib
pip install mujoco          # optional
```

Run every command from the repository root. The root folder can have any
name. Each script also runs by path, e.g. `python experiments/run_ga.py`.

## Quick start

```bash
python -m tests.selftest                                              # check that everything works (~5 s)
python -m experiments.run_ga --instances 0 --runs 3 --generations 60  # a quick run, a couple of minutes
python -m experiments.run_ga --repair --workers 12 --animate                 # the study settings, with repair
```

The output goes to `results/baseline/` by default. Each instance gets
`figures/<instance>/layout.png` (plan view) and `trajectory.png` (3-D view of
the arm at each station). `--animate` adds `cycle.gif` and `--viewer` adds
`cycle.html`, a drag-to-rotate viewer you open in a browser.

## Reproducing the results

| result tree | what it is | command |
|---|---|---|
| `cx08_mut02_r20_g300/` | **main study**: 4 arms × 10 instances × 20 seeds | `python -m experiments.run_penalty_ab --runs 20 --generations 300 --workers 22 --output results/cx08_mut02_r20_g300` |
| `cx08_mut02_r20_g300_machines/m{4..8}/` | the same at 4–8 machines | the command above plus `--machines N --output results/cx08_mut02_r20_g300_machines/mN`, then `python -m experiments.summarize_penalty_machines --sweep-dir results/cx08_mut02_r20_g300_machines` |
| `ga_ablation/` | crossover × mutation grid, 20 seeds, no repair | `python -m experiments.run_ablation --crossover-rates 0.9 0.8 0.7 0.5 0.3 --runs 20 --workers 22 --results-dir results/ga_ablation` |
| `popgen/p{50,100,200,400}/` | population × generations, 4 arms, 10 seeds, 500 generations | `python -m experiments.run_penalty_ab --only run --population P --generations 500 --runs 10 --output results/popgen/pP`, then `python -m experiments.summarize_popgen --sweep-dir results/popgen` |
| `sensitivity_repair/` | `sigma_min`, clearance and retract sweeps, wall vs repair, 10 seeds | `python -m experiments.run_sensitivity --runs 10 --output results/sensitivity_repair` |
| `repair_vs_baseline/` | early paired A/B, 15 seeds, crossover 0.9 / mutation 0.1 | `python -m experiments.run_repair_ab --runs 15 --crossover-rate 0.9 --mutation-rate 0.1` |
| `retract_vs_noretract/` | retract vs direct moves, 5 seeds, crossover 0.9 / mutation 0.1 | `python -m experiments.run_retract_ab --runs 5 --crossover-rate 0.9 --mutation-rate 0.1` |
| `mujoco/` | physics check of the best layouts | `python -m experiments.run_mujoco --results cx08_mut02_r20_g300 --instance test_N` |

`results/sensitivity/` (5 seeds) was run with the older, slower repair code;
its equal-CPU columns are out of date. Use `sensitivity_repair/`.
`results/cx08_mut02_r20_g300/REPAIR_OPTIMIZATION.md` explains the repair
speed-up and why the equal-CPU result depends on it.

## `run_ga.py`: a single-arm run

```bash
python -m experiments.run_ga [options]
```

**What to run**

| option | default | meaning |
|---|---|---|
| `--split {test,train,generalization}` | `test` | which instance set to use |
| `--instances N [N ...]` | whole split | instance indices, e.g. `--instances 0 3` |
| `--runs N` | 10 | independent seeds per instance |

**GA operators**

| option | default | meaning |
|---|---|---|
| `--population-size` | 200 | |
| `--generations` | 300 | |
| `--crossover-rate` | 0.8 | |
| `--mutation-rate` | 0.2 | per machine |
| `--elitism-rate` | 0.1 | |
| `--tournament-k` | 3 | |
| `--repair` | off | repair children with overlapping, out-of-bounds or keep-out machines instead of scoring them `+∞` |
| `--repair-margin` | 0.005 | slack (m) the repair pushes past each constraint |

**Robot and objective**

| option | default | meaning |
|---|---|---|
| `--sigma-min` | 0.14 | manipulability floor (m/rad); keeps the arm away from singular poses |
| `--dwell` | 0.3 | seconds held at each station |
| `--retract` | 0.06 | metres above the tallest machine the tool lifts to between stations |
| `--no-retract` | off | use direct point-to-point moves. Pair it with `--no-path-check`, since whole-cycle clearance can't be met without the retract |
| `--no-path-check` | off | test clearance only at the stations, not along the moves |
| `--no-arm-check` | off | drop the arm-vs-machine clearance check entirely |
| `--arm-clearance` | 0.005 | metres the robot must keep from every machine |

**Execution and output**

| option | default | meaning |
|---|---|---|
| `--workers N` | 1 | parallel worker processes |
| `--results-dir DIR` | `results/baseline` | where to write the results |
| `--only {run,aggregate,figures} ...` | all | run only these stages, e.g. `--only aggregate figures` rebuilds the summaries and figures from existing `raw/` runs |
| `--animate` | off | also write an orbiting `cycle.gif` per instance |
| `--spin DEG` | 360 | how far the GIF camera orbits (0 = fixed view) |
| `--viewer` | off | also write the interactive `cycle.html` |
| `--no-traces` | off | leave the per-generation history out of the run JSON |
| `--dry-run` | off | print the plan without running anything |

A results tree contains `runs.csv`, `runs.json`, `summary.csv`, `summary.md`,
`metadata.json`, `raw/<instance>_run<k>.json`, and `figures/` (one folder per
instance, plus `convergence.png`).

## `run_penalty_ab.py`: the four-arm comparison

| option | default | meaning |
|---|---|---|
| `--split` | `test` | instance set |
| `--machines` | 6 | machines per instance |
| `--instances N` | 0 | first N instances (0 = all) |
| `--runs` | 5 | seeds per instance (study: 20) |
| `--generations` | 150 | (study: 300) |
| `--population` | 200 | |
| `--crossover-rate` | 0.8 | |
| `--mutation-rate` | 0.2 | |
| `--elitism-rate`, `--tournament-k`, `--mutation-sigma` | 0.1, 3, 0.10 | |
| `--sigma-min`, `--dwell`, `--retract` | 0.14, 0.30, 0.06 | |
| `--workers` | CPU count − 2 | |
| `--output` | `results/penalty` | |
| `--only {run,aggregate}` | both | |
| `--no-resume` | off | rerun runs whose JSON already exists |

Each run's reported cycle time is re-checked against the plain evaluator
before it is written, so a penalty score can't be reported as a cycle time.

## Checking a layout under physics (MuJoCo)

`run_mujoco.py` loads a finished run, builds the cell in MuJoCo and checks the
layout three ways:

1. exact clearance between the robot's solid model and each machine,
2. joint torque demand against the Puma's torque limits,
3. tracking of the planned motion under a computed-torque controller.

It needs `pip install mujoco`.

```bash
python -m experiments.run_mujoco --instance test_3 --view     # best run of test_3 in the main study
```

| option | default | meaning |
|---|---|---|
| `--results NAME` | `cx08_mut02_r20_g300` | results tree under `results/` to read from |
| `--instance NAME` | `test_1` | instance to load, e.g. `test_3` |
| `--run K` | fastest | run index within that instance |
| `--dt` | 0.001 | physics and control step (s) |
| `--bandwidth` | 25.0 | controller natural frequency (rad/s) |
| `--friction-comp` | 1.0 | Coulomb friction feed-forward (1 = exact, 0 = none) |
| `--no-gif` | off | skip writing `cycle.gif` |
| `--view` | off | replay the simulated cycle in MuJoCo's interactive viewer |

The output goes to `results/mujoco/<tree>_<instance>/`: `scene.xml`,
`report.md`, `torques.png`, `tracking.png` and `cycle.gif`. To open the scene
on its own:

```bash
python -m mujoco.viewer --mjcf results/mujoco/<tree>_<instance>/scene.xml
```

## Other scripts

All of these take `--help`.

```bash
python -m experiments.make_arm_figure [--spheres]    # dimensioned drawing of the robot -> results/model/arm_model.png
python -m experiments.make_repair_figure             # one repair step on a real child -> results/model/repair_example.png
python -m experiments.run_ablation --workers 12      # crossover x mutation rate grid
python -m experiments.ablation_stats                 # rank statistics for results/ga_ablation
python -m experiments.run_repair_ab --workers 12     # paired A/B: with vs without --repair
python -m experiments.run_penalty_ab --workers 12    # +inf wall vs repair vs two penalty functions
python -m experiments.run_retract_ab --workers 12    # with vs without the retract
python -m experiments.run_sensitivity --dry-run      # threshold sweeps (also --only pathcheck)
python -m experiments.make_budget_figures            # equal-generation / equal-CPU figures for the paper
```

## Known limitations

* **Clearance between samples.** Clearance along a move is checked at 12
  samples. Re-checking every best layout at 100 samples per move
  (`run_sensitivity --only pathcheck`) rejects about 1 % of them at the
  default settings and about 4 % in `popgen/p200`. At `retract = 0.03 m`
  every best layout fails the dense check, so treat that sensitivity cell
  with care. See `results/sensitivity_repair/pathcheck.md`.
* **Last generation's offspring are not used.** The GA scores a new set of
  180 offspring after the final generation but never reads it, so 180 of the
  54,200 fitness calls per run are wasted. This applies equally to all arms
  and does not bias the comparison.
* **Equal-CPU results depend on implementation speed.** Equal generations is
  the stable comparison (see `REPAIR_OPTIMIZATION.md`).
* The robot is a 4-joint model on Puma 560 geometry (waist, shoulder, elbow,
  wrist pitch); the two roll joints are locked.

## Project layout

```
kinematics/   the Puma 560: FK, closed-form IK, Jacobians, trajectories, solid model
layout/       machines, placement geometry, the repair, and the instance sets
optimizer/    the cycle-time evaluator, the GA, and the penalty variants
sim/          the MuJoCo model of the cell (optional)
viz/          figures, animations, and the HTML viewer
experiments/  the command-line scripts above
tests/        the self-test
results/      output, one directory per run
_paths.py     puts the source directories on sys.path
```
