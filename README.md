# Robotic Arm Study: Layout Optimization Against Cycle Time

A genetic algorithm that places machines in a robot workcell to minimise the
**cycle time of a Puma 560**. A layout only counts if the arm can reach every
machine's loading port, stays away from singular poses, and keeps clear of
every machine over the whole cycle.

One cycle is `home → every loading port in process order → home`. The arm
lifts above the cell between stations, so each station is descend, dwell,
retract, traverse. Each move is a coordinated joint-space motion with a
trapezoidal velocity profile, limited by the Puma's published joint speeds.

The GA's chromosome is `[x, y, rotation]` per machine. Defaults: population
200, 300 generations, tournament `k=3`, uniform crossover `p=0.9`, mutation
`p=0.1`, elitism 10 %.

## Requirements

* Python 3
* `numpy` and `matplotlib`. Nothing else is needed for the GA, the figures
  or the self-test.
* `mujoco` (optional), only for the physics check in `run_mujoco.py`

```bash
pip install numpy matplotlib
pip install mujoco          # optional
```

Run every command from the repository root. The root folder can have any
name. Each script also runs by path, e.g. `python experiments/run_ga.py`.

## Quick start

```bash
python -m tests.selftest                                              # check that everything works
python -m experiments.run_ga --instances 0 --runs 3 --generations 60  # a quick run, a couple of minutes
python -m experiments.run_ga --workers 12 --animate                   # the full study
```

The output goes to `results/baseline/` by default. Each instance gets
`figures/<instance>/layout.png` (plan view) and `trajectory.png` (3-D view of
the arm at each station). `--animate` adds `cycle.gif` and `--viewer` adds
`cycle.html`, a drag-to-rotate viewer you open in a browser.

## `run_ga.py`: the main optimiser

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
| `--crossover-rate` | 0.9 | |
| `--mutation-rate` | 0.1 | per machine |
| `--elitism-rate` | 0.1 | |
| `--tournament-k` | 3 | |
| `--repair` | off | move a child with overlapping or out-of-bounds machines back to a valid placement instead of scoring it `+∞` |
| `--repair-margin` | 0.005 | slack (m) used by `--repair` |

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

## Checking a layout under physics (MuJoCo)

`run_mujoco.py` loads a finished run, builds the cell in MuJoCo and checks the
layout three ways:

1. exact clearance between the robot's solid model and each machine,
2. joint torque demand against the Puma's torque limits,
3. tracking of the planned motion under a computed-torque controller.

It needs `pip install mujoco`.

```bash
python -m experiments.run_ga --instances 2 --runs 3 --results-dir results/demo
python -m experiments.run_mujoco --results demo --instance test_3 --view
```

| option | default | meaning |
|---|---|---|
| `--results NAME` | `retract` | results tree under `results/` to read from |
| `--instance NAME` | `test_1` | instance to load, e.g. `test_3` |
| `--run K` | fastest | run index within that instance |
| `--dt` | 0.001 | physics and control step (s) |
| `--bandwidth` | 25.0 | controller natural frequency (rad/s) |
| `--friction-comp` | 1.0 | Coulomb friction feed-forward (1 = exact, 0 = none) |
| `--no-gif` | off | skip writing `cycle.gif` |
| `--view` | off | replay the simulated cycle in MuJoCo's interactive viewer |

Instance names are the split plus the index plus one, so `--instances 2` on
the `test` split is `test_3`.

The output goes to `results/mujoco/<tree>_<instance>/`: `scene.xml`,
`report.md`, `torques.png`, `tracking.png` and `cycle.gif`. To open the scene
on its own:

```bash
python -m mujoco.viewer --mjcf results/mujoco/<tree>_<instance>/scene.xml
```

## Other scripts

All of these take `--help` and `--workers`.

```bash
python -m experiments.make_arm_figure [--spheres]    # dimensioned drawing of the robot -> results/model/arm_model.png
python -m experiments.run_ablation --workers 12      # crossover x mutation rate grid
python -m experiments.run_repair_ab --workers 12     # paired A/B: with vs without --repair
python -m experiments.run_penalty_ab --workers 12    # +inf wall vs repair vs two penalty functions
python -m experiments.run_retract_ab --workers 12    # with vs without the retract
```

## Project layout

```
kinematics/   the Puma 560: FK, closed-form IK, Jacobians, trajectories, solid model
layout/       machines, placement geometry, and the instance sets
optimizer/    the cycle-time evaluator, the GA, and the penalty variants
sim/          the MuJoCo model of the cell (optional)
viz/          figures, animations, and the HTML viewer
experiments/  the command-line scripts above
tests/        the self-test
results/      output, one directory per run
_paths.py     puts the source directories on sys.path
```
