# Static GA on robot cycle time -- Puma 560 workcell

1 'test' instances x 3 seeds = 3 runs. **Cycle time is minimised** (seconds); an infeasible layout scores `+inf` and is excluded from the statistics, with `n_feasible` recording how many runs of each instance produced a layout at all.

## What is being optimised

The objective is the time for one full cycle: home -> every loading port in process order -> home, under a trapezoidal velocity profile per move, plus a 0.3 s dwell at each station. Each move is time-scaled so that no joint exceeds its rate or acceleration limit, and the move takes as long as the slowest joint needs -- so the cost of a layout depends on which joints a move loads, not merely on how far the tool travels.

A layout is feasible only if all five hold:

1. footprints inside the cell, clear of the robot column, and mutually disjoint;
2. every port reachable with a vertical top-down approach;
3. every joint inside its travel at every port;
4. `sigma_min` of the position Jacobian >= 0.14 m/rad at every port, which is the anti-singularity condition;
5. at every station, no part of the robot within the clearance margin of any machine -- except the wrist and gripper against the machine the tool is descending onto, which is the approach corridor;

and a **single** arm posture must satisfy 2-5 at every station, since a real cell does not reconfigure the arm mid-cycle. Criterion 5 governs the stations only: between them the moves interpolate in joint space and can still sweep the arm through a machine, which `--check-path` rejects and the README discusses.

## Robot

4-DOF arm on Puma 560 link geometry: shoulder 0.6718 m up, upper arm 0.4318 m, forearm 0.4323 m (= hypot(0.0203, 0.4318)), shoulder offset 0.15005 m, tool 0.1 m. Reach 0.349-0.864 m from the shoulder. Joint rate limits 1.74533, 1.65806, 1.74533, 2.26893 rad/s; accelerations 6, 6, 6, 10 rad/s^2.

Joint travel is the manufacturer's, mapped into this model's angle convention by a fixed per-joint offset and otherwise unaltered: q1 -160.0 to 160.0 deg (joint 1, -160 to 160), q2 -45.0 to 225.0 deg (joint 2, -45 to 225), q3 -137.7 to 132.3 deg (joint 3, -225 to 45), q4 -97.3 to 102.7 deg (joint 5, -100 to 100). The two roll joints, 4 and 6, are locked at zero because a top-down pick does not need them.

## GA

Population 200, 300 generations (60000 evaluations), tournament k=3, uniform crossover p=0.9, mutation p=0.1 per machine, elitism 10%. **No local search, no learned control.**

Across every feasible run: cycle time 10.143 +- 0.451 s, best 9.742 s (`test_3` run 1, posture right/down).


## Per instance

| instance | feasible | cycle mean (s) | sd | median | best | min sigma mean | evals | CPU mean (s) |
|---|---|---|---|---|---|---|---|---|
| test_3 | 3/3 | 10.143 | 0.451 | 10.055 | 9.742 | 0.1876 | 54200 | 165.2 |

## Posture chosen by the best layout

| posture | runs |
|---|---|
| right/down | 2 |
| left/up | 1 |

---

Wall-clock time is inflated with `--workers > 1`; CPU time and the evaluation count are the reliable effort measures in that case.
