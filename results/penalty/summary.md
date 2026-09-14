# Repair, the wall, and two penalties

50 matched sets (10 instances x 5 seeds), population 200, 150 generations, 30000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.9, mutation p=0.1 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = 1000 s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For six machines there are 45 of them:

| count | constraint | `g_i(x)` |
| ---: | --- | --- |
| 24 | footprint inside the cell | signed excursion of the footprint past each of the four walls |
| 6 | clear of the robot column | `keep_out_radius - distance(footprint, base)` |
| 15 | machines mutually disjoint | `overlap_depth(i, j)`, the minimum translation separating the pair with the clearance included |

`CVD(x) = 0` exactly when `layout_is_placeable(x)` is true; this is checked
rather than assumed. `N(x)` is the number of machines caught in at least one
overlap, an integer in `0..6`.

```
baseline        F(x) = T(x)   if x feasible
                     = +inf   otherwise

repair          F(x) = T(x)   if x feasible          (an unplaceable child is
                     = +inf   otherwise               projected onto the
                                                      placeable set first)

penalty_cvd     F(x) = T(x)                  if x feasible
                     = T_inf + w * CVD(x)    otherwise           w = 1

penalty_count   F(x) = T(x)                            if x feasible
                     = T_inf + w * CVD(x) + m * N(x)   otherwise
                                                       w = 1, m = 0.05
```

**Why `w` is not a free parameter to be argued over in `penalty_cvd`.** The GA
selects by tournament, elitises by `argsort`, and tracks its best by `min` --
every one of those is rank-based. Because the `T_inf` layer already separates
feasible from infeasible, multiplying `CVD` by any positive `w` leaves all
three unchanged, so `w` cannot affect the search at all. The usual objection to
penalty methods -- that the exchange rate between metres of overlap and seconds
of cycle time is an unstated preference -- does not apply to this formulation,
because the layers never let the two quantities trade.

The ratio `m/w` in `penalty_count` **does** matter: it decides whether four
shallow overlaps rank above or below one deep one. `m = 0.05` makes one
overlapping machine worth 0.05 m of violation depth, against a median
violation of about 0.13 m over random infeasible layouts -- so the two terms
are of comparable size rather than one dominating.

## Cycle time, at equal generations

| arm | runs feasible | mean cycle time (s) | median (s) | best (s) | mean CPU (s) | feasible evaluations |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| `baseline` | 50/50 | 11.0511 | 11.1600 | 9.7926 | 127.4 | 0.5215 |
| `repair` | 50/50 | 10.6627 | 10.7897 | 9.7259 | 266.1 | 0.7157 |
| `penalty_cvd` | 50/50 | 10.9888 | 11.0242 | 9.8565 | 173.6 | 0.5399 |
| `penalty_count` | 50/50 | 10.9811 | 11.1103 | 9.8276 | 170.1 | 0.5363 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 11.0511 | 10.6627 | -0.3884 | [-0.4590, -0.3183] | 2 | 48 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 10.9888 | 10.6627 | -0.3261 | [-0.4012, -0.2564] | 3 | 47 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 10.9811 | 10.6627 | -0.3184 | [-0.3823, -0.2591] | 4 | 46 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 0.5215 | 0.7157 | -0.1942 | [-0.2120, -0.1766] | 0 | 50 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 0.5399 | 0.7157 | -0.1758 | [-0.1918, -0.1602] | 0 | 50 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 0.5363 | 0.7157 | -0.1794 | [-0.1967, -0.1624] | 0 | 50 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 127.4016 | 266.1000 | 138.6984 | [121.4232, 155.4866] | 50 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 173.6125 | 266.1000 | 92.4875 | [75.9878, 108.8806] | 45 | 5 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 170.1297 | 266.1000 | 95.9703 | [77.5400, 114.0129] | 46 | 4 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 50 | 10.6627 | 11.0511 | 0.3884 | [0.3183, 0.4590] | 48 | 2 | 0.0000 |
| `penalty_cvd` | 50 | 10.9888 | 11.0511 | 0.0623 | [-0.0419, 0.1644] | 30 | 20 | 0.2484 |
| `penalty_count` | 50 | 10.9811 | 11.0511 | 0.0699 | [-0.0183, 0.1531] | 32 | 18 | 0.1197 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 50 | 10.7225 | 11.0539 | 0.3314 | [0.2621, 0.4003] | 46 | 4 | 0.0000 |
| `penalty_cvd` | 50 | 11.0510 | 11.0539 | 0.0029 | [-0.1062, 0.1105] | 25 | 25 | 0.9595 |
| `penalty_count` | 50 | 11.0384 | 11.0539 | 0.0155 | [-0.0874, 0.1093] | 30 | 20 | 0.7708 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 145.1 | 119.3 | 11.0539 |
| `repair` | 74.4 | 118.8 | 10.7225 |
| `penalty_cvd` | 113.1 | 119.3 | 11.0510 |
| `penalty_count` | 110.1 | 119.5 | 11.0384 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 11.0539 | 10.7225 | -0.3314 | [-0.4003, -0.2621] | 4 | 46 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 11.0510 | 10.7225 | -0.3285 | [-0.4109, -0.2532] | 4 | 46 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 11.0384 | 10.7225 | -0.3159 | [-0.3957, -0.2461] | 4 | 46 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 5 | 10.1441 | 9.8947 | 10.1254 | 10.2417 |
| test_2 | 5 | 11.4041 | 11.0432 | 11.4627 | 11.4990 |
| test_3 | 5 | 10.1642 | 9.9416 | 10.1462 | 10.1126 |
| test_4 | 5 | 11.3577 | 10.8612 | 11.1229 | 11.3565 |
| test_5 | 5 | 11.2750 | 10.9309 | 11.3619 | 11.2423 |
| test_6 | 5 | 11.8301 | 11.3415 | 11.8191 | 11.6470 |
| test_7 | 5 | 10.2355 | 9.9763 | 10.2281 | 10.1744 |
| test_8 | 5 | 10.8869 | 10.5512 | 11.0253 | 10.8077 |
| test_9 | 5 | 11.9017 | 11.3320 | 11.6637 | 11.7206 |
| test_10 | 5 | 11.3114 | 10.7545 | 10.9327 | 11.0095 |

