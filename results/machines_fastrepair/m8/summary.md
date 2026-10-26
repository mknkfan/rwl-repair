# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds), population 200, 300 generations, 60000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.8, mutation p=0.2 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

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
| `baseline` | 200/200 | 15.4943 | 15.3575 | 13.9751 | 45.1 | 0.1357 |
| `repair` | 200/200 | 14.5948 | 14.4542 | 13.3923 | 214.4 | 0.4041 |
| `penalty_cvd` | 200/200 | 15.4367 | 15.3334 | 13.7232 | 110.4 | 0.1678 |
| `penalty_count` | 200/200 | 15.4552 | 15.3448 | 13.8622 | 107.8 | 0.1679 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 15.4943 | 14.5948 | -0.8996 | [-0.9783, -0.8240] | 6 | 194 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 15.4367 | 14.5948 | -0.8419 | [-0.9187, -0.7679] | 7 | 193 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 15.4552 | 14.5948 | -0.8604 | [-0.9362, -0.7870] | 5 | 195 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.1357 | 0.4041 | -0.2684 | [-0.2791, -0.2575] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.1678 | 0.4041 | -0.2363 | [-0.2468, -0.2257] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.1679 | 0.4041 | -0.2362 | [-0.2465, -0.2257] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 45.0954 | 214.4493 | 169.3539 | [164.7619, 173.9375] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 110.3633 | 214.4493 | 104.0860 | [99.2829, 108.8607] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 107.8043 | 214.4493 | 106.6450 | [101.8817, 111.3796] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 14.5948 | 15.4943 | 0.8996 | [0.8240, 0.9783] | 194 | 6 | 0.0000 |
| `penalty_cvd` | 200 | 15.4367 | 15.4943 | 0.0576 | [-0.0274, 0.1439] | 106 | 94 | 0.1916 |
| `penalty_count` | 200 | 15.4552 | 15.4943 | 0.0392 | [-0.0424, 0.1221] | 109 | 91 | 0.3564 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 15.0719 | 15.4943 | 0.4224 | [0.3474, 0.5017] | 159 | 41 | 0.0000 |
| `penalty_cvd` | 200 | 15.9052 | 15.4943 | -0.4109 | [-0.4992, -0.3206] | 35 | 165 | 0.0000 |
| `penalty_count` | 200 | 15.9004 | 15.4943 | -0.4061 | [-0.4978, -0.3139] | 45 | 155 | 0.0000 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 300.0 | 45.0 | 15.4943 |
| `repair` | 71.2 | 44.7 | 15.0719 |
| `penalty_cvd` | 123.1 | 44.9 | 15.9052 |
| `penalty_count` | 124.7 | 44.9 | 15.9004 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 15.4943 | 15.0719 | -0.4224 | [-0.5017, -0.3474] | 41 | 159 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 15.9052 | 15.0719 | -0.8333 | [-0.9087, -0.7604] | 6 | 194 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 15.9004 | 15.0719 | -0.8285 | [-0.9033, -0.7566] | 4 | 196 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 15.5646 | 14.7081 | 15.4913 | 15.6224 |
| test_2 | 20 | 16.1671 | 15.2486 | 15.9668 | 15.9765 |
| test_3 | 20 | 14.6532 | 13.9798 | 14.8377 | 14.5623 |
| test_4 | 20 | 14.6101 | 13.7971 | 14.5531 | 14.5181 |
| test_5 | 20 | 15.3434 | 14.3945 | 15.3217 | 15.2754 |
| test_6 | 20 | 15.6563 | 14.7810 | 15.8394 | 15.7600 |
| test_7 | 20 | 14.7813 | 13.9884 | 14.7013 | 14.7099 |
| test_8 | 20 | 15.2140 | 14.1997 | 14.9585 | 15.3271 |
| test_9 | 20 | 15.5682 | 14.6151 | 15.4573 | 15.5071 |
| test_10 | 20 | 17.3851 | 16.2355 | 17.2399 | 17.2931 |

