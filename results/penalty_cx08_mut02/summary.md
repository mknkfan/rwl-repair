# Repair, the wall, and two penalties

50 matched sets (10 instances x 5 seeds), population 200, 150 generations, 30000 evaluations per run.

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
| `baseline` | 50/50 | 10.9677 | 10.9990 | 9.9233 | 13.0 | 0.2482 |
| `repair` | 50/50 | 10.6266 | 10.8033 | 9.7838 | 79.6 | 0.5582 |
| `penalty_cvd` | 50/50 | 11.0121 | 11.1050 | 9.8789 | 23.4 | 0.2812 |
| `penalty_count` | 50/50 | 11.0055 | 11.1309 | 9.9794 | 23.3 | 0.2866 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 10.9677 | 10.6266 | -0.3411 | [-0.4061, -0.2783] | 0 | 50 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 11.0121 | 10.6266 | -0.3855 | [-0.4566, -0.3216] | 1 | 49 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 11.0055 | 10.6266 | -0.3789 | [-0.4640, -0.3068] | 2 | 48 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 0.2482 | 0.5582 | -0.3100 | [-0.3318, -0.2877] | 0 | 50 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 0.2812 | 0.5582 | -0.2769 | [-0.2996, -0.2549] | 0 | 50 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 0.2866 | 0.5582 | -0.2716 | [-0.2955, -0.2483] | 0 | 50 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 13.0069 | 79.6237 | 66.6169 | [64.6219, 68.6269] | 50 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 50 | 23.4088 | 79.6237 | 56.2150 | [54.2343, 58.2066] | 50 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 50 | 23.3306 | 79.6237 | 56.2931 | [54.1977, 58.4085] | 50 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 50 | 10.6266 | 10.9677 | 0.3411 | [0.2783, 0.4061] | 50 | 0 | 0.0000 |
| `penalty_cvd` | 50 | 11.0121 | 10.9677 | -0.0444 | [-0.1409, 0.0433] | 23 | 27 | 0.3586 |
| `penalty_count` | 50 | 11.0055 | 10.9677 | -0.0378 | [-0.1442, 0.0605] | 21 | 29 | 0.4826 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 50 | 11.0613 | 10.9677 | -0.0937 | [-0.1722, -0.0103] | 13 | 37 | 0.0300 |
| `penalty_cvd` | 50 | 11.1945 | 10.9677 | -0.2269 | [-0.3424, -0.1243] | 12 | 38 | 0.0000 |
| `penalty_count` | 50 | 11.1764 | 10.9677 | -0.2087 | [-0.3220, -0.1026] | 13 | 37 | 0.0003 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 150.0 | 12.9 | 10.9677 |
| `repair` | 31.6 | 12.7 | 11.0613 |
| `penalty_cvd` | 83.0 | 12.9 | 11.1945 |
| `penalty_count` | 82.4 | 12.9 | 11.1764 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 50 | 10.9677 | 11.0613 | 0.0937 | [0.0103, 0.1722] | 37 | 13 | 0.0300 | 0.0152 |
| `penalty_cvd` | 50 | 11.1945 | 11.0613 | -0.1332 | [-0.2258, -0.0522] | 18 | 32 | 0.0020 | 0.0038 |
| `penalty_count` | 50 | 11.1764 | 11.0613 | -0.1151 | [-0.2034, -0.0389] | 17 | 33 | 0.0041 | 0.0042 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 5 | 10.1634 | 9.8606 | 10.1749 | 10.2469 |
| test_2 | 5 | 11.4110 | 11.0442 | 11.3516 | 11.3704 |
| test_3 | 5 | 10.2095 | 9.8573 | 10.2953 | 10.2083 |
| test_4 | 5 | 11.1338 | 10.7789 | 11.2967 | 11.3875 |
| test_5 | 5 | 11.2786 | 10.8680 | 11.2077 | 11.1405 |
| test_6 | 5 | 11.8229 | 11.3316 | 11.7822 | 11.7769 |
| test_7 | 5 | 10.4548 | 9.9601 | 10.2673 | 10.2524 |
| test_8 | 5 | 10.7412 | 10.4825 | 10.7386 | 10.8018 |
| test_9 | 5 | 11.5087 | 11.2926 | 11.9242 | 11.7476 |
| test_10 | 5 | 10.9527 | 10.7902 | 11.0826 | 11.1224 |

