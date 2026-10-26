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
| `baseline` | 200/200 | 13.0767 | 13.0284 | 11.7065 | 45.9 | 0.1776 |
| `repair` | 200/200 | 12.4876 | 12.5397 | 11.4878 | 196.6 | 0.4883 |
| `penalty_cvd` | 200/200 | 13.0820 | 13.0999 | 11.7753 | 93.6 | 0.2049 |
| `penalty_count` | 200/200 | 13.0851 | 13.0538 | 11.7737 | 92.3 | 0.2123 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 13.0767 | 12.4876 | -0.5891 | [-0.6471, -0.5341] | 9 | 191 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 13.0820 | 12.4876 | -0.5944 | [-0.6549, -0.5360] | 8 | 192 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 13.0851 | 12.4876 | -0.5975 | [-0.6590, -0.5384] | 6 | 194 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.1776 | 0.4883 | -0.3108 | [-0.3228, -0.2985] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.2049 | 0.4883 | -0.2834 | [-0.2950, -0.2716] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.2123 | 0.4883 | -0.2761 | [-0.2880, -0.2638] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 45.8664 | 196.5917 | 150.7252 | [145.8344, 155.5920] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 93.6103 | 196.5917 | 102.9814 | [97.5740, 108.3709] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 92.3397 | 196.5917 | 104.2520 | [98.8547, 109.5988] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 12.4876 | 13.0767 | 0.5891 | [0.5341, 0.6471] | 191 | 9 | 0.0000 |
| `penalty_cvd` | 200 | 13.0820 | 13.0767 | -0.0053 | [-0.0618, 0.0514] | 97 | 103 | 0.8553 |
| `penalty_count` | 200 | 13.0851 | 13.0767 | -0.0084 | [-0.0642, 0.0469] | 102 | 98 | 0.7651 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 12.7388 | 13.0771 | 0.3382 | [0.2778, 0.4006] | 169 | 31 | 0.0000 |
| `penalty_cvd` | 200 | 13.2948 | 13.0771 | -0.2178 | [-0.2807, -0.1540] | 53 | 147 | 0.0000 |
| `penalty_count` | 200 | 13.3131 | 13.0771 | -0.2360 | [-0.2944, -0.1792] | 49 | 151 | 0.0000 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 299.9 | 45.7 | 13.0771 |
| `repair` | 78.0 | 45.5 | 12.7388 |
| `penalty_cvd` | 148.8 | 45.7 | 13.2948 |
| `penalty_count` | 148.7 | 45.7 | 13.3131 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 13.0771 | 12.7388 | -0.3382 | [-0.4006, -0.2778] | 31 | 169 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 13.2948 | 12.7388 | -0.5560 | [-0.6203, -0.4932] | 14 | 186 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 13.3131 | 12.7388 | -0.5742 | [-0.6370, -0.5140] | 13 | 187 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 13.3791 | 12.8741 | 13.4497 | 13.4235 |
| test_2 | 20 | 13.6276 | 12.9766 | 13.6388 | 13.6803 |
| test_3 | 20 | 12.0640 | 11.7027 | 12.1582 | 12.1015 |
| test_4 | 20 | 12.8032 | 12.1857 | 12.8685 | 12.8655 |
| test_5 | 20 | 12.7986 | 12.3621 | 12.8521 | 12.9898 |
| test_6 | 20 | 13.9380 | 13.1271 | 13.8566 | 13.7295 |
| test_7 | 20 | 12.6033 | 12.0242 | 12.4746 | 12.5847 |
| test_8 | 20 | 12.9323 | 12.2569 | 12.8595 | 12.9015 |
| test_9 | 20 | 13.3903 | 12.6938 | 13.4161 | 13.3699 |
| test_10 | 20 | 13.2301 | 12.6727 | 13.2459 | 13.2045 |

