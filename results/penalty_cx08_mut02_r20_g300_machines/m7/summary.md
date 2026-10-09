# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds, 7 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.8, mutation p=0.2 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = 1000 s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For 7 machines there are 56 of them:

| count | constraint | `g_i(x)` |
| ---: | --- | --- |
| 28 | footprint inside the cell | signed excursion of the footprint past each of the four walls |
| 7 | clear of the robot column | `keep_out_radius - distance(footprint, base)` |
| 21 | machines mutually disjoint | `overlap_depth(i, j)`, the minimum translation separating the pair with the clearance included |

`CVD(x) = 0` exactly when `layout_is_placeable(x)` is true; this is checked
rather than assumed. `N(x)` is the number of machines caught in at least one
overlap, an integer in `0..7`.

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
| `baseline` | 200/200 | 13.0834 | 13.0284 | 11.7065 | 27.7 | 0.1778 |
| `repair` | 200/200 | 12.4890 | 12.5397 | 11.4878 | 210.4 | 0.4885 |
| `penalty_cvd` | 200/200 | 13.0747 | 13.0999 | 11.7753 | 56.0 | 0.2047 |
| `penalty_count` | 200/200 | 13.0863 | 13.0538 | 11.7737 | 55.7 | 0.2118 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 13.0834 | 12.4890 | -0.5943 | [-0.6542, -0.5372] | 9 | 191 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 13.0747 | 12.4890 | -0.5857 | [-0.6454, -0.5275] | 8 | 192 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 13.0863 | 12.4890 | -0.5972 | [-0.6582, -0.5384] | 6 | 194 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.1778 | 0.4885 | -0.3107 | [-0.3228, -0.2984] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.2047 | 0.4885 | -0.2838 | [-0.2953, -0.2720] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.2118 | 0.4885 | -0.2767 | [-0.2886, -0.2644] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 27.6606 | 210.4245 | 182.7639 | [180.3886, 185.2103] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 56.0402 | 210.4245 | 154.3843 | [152.2388, 156.6022] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 55.7078 | 210.4245 | 154.7167 | [152.1992, 157.3088] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 12.4890 | 13.0834 | 0.5943 | [0.5372, 0.6542] | 191 | 9 | 0.0000 |
| `penalty_cvd` | 200 | 13.0747 | 13.0834 | 0.0087 | [-0.0497, 0.0693] | 99 | 101 | 0.7766 |
| `penalty_count` | 200 | 13.0863 | 13.0834 | -0.0029 | [-0.0596, 0.0533] | 102 | 98 | 0.9186 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 12.9293 | 13.0834 | 0.1541 | [0.0920, 0.2194] | 121 | 79 | 0.0000 |
| `penalty_cvd` | 200 | 13.2809 | 13.0834 | -0.1975 | [-0.2598, -0.1322] | 55 | 145 | 0.0000 |
| `penalty_count` | 200 | 13.3083 | 13.0834 | -0.2249 | [-0.2856, -0.1664] | 49 | 151 | 0.0000 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 300.0 | 27.6 | 13.0834 |
| `repair` | 50.1 | 27.3 | 12.9293 |
| `penalty_cvd` | 145.3 | 27.6 | 13.2809 |
| `penalty_count` | 144.5 | 27.6 | 13.3083 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 13.0834 | 12.9293 | -0.1541 | [-0.2194, -0.0920] | 79 | 121 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 13.2809 | 12.9293 | -0.3516 | [-0.4124, -0.2926] | 32 | 168 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 13.3083 | 12.9293 | -0.3790 | [-0.4408, -0.3200] | 30 | 170 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 13.3791 | 12.8741 | 13.4497 | 13.4235 |
| test_2 | 20 | 13.6946 | 12.9909 | 13.5657 | 13.6922 |
| test_3 | 20 | 12.0640 | 11.7027 | 12.1582 | 12.1015 |
| test_4 | 20 | 12.8032 | 12.1857 | 12.8685 | 12.8655 |
| test_5 | 20 | 12.7986 | 12.3621 | 12.8521 | 12.9898 |
| test_6 | 20 | 13.9380 | 13.1271 | 13.8566 | 13.7295 |
| test_7 | 20 | 12.6033 | 12.0242 | 12.4746 | 12.5847 |
| test_8 | 20 | 12.9323 | 12.2569 | 12.8595 | 12.9015 |
| test_9 | 20 | 13.3903 | 12.6938 | 13.4161 | 13.3699 |
| test_10 | 20 | 13.2301 | 12.6727 | 13.2459 | 13.2045 |

