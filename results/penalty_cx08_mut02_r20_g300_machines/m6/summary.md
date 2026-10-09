# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds, 6 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.8, mutation p=0.2 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = 1000 s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For 6 machines there are 45 of them:

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
| `baseline` | 200/200 | 10.9020 | 10.9486 | 9.7588 | 23.3 | 0.2287 |
| `repair` | 200/200 | 10.5886 | 10.7393 | 9.6961 | 158.9 | 0.5518 |
| `penalty_cvd` | 200/200 | 10.9128 | 11.0327 | 9.7502 | 44.3 | 0.2609 |
| `penalty_count` | 200/200 | 10.9102 | 11.0202 | 9.7662 | 43.8 | 0.2633 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 10.9020 | 10.5886 | -0.3135 | [-0.3497, -0.2779] | 14 | 186 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 10.9128 | 10.5886 | -0.3242 | [-0.3606, -0.2896] | 14 | 186 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 10.9102 | 10.5886 | -0.3216 | [-0.3598, -0.2852] | 17 | 183 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.2287 | 0.5518 | -0.3231 | [-0.3349, -0.3112] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.2609 | 0.5518 | -0.2909 | [-0.3021, -0.2797] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.2633 | 0.5518 | -0.2885 | [-0.2995, -0.2774] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 23.2859 | 158.8774 | 135.5916 | [133.4626, 137.7220] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 44.3270 | 158.8774 | 114.5505 | [112.5130, 116.5939] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 43.8159 | 158.8774 | 115.0616 | [112.9545, 117.1741] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 10.5886 | 10.9020 | 0.3135 | [0.2779, 0.3497] | 186 | 14 | 0.0000 |
| `penalty_cvd` | 200 | 10.9128 | 10.9020 | -0.0107 | [-0.0512, 0.0308] | 98 | 102 | 0.6089 |
| `penalty_count` | 200 | 10.9102 | 10.9020 | -0.0082 | [-0.0534, 0.0371] | 93 | 107 | 0.7247 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 10.8434 | 10.9020 | 0.0586 | [0.0177, 0.1008] | 104 | 96 | 0.0059 |
| `penalty_cvd` | 200 | 11.0140 | 10.9020 | -0.1120 | [-0.1566, -0.0671] | 62 | 138 | 0.0000 |
| `penalty_count` | 200 | 11.0287 | 10.9020 | -0.1266 | [-0.1768, -0.0769] | 67 | 133 | 0.0000 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 300.0 | 23.2 | 10.9020 |
| `repair` | 52.5 | 23.0 | 10.8434 |
| `penalty_cvd` | 152.7 | 23.2 | 11.0140 |
| `penalty_count` | 154.4 | 23.2 | 11.0287 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 10.9020 | 10.8434 | -0.0586 | [-0.1008, -0.0177] | 96 | 104 | 0.0059 | 0.0963 |
| `penalty_cvd` | 200 | 11.0140 | 10.8434 | -0.1706 | [-0.2092, -0.1336] | 49 | 151 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 11.0287 | 10.8434 | -0.1852 | [-0.2258, -0.1464] | 48 | 152 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 10.1429 | 9.8597 | 10.2069 | 10.1347 |
| test_2 | 20 | 11.2974 | 11.0066 | 11.2924 | 11.2857 |
| test_3 | 20 | 10.1629 | 9.8660 | 10.0783 | 10.1452 |
| test_4 | 20 | 11.0809 | 10.7760 | 11.1904 | 11.2063 |
| test_5 | 20 | 11.1722 | 10.8492 | 11.2005 | 11.1955 |
| test_6 | 20 | 11.6626 | 11.2979 | 11.6589 | 11.6734 |
| test_7 | 20 | 10.2281 | 9.9327 | 10.1270 | 10.1278 |
| test_8 | 20 | 10.7334 | 10.3902 | 10.7748 | 10.7293 |
| test_9 | 20 | 11.5686 | 11.1767 | 11.6003 | 11.5576 |
| test_10 | 20 | 10.9715 | 10.7307 | 10.9982 | 11.0467 |

