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
| `baseline` | 200/200 | 9.1097 | 9.2397 | 8.1407 | 43.9 | 0.3227 |
| `repair` | 200/200 | 8.9395 | 9.1188 | 8.0638 | 149.4 | 0.6472 |
| `penalty_cvd` | 200/200 | 9.0918 | 9.1940 | 8.0838 | 69.7 | 0.3391 |
| `penalty_count` | 200/200 | 9.0929 | 9.2061 | 8.1039 | 69.1 | 0.3430 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 9.1097 | 8.9395 | -0.1702 | [-0.1958, -0.1462] | 30 | 170 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 9.0918 | 8.9395 | -0.1523 | [-0.1796, -0.1260] | 42 | 158 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 9.0929 | 8.9395 | -0.1534 | [-0.1772, -0.1294] | 29 | 171 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.3227 | 0.6472 | -0.3245 | [-0.3339, -0.3149] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.3391 | 0.6472 | -0.3082 | [-0.3176, -0.2985] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.3430 | 0.6472 | -0.3042 | [-0.3140, -0.2944] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 43.8811 | 149.4199 | 105.5388 | [101.4451, 109.6876] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 69.6983 | 149.4199 | 79.7216 | [75.2240, 84.2157] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 69.1452 | 149.4199 | 80.2747 | [75.8456, 84.8204] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 8.9395 | 9.1097 | 0.1702 | [0.1462, 0.1958] | 170 | 30 | 0.0000 |
| `penalty_cvd` | 200 | 9.0918 | 9.1097 | 0.0179 | [-0.0084, 0.0445] | 109 | 91 | 0.1860 |
| `penalty_count` | 200 | 9.0929 | 9.1097 | 0.0168 | [-0.0085, 0.0433] | 111 | 89 | 0.2026 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 8.9913 | 9.1101 | 0.1188 | [0.0928, 0.1463] | 143 | 57 | 0.0000 |
| `penalty_cvd` | 200 | 9.1302 | 9.1101 | -0.0201 | [-0.0478, 0.0082] | 88 | 112 | 0.1583 |
| `penalty_count` | 200 | 9.1290 | 9.1101 | -0.0189 | [-0.0456, 0.0088] | 88 | 112 | 0.1713 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 295.9 | 42.9 | 9.1101 |
| `repair` | 90.5 | 42.8 | 8.9913 |
| `penalty_cvd` | 186.2 | 42.9 | 9.1302 |
| `penalty_count` | 187.0 | 42.9 | 9.1290 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 9.1101 | 8.9913 | -0.1188 | [-0.1463, -0.0928] | 57 | 143 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 9.1302 | 8.9913 | -0.1389 | [-0.1675, -0.1117] | 44 | 156 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 9.1290 | 8.9913 | -0.1377 | [-0.1630, -0.1122] | 44 | 156 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 8.6348 | 8.4753 | 8.6487 | 8.6263 |
| test_2 | 20 | 9.8098 | 9.5884 | 9.8391 | 9.7271 |
| test_3 | 20 | 8.6506 | 8.4942 | 8.5983 | 8.6394 |
| test_4 | 20 | 9.2005 | 9.0368 | 9.1315 | 9.1579 |
| test_5 | 20 | 9.5452 | 9.3998 | 9.5665 | 9.5342 |
| test_6 | 20 | 9.3872 | 9.2721 | 9.3709 | 9.4325 |
| test_7 | 20 | 8.3994 | 8.2219 | 8.3784 | 8.3619 |
| test_8 | 20 | 8.5552 | 8.4787 | 8.5942 | 8.5940 |
| test_9 | 20 | 9.4788 | 9.2393 | 9.4156 | 9.4429 |
| test_10 | 20 | 9.4356 | 9.1888 | 9.3750 | 9.4129 |

