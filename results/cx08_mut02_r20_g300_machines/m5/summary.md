# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds, 5 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.8, mutation p=0.2 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = 1000 s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For 5 machines there are 35 of them:

| count | constraint | `g_i(x)` |
| ---: | --- | --- |
| 20 | footprint inside the cell | signed excursion of the footprint past each of the four walls |
| 5 | clear of the robot column | `keep_out_radius - distance(footprint, base)` |
| 10 | machines mutually disjoint | `overlap_depth(i, j)`, the minimum translation separating the pair with the clearance included |

`CVD(x) = 0` exactly when `layout_is_placeable(x)` is true; this is checked
rather than assumed. `N(x)` is the number of machines caught in at least one
overlap, an integer in `0..5`.

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
| `baseline` | 200/200 | 9.1097 | 9.2397 | 8.1407 | 23.1 | 0.3227 |
| `repair` | 200/200 | 8.9395 | 9.1188 | 8.0638 | 130.2 | 0.6472 |
| `penalty_cvd` | 200/200 | 9.0918 | 9.1940 | 8.0838 | 37.8 | 0.3391 |
| `penalty_count` | 200/200 | 9.0929 | 9.2061 | 8.1039 | 40.5 | 0.3430 |

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
| `baseline` | 200 | 23.0948 | 130.2476 | 107.1527 | [105.3875, 108.9339] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 37.8393 | 130.2476 | 92.4083 | [90.6375, 94.2038] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 40.5238 | 130.2476 | 89.7238 | [87.8791, 91.5791] | 200 | 0 | 0.0000 | 0.0000 |

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
| `repair` | 200 | 9.0437 | 9.1097 | 0.0661 | [0.0402, 0.0932] | 115 | 85 | 0.0000 |
| `penalty_cvd` | 200 | 9.1326 | 9.1097 | -0.0229 | [-0.0503, 0.0057] | 84 | 116 | 0.1076 |
| `penalty_count` | 200 | 9.1332 | 9.1097 | -0.0234 | [-0.0503, 0.0044] | 90 | 110 | 0.0910 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 299.9 | 23.0 | 9.1097 |
| `repair` | 58.6 | 22.9 | 9.0437 |
| `penalty_cvd` | 178.3 | 23.0 | 9.1326 |
| `penalty_count` | 165.3 | 23.0 | 9.1332 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 9.1097 | 9.0437 | -0.0661 | [-0.0932, -0.0402] | 85 | 115 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 9.1326 | 9.0437 | -0.0889 | [-0.1173, -0.0617] | 69 | 131 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 9.1332 | 9.0437 | -0.0895 | [-0.1145, -0.0641] | 62 | 138 | 0.0000 | 0.0000 |

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

