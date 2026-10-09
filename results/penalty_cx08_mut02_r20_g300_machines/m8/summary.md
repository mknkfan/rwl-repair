# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds, 8 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.8, mutation p=0.2 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = 1000 s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For 8 machines there are 68 of them:

| count | constraint | `g_i(x)` |
| ---: | --- | --- |
| 32 | footprint inside the cell | signed excursion of the footprint past each of the four walls |
| 8 | clear of the robot column | `keep_out_radius - distance(footprint, base)` |
| 28 | machines mutually disjoint | `overlap_depth(i, j)`, the minimum translation separating the pair with the clearance included |

`CVD(x) = 0` exactly when `layout_is_placeable(x)` is true; this is checked
rather than assumed. `N(x)` is the number of machines caught in at least one
overlap, an integer in `0..8`.

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
| `baseline` | 200/200 | 15.4863 | 15.3553 | 13.9751 | 27.4 | 0.1360 |
| `repair` | 200/200 | 14.5889 | 14.4724 | 13.3923 | 242.0 | 0.4030 |
| `penalty_cvd` | 200/200 | 15.4390 | 15.3086 | 13.7232 | 63.2 | 0.1676 |
| `penalty_count` | 200/200 | 15.4399 | 15.3480 | 13.8622 | 63.6 | 0.1669 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 15.4863 | 14.5889 | -0.8974 | [-0.9770, -0.8214] | 6 | 194 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 15.4390 | 14.5889 | -0.8501 | [-0.9280, -0.7749] | 6 | 194 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 15.4399 | 14.5889 | -0.8510 | [-0.9210, -0.7824] | 5 | 195 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.1360 | 0.4030 | -0.2670 | [-0.2782, -0.2558] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.1676 | 0.4030 | -0.2353 | [-0.2462, -0.2245] | 1 | 199 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.1669 | 0.4030 | -0.2361 | [-0.2468, -0.2253] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 27.3577 | 242.0390 | 214.6813 | [212.0843, 217.3143] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 63.2277 | 242.0390 | 178.8112 | [176.2936, 181.3919] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 63.5609 | 242.0390 | 178.4781 | [176.0082, 181.0263] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 14.5889 | 15.4863 | 0.8974 | [0.8214, 0.9770] | 194 | 6 | 0.0000 |
| `penalty_cvd` | 200 | 15.4390 | 15.4863 | 0.0474 | [-0.0382, 0.1349] | 99 | 101 | 0.2866 |
| `penalty_count` | 200 | 15.4399 | 15.4863 | 0.0464 | [-0.0388, 0.1335] | 108 | 92 | 0.2901 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 15.3901 | 15.4863 | 0.0962 | [0.0164, 0.1819] | 101 | 99 | 0.0238 |
| `penalty_cvd` | 200 | 15.8398 | 15.4863 | -0.3535 | [-0.4428, -0.2621] | 51 | 149 | 0.0000 |
| `penalty_count` | 200 | 15.8586 | 15.4863 | -0.3723 | [-0.4644, -0.2791] | 48 | 152 | 0.0000 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 300.0 | 27.3 | 15.4863 |
| `repair` | 45.5 | 27.0 | 15.3901 |
| `penalty_cvd` | 127.2 | 27.3 | 15.8398 |
| `penalty_count` | 126.9 | 27.2 | 15.8586 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 15.4863 | 15.3901 | -0.0962 | [-0.1819, -0.0164] | 99 | 101 | 0.0238 | 0.3375 |
| `penalty_cvd` | 200 | 15.8398 | 15.3901 | -0.4497 | [-0.5247, -0.3754] | 37 | 163 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 15.8586 | 15.3901 | -0.4685 | [-0.5368, -0.4019] | 24 | 176 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 15.5593 | 14.7322 | 15.4824 | 15.5644 |
| test_2 | 20 | 16.2399 | 15.2293 | 15.9261 | 16.1112 |
| test_3 | 20 | 14.6532 | 13.9727 | 14.8400 | 14.5618 |
| test_4 | 20 | 14.5980 | 13.7752 | 14.7208 | 14.5036 |
| test_5 | 20 | 15.3434 | 14.4150 | 15.3297 | 15.2962 |
| test_6 | 20 | 15.6589 | 14.7863 | 15.7435 | 15.7092 |
| test_7 | 20 | 14.7813 | 13.9884 | 14.7013 | 14.7099 |
| test_8 | 20 | 15.2140 | 14.1997 | 14.9743 | 15.3095 |
| test_9 | 20 | 15.5660 | 14.6095 | 15.4644 | 15.5179 |
| test_10 | 20 | 17.2492 | 16.1805 | 17.2072 | 17.1154 |

