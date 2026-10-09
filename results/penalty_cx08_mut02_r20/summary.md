# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds), population 200, 150 generations, 30000 evaluations per run.

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
| `baseline` | 200/200 | 11.0128 | 11.0870 | 9.8783 | 13.4 | 0.2563 |
| `repair` | 200/200 | 10.6242 | 10.7762 | 9.7097 | 78.5 | 0.5552 |
| `penalty_cvd` | 200/200 | 11.0095 | 11.1060 | 9.8642 | 23.8 | 0.2844 |
| `penalty_count` | 200/200 | 11.0290 | 11.1296 | 9.8120 | 23.6 | 0.2871 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 11.0128 | 10.6242 | -0.3886 | [-0.4263, -0.3522] | 4 | 196 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 11.0095 | 10.6242 | -0.3853 | [-0.4216, -0.3505] | 8 | 192 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 11.0290 | 10.6242 | -0.4048 | [-0.4432, -0.3680] | 6 | 194 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.2563 | 0.5552 | -0.2989 | [-0.3096, -0.2882] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.2844 | 0.5552 | -0.2709 | [-0.2809, -0.2608] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.2871 | 0.5552 | -0.2681 | [-0.2781, -0.2581] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 13.3653 | 78.4921 | 65.1268 | [64.0576, 66.1945] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 23.7534 | 78.4921 | 54.7387 | [53.7225, 55.7438] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 23.5552 | 78.4921 | 54.9369 | [53.8784, 55.9823] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 10.6242 | 11.0128 | 0.3886 | [0.3522, 0.4263] | 196 | 4 | 0.0000 |
| `penalty_cvd` | 200 | 11.0095 | 11.0128 | 0.0033 | [-0.0392, 0.0455] | 102 | 98 | 0.8800 |
| `penalty_count` | 200 | 11.0290 | 11.0128 | -0.0163 | [-0.0634, 0.0307] | 92 | 108 | 0.4971 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 11.0568 | 11.0128 | -0.0440 | [-0.0886, 0.0024] | 75 | 125 | 0.0577 |
| `penalty_cvd` | 200 | 11.1893 | 11.0128 | -0.1765 | [-0.2257, -0.1282] | 55 | 145 | 0.0000 |
| `penalty_count` | 200 | 11.2028 | 11.0128 | -0.1900 | [-0.2399, -0.1403] | 54 | 146 | 0.0000 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 150.0 | 13.3 | 11.0128 |
| `repair` | 32.7 | 13.1 | 11.0568 |
| `penalty_cvd` | 83.9 | 13.3 | 11.1893 |
| `penalty_count` | 84.3 | 13.3 | 11.2028 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 11.0128 | 11.0568 | 0.0440 | [-0.0024, 0.0886] | 125 | 75 | 0.0577 | 0.0095 |
| `penalty_cvd` | 200 | 11.1893 | 11.0568 | -0.1325 | [-0.1755, -0.0923] | 78 | 122 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 11.2028 | 11.0568 | -0.1460 | [-0.1859, -0.1077] | 63 | 137 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 10.2510 | 9.8910 | 10.3079 | 10.2402 |
| test_2 | 20 | 11.4045 | 11.0596 | 11.3833 | 11.3924 |
| test_3 | 20 | 10.2585 | 9.8815 | 10.1800 | 10.2249 |
| test_4 | 20 | 11.1930 | 10.8359 | 11.2626 | 11.3292 |
| test_5 | 20 | 11.2785 | 10.8828 | 11.2752 | 11.3365 |
| test_6 | 20 | 11.7778 | 11.3324 | 11.7785 | 11.8004 |
| test_7 | 20 | 10.3291 | 9.9569 | 10.2011 | 10.2250 |
| test_8 | 20 | 10.8764 | 10.4209 | 10.8759 | 10.8669 |
| test_9 | 20 | 11.6695 | 11.2169 | 11.7310 | 11.7043 |
| test_10 | 20 | 11.0893 | 10.7642 | 11.0991 | 11.1707 |

