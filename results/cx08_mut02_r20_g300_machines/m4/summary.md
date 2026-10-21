# Repair, the wall, and two penalties

200 matched sets (10 instances x 20 seeds, 4 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each set is one (instance, seed). All four arms are handed the identical rejection-sampled initial population, the same seed and the same operator rates (tournament k=3, uniform crossover p=0.8, mutation p=0.2 per machine, elitism 10%, sigma=0.1), and differ only in the scoring function. Differences are taken within a set and signed so **positive means the arm beat `repair`**.

## The four fitness functions

All four minimise the same thing -- `T(x)`, the cycle time of layout `x` in
seconds, as defined by `CycleTimeEvaluator`. They differ only in what happens
when `x` is not feasible.

`T_inf` = 1000 s is a constant above every attainable cycle time, so a
feasible layout always outranks an infeasible one and **cycle time stays the
primary objective in every arm**.

`CVD(x) = sum_i max(0, g_i(x))` over the geometric inequality constraints
`g_i(x) <= 0`, in metres. For 4 machines there are 26 of them:

| count | constraint | `g_i(x)` |
| ---: | --- | --- |
| 16 | footprint inside the cell | signed excursion of the footprint past each of the four walls |
| 4 | clear of the robot column | `keep_out_radius - distance(footprint, base)` |
| 6 | machines mutually disjoint | `overlap_depth(i, j)`, the minimum translation separating the pair with the clearance included |

`CVD(x) = 0` exactly when `layout_is_placeable(x)` is true; this is checked
rather than assumed. `N(x)` is the number of machines caught in at least one
overlap, an integer in `0..4`.

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
| `baseline` | 200/200 | 7.2851 | 7.2658 | 6.5402 | 22.2 | 0.4638 |
| `repair` | 200/200 | 7.1783 | 7.1961 | 6.5073 | 88.6 | 0.7462 |
| `penalty_cvd` | 200/200 | 7.2858 | 7.2755 | 6.5377 | 31.4 | 0.4773 |
| `penalty_count` | 200/200 | 7.2885 | 7.2649 | 6.5019 | 30.6 | 0.4787 |

`feasible evaluations` is the share of all fitness calls that landed on a genuinely feasible layout -- the one feasibility measure defined identically in all four arms, since the population-share trace in `ga.py` is `isfinite(fitness)` and a penalty makes that identically 1.0.

## Paired against `repair`, equal generations

**cycle time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 7.2851 | 7.1783 | -0.1068 | [-0.1231, -0.0908] | 36 | 164 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 7.2858 | 7.1783 | -0.1075 | [-0.1267, -0.0890] | 38 | 162 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 7.2885 | 7.1783 | -0.1102 | [-0.1288, -0.0921] | 42 | 158 | 0.0000 | 0.0000 |

**feasible evaluation share**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 0.4638 | 0.7462 | -0.2824 | [-0.2932, -0.2715] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 0.4773 | 0.7462 | -0.2689 | [-0.2796, -0.2583] | 0 | 200 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 0.4787 | 0.7462 | -0.2675 | [-0.2781, -0.2568] | 0 | 200 | 0.0000 | 0.0000 |

**CPU time (s)**

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 22.2493 | 88.6123 | 66.3630 | [64.9036, 67.7997] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 31.4113 | 88.6123 | 57.2010 | [55.7579, 58.6555] | 200 | 0 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 30.6316 | 88.6123 | 57.9807 | [56.5704, 59.3882] | 200 | 0 | 0.0000 | 0.0000 |

## Paired against `baseline`: does a graded penalty beat the wall?

The comparison above asks whether anything beats repair. This one asks the question the penalty arms were actually built to answer -- whether putting a slope on the infeasible plateau beats leaving it flat. Signed so positive means the arm beat `baseline`.

**equal generations**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 7.1783 | 7.2851 | 0.1068 | [0.0908, 0.1231] | 164 | 36 | 0.0000 |
| `penalty_cvd` | 200 | 7.2858 | 7.2851 | -0.0007 | [-0.0200, 0.0185] | 106 | 94 | 0.9432 |
| `penalty_count` | 200 | 7.2885 | 7.2851 | -0.0033 | [-0.0216, 0.0144] | 101 | 99 | 0.7169 |

**equal CPU**

| arm | pairs | arm | baseline | mean gain (s) | 95% CI | arm wins | baseline wins | perm p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: |
| `repair` | 200 | 7.2141 | 7.2852 | 0.0711 | [0.0546, 0.0876] | 141 | 59 | 0.0000 |
| `penalty_cvd` | 200 | 7.2963 | 7.2852 | -0.0111 | [-0.0304, 0.0081] | 103 | 97 | 0.2644 |
| `penalty_count` | 200 | 7.2978 | 7.2852 | -0.0126 | [-0.0312, 0.0055] | 98 | 102 | 0.1779 |

## Paired against `repair`, equal CPU

For each (instance, seed) the budget is the smallest total CPU any of the four arms spent on that set -- a budget every arm demonstrably had -- and each arm is read at the last generation it finished inside it. No arm is rerun.

| arm | generations at budget | realised CPU (s) | mean cycle time (s) |
| --- | ---: | ---: | ---: |
| `baseline` | 298.5 | 22.0 | 7.2852 |
| `repair` | 74.9 | 21.9 | 7.2141 |
| `penalty_cvd` | 206.2 | 22.1 | 7.2963 |
| `penalty_count` | 210.6 | 22.0 | 7.2978 |

| arm | pairs | arm | repair | mean gain | 95% CI | arm wins | repair wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `baseline` | 200 | 7.2852 | 7.2141 | -0.0711 | [-0.0876, -0.0546] | 59 | 141 | 0.0000 | 0.0000 |
| `penalty_cvd` | 200 | 7.2963 | 7.2141 | -0.0822 | [-0.1022, -0.0631] | 55 | 145 | 0.0000 | 0.0000 |
| `penalty_count` | 200 | 7.2978 | 7.2141 | -0.0837 | [-0.1026, -0.0655] | 56 | 144 | 0.0000 | 0.0000 |

## Per instance, mean cycle time (s), equal generations

| instance | sets | `baseline` | `repair` | `penalty_cvd` | `penalty_count` |
| --- | ---: | ---: | ---: | ---: | ---: |
| test_1 | 20 | 6.7772 | 6.7311 | 6.8263 | 6.8141 |
| test_2 | 20 | 7.9857 | 7.8173 | 7.9738 | 7.9622 |
| test_3 | 20 | 6.7563 | 6.6163 | 6.7020 | 6.7162 |
| test_4 | 20 | 7.1872 | 7.0622 | 7.1947 | 7.1708 |
| test_5 | 20 | 7.6208 | 7.5792 | 7.6165 | 7.6406 |
| test_6 | 20 | 7.3693 | 7.2878 | 7.4157 | 7.4049 |
| test_7 | 20 | 6.9602 | 6.8062 | 6.9381 | 6.9494 |
| test_8 | 20 | 6.9915 | 6.9019 | 7.0222 | 6.9867 |
| test_9 | 20 | 7.6414 | 7.5145 | 7.5841 | 7.6073 |
| test_10 | 20 | 7.5620 | 7.4666 | 7.5850 | 7.6326 |

