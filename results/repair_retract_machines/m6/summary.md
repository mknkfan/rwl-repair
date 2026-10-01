# Does the placement repair help?

100 paired runs (10 instances x 10 seeds, 6 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each pair is one (instance, seed): both arms start from the identical rejection-sampled population (checked: the two runs' SHA-256 fingerprints of it match) and differ only in whether an unplaceable child is projected back onto the placeable set after mutation. Differences are taken within a pair and signed so **positive means repair did better**.


## Verdict

- **Cycle time** -- repair better by 0.3714 s on average (p = 0.0000)
- **Feasible population share** -- repair better by 0.1565 on average (p = 0.0000)
- **Evaluations to first feasible layout** -- no detectable difference (mean +0.0000 evals, p = 1.0000)
- **Gene diversity** -- repair worse by 0.4515 on average (p = 0.0027) (but see the caveat below: this metric is nearly blind to the thing repair would actually collapse)
- **CPU time** -- repair worse by 192.2231 s on average (p = 0.0000)

## Runs that found a feasible layout at all

| both | baseline only | repair only | neither | McNemar p |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0 | 0 | 0 | -- |

Only the discordant pairs carry information here; the exact test conditions on them.


## Paired metrics

| metric | pairs | baseline | repair | mean gain | 95% CI | repair wins | baseline wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `cycle_time` | 100 | 10.9934 | 10.6220 | 0.3714 | [0.3180, 0.4263] | 96 | 4 | 0.0000 | 0.0000 |
| `mean_feasible` | 100 | 0.5929 | 0.7494 | 0.1565 | [0.1456, 0.1673] | 100 | 0 | 0.0000 | 0.0000 |
| `final_feasible` | 100 | 0.6157 | 0.7621 | 0.1463 | [0.1316, 0.1609] | 98 | 2 | 0.0000 | 0.0000 |
| `evals_to_first` | 100 | 200 | 200 | 0 | [0, 0] | 0 | 0 | 1.0000 | -- |
| `mean_diversity` | 100 | 18.4368 | 17.9853 | -0.4515 | [-0.7431, -0.1670] | 43 | 57 | 0.0027 | 0.0062 |
| `cpu_time` | 100 | 273.96 | 466.18 | -192.22 | [-215.01, -169.76] | 5 | 95 | 0.0000 | 0.0000 |

`cycle_time` and `evals_to_first` are taken over the pairs where **both** arms found a feasible layout; the dropped counts are in `pairs_summary.csv`. Every row is signed so that positive favours repair -- for `mean_diversity` that means repair held *more* spread, and for `cpu_time` that it spent *less* time.


> **Caveat on `mean_diversity`.** `population_diversity` averages the per-gene standard deviation over every gene, and the rotation genes span 0-360 while the position genes span about 1.15 m. Rotation spread is therefore two orders of magnitude larger and the average is essentially rotation spread alone -- which is the one gene the repair never touches. Read this row as a check that repair leaves orientation diversity undisturbed, **not** as a measurement of whether it presses machines together onto the placement boundary. Positional collapse needs a position-only trace, and adding one to `ga.py` would change what every run tree reports, so it was measured separately instead: over 3 instances x 4 seeds x 150 generations the mean position spread is 73.8 mm without repair and 71.8 mm with it, a 2.7 % difference, and by the last generation the repaired runs hold slightly *more* spread (36.1 mm vs 34.0 mm). Both arms collapse from 377 mm to about 35 mm, which is the GA converging, not the repair flattening anything. The feared collapse does not happen.


## Per instance

| instance | pairs | both feasible | baseline s | repair s | gain s | repair wins | baseline feas. | repair feas. | baseline feas. share | repair feas. share |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| test_1 | 10 | 10 | 10.1643 | 9.9190 | 0.2453 | 9 | 10 | 10 | 0.5628 | 0.7533 |
| test_2 | 10 | 10 | 11.4685 | 11.0864 | 0.3820 | 10 | 10 | 10 | 0.5889 | 0.7460 |
| test_3 | 10 | 10 | 10.1504 | 9.8726 | 0.2778 | 8 | 10 | 10 | 0.6049 | 0.7530 |
| test_4 | 10 | 10 | 11.1701 | 10.7768 | 0.3933 | 10 | 10 | 10 | 0.6009 | 0.7205 |
| test_5 | 10 | 10 | 11.2815 | 10.8851 | 0.3964 | 10 | 10 | 10 | 0.5896 | 0.7290 |
| test_6 | 10 | 10 | 11.6980 | 11.2980 | 0.4000 | 9 | 10 | 10 | 0.5827 | 0.7213 |
| test_7 | 10 | 10 | 10.3200 | 9.9674 | 0.3527 | 10 | 10 | 10 | 0.6293 | 0.7820 |
| test_8 | 10 | 10 | 10.8547 | 10.4617 | 0.3930 | 10 | 10 | 10 | 0.5911 | 0.7586 |
| test_9 | 10 | 10 | 11.7169 | 11.2145 | 0.5024 | 10 | 10 | 10 | 0.5946 | 0.7234 |
| test_10 | 10 | 10 | 11.1095 | 10.7383 | 0.3713 | 10 | 10 | 10 | 0.5841 | 0.8070 |

## Reading it

**The budget is equal in evaluations, not in compute, and the two are far apart here.** An infeasible layout is rejected by the placement test and costs about 0.36 ms; a feasible one runs inverse kinematics, the manipulability check and the swept-path collision test over four branches, and costs about 6.1 ms -- some seventeen times more. Repair works by converting the cheap evaluations into expensive ones, so the repair arm buys strictly more computation for the same nominal budget. That is why `cpu_time` is in the table and why it should be read alongside the cycle time rather than as a footnote: a win at equal evaluations is a weaker claim than a win at equal seconds, and only the `cpu_time` row says which one this is.

The feasible-share column is the mechanism and the cycle-time column is the outcome, and they are not the same question. Repair can lift the share of the population that scores at all -- that is close to arithmetic -- without the extra evaluations landing anywhere useful, because a layout projected onto the placement boundary is a layout with the machines pressed up against each other and against the robot's column, which is where reach and manipulability are hardest to satisfy. If the share rises and the cycle time does not, that gap is the finding, not a bug.

