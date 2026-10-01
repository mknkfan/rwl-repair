# Does the placement repair help?

100 paired runs (10 instances x 10 seeds, 4 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each pair is one (instance, seed): both arms start from the identical rejection-sampled population (checked: the two runs' SHA-256 fingerprints of it match) and differ only in whether an unplaceable child is projected back onto the placeable set after mutation. Differences are taken within a pair and signed so **positive means repair did better**.


## Verdict

- **Cycle time** -- repair better by 0.1214 s on average (p = 0.0000)
- **Feasible population share** -- repair better by 0.1238 on average (p = 0.0000)
- **Evaluations to first feasible layout** -- no detectable difference (mean +0.0000 evals, p = 1.0000)
- **Gene diversity** -- repair worse by 0.2484 on average (p = 0.0470) (but see the caveat below: this metric is nearly blind to the thing repair would actually collapse)
- **CPU time** -- repair worse by 65.7202 s on average (p = 0.0000)

## Runs that found a feasible layout at all

| both | baseline only | repair only | neither | McNemar p |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0 | 0 | 0 | -- |

Only the discordant pairs carry information here; the exact test conditions on them.


## Paired metrics

| metric | pairs | baseline | repair | mean gain | 95% CI | repair wins | baseline wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `cycle_time` | 100 | 7.3192 | 7.1978 | 0.1214 | [0.0967, 0.1466] | 85 | 15 | 0.0000 | 0.0000 |
| `mean_feasible` | 100 | 0.7430 | 0.8668 | 0.1238 | [0.1156, 0.1321] | 100 | 0 | 0.0000 | 0.0000 |
| `final_feasible` | 100 | 0.7602 | 0.8791 | 0.1188 | [0.1075, 0.1301] | 98 | 2 | 0.0000 | 0.0000 |
| `evals_to_first` | 100 | 200 | 200 | 0 | [0, 0] | 0 | 0 | 1.0000 | -- |
| `mean_diversity` | 100 | 16.4250 | 16.1765 | -0.2484 | [-0.4817, -0.0090] | 41 | 59 | 0.0470 | 0.0263 |
| `cpu_time` | 100 | 218.05 | 283.77 | -65.72 | [-82.19, -49.25] | 21 | 79 | 0.0000 | 0.0000 |

`cycle_time` and `evals_to_first` are taken over the pairs where **both** arms found a feasible layout; the dropped counts are in `pairs_summary.csv`. Every row is signed so that positive favours repair -- for `mean_diversity` that means repair held *more* spread, and for `cpu_time` that it spent *less* time.


> **Caveat on `mean_diversity`.** `population_diversity` averages the per-gene standard deviation over every gene, and the rotation genes span 0-360 while the position genes span about 1.15 m. Rotation spread is therefore two orders of magnitude larger and the average is essentially rotation spread alone -- which is the one gene the repair never touches. Read this row as a check that repair leaves orientation diversity undisturbed, **not** as a measurement of whether it presses machines together onto the placement boundary. Positional collapse needs a position-only trace, and adding one to `ga.py` would change what every run tree reports, so it was measured separately instead: over 3 instances x 4 seeds x 150 generations the mean position spread is 73.8 mm without repair and 71.8 mm with it, a 2.7 % difference, and by the last generation the repaired runs hold slightly *more* spread (36.1 mm vs 34.0 mm). Both arms collapse from 377 mm to about 35 mm, which is the GA converging, not the repair flattening anything. The feared collapse does not happen.


## Per instance

| instance | pairs | both feasible | baseline s | repair s | gain s | repair wins | baseline feas. | repair feas. | baseline feas. share | repair feas. share |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| test_1 | 10 | 10 | 6.8661 | 6.8006 | 0.0656 | 5 | 10 | 10 | 0.7260 | 0.8903 |
| test_2 | 10 | 10 | 7.9253 | 7.8243 | 0.1010 | 6 | 10 | 10 | 0.7457 | 0.8360 |
| test_3 | 10 | 10 | 6.8075 | 6.6363 | 0.1712 | 9 | 10 | 10 | 0.7660 | 0.8765 |
| test_4 | 10 | 10 | 7.1908 | 7.0949 | 0.0959 | 9 | 10 | 10 | 0.7499 | 0.8591 |
| test_5 | 10 | 10 | 7.7249 | 7.5457 | 0.1793 | 10 | 10 | 10 | 0.7442 | 0.8482 |
| test_6 | 10 | 10 | 7.4479 | 7.2994 | 0.1485 | 9 | 10 | 10 | 0.7399 | 0.8498 |
| test_7 | 10 | 10 | 6.9543 | 6.8440 | 0.1103 | 10 | 10 | 10 | 0.7318 | 0.8582 |
| test_8 | 10 | 10 | 7.0424 | 6.9088 | 0.1336 | 10 | 10 | 10 | 0.7580 | 0.8624 |
| test_9 | 10 | 10 | 7.6534 | 7.5522 | 0.1011 | 7 | 10 | 10 | 0.7423 | 0.8636 |
| test_10 | 10 | 10 | 7.5792 | 7.4715 | 0.1077 | 10 | 10 | 10 | 0.7257 | 0.9234 |

## Reading it

**The budget is equal in evaluations, not in compute, and the two are far apart here.** An infeasible layout is rejected by the placement test and costs about 0.36 ms; a feasible one runs inverse kinematics, the manipulability check and the swept-path collision test over four branches, and costs about 6.1 ms -- some seventeen times more. Repair works by converting the cheap evaluations into expensive ones, so the repair arm buys strictly more computation for the same nominal budget. That is why `cpu_time` is in the table and why it should be read alongside the cycle time rather than as a footnote: a win at equal evaluations is a weaker claim than a win at equal seconds, and only the `cpu_time` row says which one this is.

The feasible-share column is the mechanism and the cycle-time column is the outcome, and they are not the same question. Repair can lift the share of the population that scores at all -- that is close to arithmetic -- without the extra evaluations landing anywhere useful, because a layout projected onto the placement boundary is a layout with the machines pressed up against each other and against the robot's column, which is where reach and manipulability are hardest to satisfy. If the share rises and the cycle time does not, that gap is the finding, not a bug.

