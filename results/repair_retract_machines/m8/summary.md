# Does the placement repair help?

100 paired runs (10 instances x 10 seeds, 8 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each pair is one (instance, seed): both arms start from the identical rejection-sampled population (checked: the two runs' SHA-256 fingerprints of it match) and differ only in whether an unplaceable child is projected back onto the placeable set after mutation. Differences are taken within a pair and signed so **positive means repair did better**.


## Verdict

- **Cycle time** -- repair better by 0.9235 s on average (p = 0.0000)
- **Feasible population share** -- repair better by 0.1730 on average (p = 0.0000)
- **Evaluations to first feasible layout** -- no detectable difference (mean +0.0000 evals, p = 1.0000)
- **Gene diversity** -- no detectable difference (mean -0.3222, p = 0.0604) (but see the caveat below: this metric is nearly blind to the thing repair would actually collapse)
- **CPU time** -- repair worse by 293.3130 s on average (p = 0.0000)

## Runs that found a feasible layout at all

| both | baseline only | repair only | neither | McNemar p |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0 | 0 | 0 | -- |

Only the discordant pairs carry information here; the exact test conditions on them.


## Paired metrics

| metric | pairs | baseline | repair | mean gain | 95% CI | repair wins | baseline wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `cycle_time` | 100 | 15.5230 | 14.5996 | 0.9235 | [0.8112, 1.0438] | 97 | 3 | 0.0000 | 0.0000 |
| `mean_feasible` | 100 | 0.4754 | 0.6484 | 0.1730 | [0.1564, 0.1890] | 95 | 5 | 0.0000 | 0.0000 |
| `final_feasible` | 100 | 0.4812 | 0.6544 | 0.1732 | [0.1494, 0.1962] | 90 | 10 | 0.0000 | 0.0000 |
| `evals_to_first` | 100 | 200 | 200 | 0 | [0, 0] | 0 | 0 | 1.0000 | -- |
| `mean_diversity` | 100 | 20.7314 | 20.4092 | -0.3222 | [-0.6537, 0.0122] | 43 | 57 | 0.0604 | 0.0516 |
| `cpu_time` | 100 | 301.11 | 594.42 | -293.31 | [-323.17, -264.70] | 1 | 99 | 0.0000 | 0.0000 |

`cycle_time` and `evals_to_first` are taken over the pairs where **both** arms found a feasible layout; the dropped counts are in `pairs_summary.csv`. Every row is signed so that positive favours repair -- for `mean_diversity` that means repair held *more* spread, and for `cpu_time` that it spent *less* time.


> **Caveat on `mean_diversity`.** `population_diversity` averages the per-gene standard deviation over every gene, and the rotation genes span 0-360 while the position genes span about 1.15 m. Rotation spread is therefore two orders of magnitude larger and the average is essentially rotation spread alone -- which is the one gene the repair never touches. Read this row as a check that repair leaves orientation diversity undisturbed, **not** as a measurement of whether it presses machines together onto the placement boundary. Positional collapse needs a position-only trace, and adding one to `ga.py` would change what every run tree reports, so it was measured separately instead: over 3 instances x 4 seeds x 150 generations the mean position spread is 73.8 mm without repair and 71.8 mm with it, a 2.7 % difference, and by the last generation the repaired runs hold slightly *more* spread (36.1 mm vs 34.0 mm). Both arms collapse from 377 mm to about 35 mm, which is the GA converging, not the repair flattening anything. The feared collapse does not happen.


## Per instance

| instance | pairs | both feasible | baseline s | repair s | gain s | repair wins | baseline feas. | repair feas. | baseline feas. share | repair feas. share |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| test_1 | 10 | 10 | 15.6704 | 14.6970 | 0.9734 | 10 | 10 | 10 | 0.4585 | 0.6986 |
| test_2 | 10 | 10 | 16.0399 | 15.4240 | 0.6160 | 9 | 10 | 10 | 0.4604 | 0.6264 |
| test_3 | 10 | 10 | 14.6803 | 13.9372 | 0.7430 | 9 | 10 | 10 | 0.4646 | 0.5766 |
| test_4 | 10 | 10 | 14.4605 | 13.6189 | 0.8416 | 10 | 10 | 10 | 0.4828 | 0.6450 |
| test_5 | 10 | 10 | 15.5264 | 14.4378 | 1.0886 | 9 | 10 | 10 | 0.4879 | 0.7002 |
| test_6 | 10 | 10 | 15.8614 | 14.7676 | 1.0938 | 10 | 10 | 10 | 0.4867 | 0.6810 |
| test_7 | 10 | 10 | 14.9042 | 14.1592 | 0.7449 | 10 | 10 | 10 | 0.4953 | 0.6815 |
| test_8 | 10 | 10 | 15.1107 | 14.0811 | 1.0296 | 10 | 10 | 10 | 0.4747 | 0.6609 |
| test_9 | 10 | 10 | 15.6017 | 14.5793 | 1.0224 | 10 | 10 | 10 | 0.4726 | 0.6265 |
| test_10 | 10 | 10 | 17.3750 | 16.2936 | 1.0813 | 10 | 10 | 10 | 0.4705 | 0.5874 |

## Reading it

**The budget is equal in evaluations, not in compute, and the two are far apart here.** An infeasible layout is rejected by the placement test and costs about 0.36 ms; a feasible one runs inverse kinematics, the manipulability check and the swept-path collision test over four branches, and costs about 6.1 ms -- some seventeen times more. Repair works by converting the cheap evaluations into expensive ones, so the repair arm buys strictly more computation for the same nominal budget. That is why `cpu_time` is in the table and why it should be read alongside the cycle time rather than as a footnote: a win at equal evaluations is a weaker claim than a win at equal seconds, and only the `cpu_time` row says which one this is.

The feasible-share column is the mechanism and the cycle-time column is the outcome, and they are not the same question. Repair can lift the share of the population that scores at all -- that is close to arithmetic -- without the extra evaluations landing anywhere useful, because a layout projected onto the placement boundary is a layout with the machines pressed up against each other and against the robot's column, which is where reach and manipulability are hardest to satisfy. If the share rises and the cycle time does not, that gap is the finding, not a bug.

