# Does the placement repair help?

150 paired runs (10 instances x 15 seeds), population 200, 49 generations, 9800 evaluations per run.

Each pair is one (instance, seed): both arms start from the identical rejection-sampled population and differ only in whether an unplaceable child is projected back onto the placeable set after mutation. Differences are taken within a pair and signed so **positive means repair did better**.


## Verdict

- **Cycle time** -- repair better by 0.4879 s on average (p = 0.0000)
- **Feasible population share** -- repair better by 0.2537 on average (p = 0.0000)
- **Evaluations to first feasible layout** -- no detectable difference (mean +0.0000 evals, p = 1.0000)
- **Gene diversity** -- repair worse by 1.0787 on average (p = 0.0000) (but see the caveat below: this metric is nearly blind to the thing repair would actually collapse)
- **CPU time** -- repair worse by 17.1957 s on average (p = 0.0000)

## Runs that found a feasible layout at all

| both | baseline only | repair only | neither | McNemar p |
| ---: | ---: | ---: | ---: | ---: |
| 150 | 0 | 0 | 0 | -- |

Only the discordant pairs carry information here; the exact test conditions on them.


## Paired metrics

| metric | pairs | baseline | repair | mean gain | 95% CI | repair wins | baseline wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `cycle_time` | 150 | 5.6541 | 5.1662 | 0.4879 | [0.4502, 0.5270] | 150 | 0 | 0.0000 | 0.0000 |
| `mean_feasible` | 150 | 0.5012 | 0.7549 | 0.2537 | [0.2437, 0.2636] | 150 | 0 | 0.0000 | 0.0000 |
| `final_feasible` | 150 | 0.6043 | 0.8330 | 0.2287 | [0.2153, 0.2425] | 150 | 0 | 0.0000 | 0.0000 |
| `evals_to_first` | 150 | 200 | 200 | 0 | [0, 0] | 0 | 0 | 1.0000 | -- |
| `mean_diversity` | 150 | 28.4465 | 27.3678 | -1.0787 | [-1.5226, -0.6477] | 48 | 102 | 0.0000 | 0.0000 |
| `cpu_time` | 150 | 5.39 | 22.58 | -17.20 | [-17.58, -16.82] | 0 | 150 | 0.0000 | 0.0000 |

`cycle_time` and `evals_to_first` are taken over the pairs where **both** arms found a feasible layout; the dropped counts are in `pairs_summary.csv`. Every row is signed so that positive favours repair -- for `mean_diversity` that means repair held *more* spread, and for `cpu_time` that it spent *less* time.


> **Caveat on `mean_diversity`.** `population_diversity` averages the per-gene standard deviation over every gene, and the rotation genes span 0-360 while the position genes span about 1.15 m. Rotation spread is therefore two orders of magnitude larger and the average is essentially rotation spread alone -- which is the one gene the repair never touches. Read this row as a check that repair leaves orientation diversity undisturbed, **not** as a measurement of whether it presses machines together onto the placement boundary. Positional collapse needs a position-only trace, and adding one to `ga.py` would change what every archived study reports, so it was measured separately instead: over 3 instances x 4 seeds x 150 generations the mean position spread is 73.8 mm without repair and 71.8 mm with it, a 2.7 % difference, and by the last generation the repaired runs hold slightly *more* spread (36.1 mm vs 34.0 mm). Both arms collapse from 377 mm to about 35 mm, which is the GA converging, not the repair flattening anything. The feared collapse does not happen.


## Per instance

| instance | pairs | both feasible | baseline s | repair s | gain s | repair wins | baseline feas. | repair feas. | baseline feas. share | repair feas. share |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| test_1 | 15 | 15 | 5.6639 | 5.1997 | 0.4642 | 15 | 15 | 15 | 0.4971 | 0.7422 |
| test_2 | 15 | 15 | 5.6781 | 5.1479 | 0.5302 | 15 | 15 | 15 | 0.5093 | 0.7650 |
| test_3 | 15 | 15 | 5.4073 | 5.0349 | 0.3724 | 15 | 15 | 15 | 0.5239 | 0.7582 |
| test_4 | 15 | 15 | 5.5915 | 5.1210 | 0.4705 | 15 | 15 | 15 | 0.4998 | 0.7415 |
| test_5 | 15 | 15 | 5.6355 | 5.2087 | 0.4268 | 15 | 15 | 15 | 0.4992 | 0.7546 |
| test_6 | 15 | 15 | 5.8393 | 5.3461 | 0.4932 | 15 | 15 | 15 | 0.5061 | 0.7429 |
| test_7 | 15 | 15 | 5.4955 | 5.0046 | 0.4909 | 15 | 15 | 15 | 0.5336 | 0.7827 |
| test_8 | 15 | 15 | 5.5586 | 5.0429 | 0.5157 | 15 | 15 | 15 | 0.4896 | 0.7397 |
| test_9 | 15 | 15 | 5.8795 | 5.2798 | 0.5997 | 15 | 15 | 15 | 0.4887 | 0.7494 |
| test_10 | 15 | 15 | 5.7921 | 5.2767 | 0.5154 | 15 | 15 | 15 | 0.4643 | 0.7724 |

## Reading it

**The budget is equal in evaluations, not in compute, and the two are far apart here.** An infeasible layout is rejected by the placement test and costs about 0.36 ms; a feasible one runs inverse kinematics, the manipulability check and the swept-path collision test over four branches, and costs about 6.1 ms -- some seventeen times more. Repair works by converting the cheap evaluations into expensive ones, so the repair arm buys strictly more computation for the same nominal budget. That is why `cpu_time` is in the table and why it should be read alongside the cycle time rather than as a footnote: a win at equal evaluations is a weaker claim than a win at equal seconds, and only the `cpu_time` row says which one this is.

The feasible-share column is the mechanism and the cycle-time column is the outcome, and they are not the same question. Repair can lift the share of the population that scores at all -- that is close to arithmetic -- without the extra evaluations landing anywhere useful, because a layout projected onto the placement boundary is a layout with the machines pressed up against each other and against the robot's column, which is where reach and manipulability are hardest to satisfy. If the share rises and the cycle time does not, that gap is the finding, not a bug.

