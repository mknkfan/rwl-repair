# Does the placement repair help?

100 paired runs (10 instances x 10 seeds, 7 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each pair is one (instance, seed): both arms start from the identical rejection-sampled population (checked: the two runs' SHA-256 fingerprints of it match) and differ only in whether an unplaceable child is projected back onto the placeable set after mutation. Differences are taken within a pair and signed so **positive means repair did better**.


## Verdict

- **Cycle time** -- repair better by 0.5704 s on average (p = 0.0000)
- **Feasible population share** -- repair better by 0.1700 on average (p = 0.0000)
- **Evaluations to first feasible layout** -- no detectable difference (mean +0.0000 evals, p = 1.0000)
- **Gene diversity** -- repair worse by 0.6651 on average (p = 0.0001) (but see the caveat below: this metric is nearly blind to the thing repair would actually collapse)
- **CPU time** -- repair worse by 247.2345 s on average (p = 0.0000)

## Runs that found a feasible layout at all

| both | baseline only | repair only | neither | McNemar p |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0 | 0 | 0 | -- |

Only the discordant pairs carry information here; the exact test conditions on them.


## Paired metrics

| metric | pairs | baseline | repair | mean gain | 95% CI | repair wins | baseline wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `cycle_time` | 100 | 13.1135 | 12.5430 | 0.5704 | [0.4611, 0.6916] | 89 | 11 | 0.0000 | 0.0000 |
| `mean_feasible` | 100 | 0.5322 | 0.7021 | 0.1700 | [0.1555, 0.1844] | 100 | 0 | 0.0000 | 0.0000 |
| `final_feasible` | 100 | 0.5448 | 0.7048 | 0.1600 | [0.1399, 0.1800] | 92 | 8 | 0.0000 | 0.0000 |
| `evals_to_first` | 100 | 200 | 200 | 0 | [0, 0] | 0 | 0 | 1.0000 | -- |
| `mean_diversity` | 100 | 19.7849 | 19.1198 | -0.6651 | [-0.9721, -0.3552] | 33 | 67 | 0.0001 | 0.0001 |
| `cpu_time` | 100 | 304.76 | 551.99 | -247.23 | [-271.90, -223.08] | 3 | 97 | 0.0000 | 0.0000 |

`cycle_time` and `evals_to_first` are taken over the pairs where **both** arms found a feasible layout; the dropped counts are in `pairs_summary.csv`. Every row is signed so that positive favours repair -- for `mean_diversity` that means repair held *more* spread, and for `cpu_time` that it spent *less* time.


> **Caveat on `mean_diversity`.** `population_diversity` averages the per-gene standard deviation over every gene, and the rotation genes span 0-360 while the position genes span about 1.15 m. Rotation spread is therefore two orders of magnitude larger and the average is essentially rotation spread alone -- which is the one gene the repair never touches. Read this row as a check that repair leaves orientation diversity undisturbed, **not** as a measurement of whether it presses machines together onto the placement boundary. Positional collapse needs a position-only trace, and adding one to `ga.py` would change what every run tree reports, so it was measured separately instead: over 3 instances x 4 seeds x 150 generations the mean position spread is 73.8 mm without repair and 71.8 mm with it, a 2.7 % difference, and by the last generation the repaired runs hold slightly *more* spread (36.1 mm vs 34.0 mm). Both arms collapse from 377 mm to about 35 mm, which is the GA converging, not the repair flattening anything. The feared collapse does not happen.


## Per instance

| instance | pairs | both feasible | baseline s | repair s | gain s | repair wins | baseline feas. | repair feas. | baseline feas. share | repair feas. share |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| test_1 | 10 | 10 | 13.4530 | 12.7815 | 0.6714 | 10 | 10 | 10 | 0.5421 | 0.6578 |
| test_2 | 10 | 10 | 13.6018 | 13.1097 | 0.4921 | 8 | 10 | 10 | 0.5321 | 0.6644 |
| test_3 | 10 | 10 | 12.1454 | 11.7686 | 0.3768 | 8 | 10 | 10 | 0.5448 | 0.6925 |
| test_4 | 10 | 10 | 12.8488 | 12.2189 | 0.6299 | 9 | 10 | 10 | 0.5419 | 0.7127 |
| test_5 | 10 | 10 | 13.0104 | 12.3498 | 0.6606 | 10 | 10 | 10 | 0.5207 | 0.7341 |
| test_6 | 10 | 10 | 13.9273 | 13.1878 | 0.7395 | 9 | 10 | 10 | 0.5404 | 0.7078 |
| test_7 | 10 | 10 | 12.8674 | 12.1145 | 0.7529 | 9 | 10 | 10 | 0.5483 | 0.6821 |
| test_8 | 10 | 10 | 12.7238 | 12.3912 | 0.3327 | 8 | 10 | 10 | 0.5391 | 0.7080 |
| test_9 | 10 | 10 | 13.3642 | 12.7717 | 0.5925 | 9 | 10 | 10 | 0.5021 | 0.7183 |
| test_10 | 10 | 10 | 13.1925 | 12.7365 | 0.4560 | 9 | 10 | 10 | 0.5102 | 0.7435 |

## Reading it

**The budget is equal in evaluations, not in compute, and the two are far apart here.** An infeasible layout is rejected by the placement test and costs about 0.36 ms; a feasible one runs inverse kinematics, the manipulability check and the swept-path collision test over four branches, and costs about 6.1 ms -- some seventeen times more. Repair works by converting the cheap evaluations into expensive ones, so the repair arm buys strictly more computation for the same nominal budget. That is why `cpu_time` is in the table and why it should be read alongside the cycle time rather than as a footnote: a win at equal evaluations is a weaker claim than a win at equal seconds, and only the `cpu_time` row says which one this is.

The feasible-share column is the mechanism and the cycle-time column is the outcome, and they are not the same question. Repair can lift the share of the population that scores at all -- that is close to arithmetic -- without the extra evaluations landing anywhere useful, because a layout projected onto the placement boundary is a layout with the machines pressed up against each other and against the robot's column, which is where reach and manipulability are hardest to satisfy. If the share rises and the cycle time does not, that gap is the finding, not a bug.

