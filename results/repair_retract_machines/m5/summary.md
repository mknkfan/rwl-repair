# Does the placement repair help?

100 paired runs (10 instances x 10 seeds, 5 machines per cell), population 200, 300 generations, 60000 evaluations per run.

Each pair is one (instance, seed): both arms start from the identical rejection-sampled population (checked: the two runs' SHA-256 fingerprints of it match) and differ only in whether an unplaceable child is projected back onto the placeable set after mutation. Differences are taken within a pair and signed so **positive means repair did better**.


## Verdict

- **Cycle time** -- repair better by 0.1841 s on average (p = 0.0000)
- **Feasible population share** -- repair better by 0.1538 on average (p = 0.0000)
- **Evaluations to first feasible layout** -- no detectable difference (mean +0.0000 evals, p = 1.0000)
- **Gene diversity** -- repair worse by 0.3040 on average (p = 0.0124) (but see the caveat below: this metric is nearly blind to the thing repair would actually collapse)
- **CPU time** -- repair worse by 127.1517 s on average (p = 0.0000)

## Runs that found a feasible layout at all

| both | baseline only | repair only | neither | McNemar p |
| ---: | ---: | ---: | ---: | ---: |
| 100 | 0 | 0 | 0 | -- |

Only the discordant pairs carry information here; the exact test conditions on them.


## Paired metrics

| metric | pairs | baseline | repair | mean gain | 95% CI | repair wins | baseline wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `cycle_time` | 100 | 9.1478 | 8.9638 | 0.1841 | [0.1460, 0.2243] | 85 | 15 | 0.0000 | 0.0000 |
| `mean_feasible` | 100 | 0.6576 | 0.8114 | 0.1538 | [0.1451, 0.1624] | 100 | 0 | 0.0000 | 0.0000 |
| `final_feasible` | 100 | 0.6813 | 0.8253 | 0.1439 | [0.1312, 0.1566] | 97 | 3 | 0.0000 | 0.0000 |
| `evals_to_first` | 100 | 200 | 200 | 0 | [0, 0] | 0 | 0 | 1.0000 | -- |
| `mean_diversity` | 100 | 17.4928 | 17.1888 | -0.3040 | [-0.5388, -0.0650] | 38 | 62 | 0.0124 | 0.0124 |
| `cpu_time` | 100 | 251.60 | 378.75 | -127.15 | [-148.09, -106.20] | 16 | 84 | 0.0000 | 0.0000 |

`cycle_time` and `evals_to_first` are taken over the pairs where **both** arms found a feasible layout; the dropped counts are in `pairs_summary.csv`. Every row is signed so that positive favours repair -- for `mean_diversity` that means repair held *more* spread, and for `cpu_time` that it spent *less* time.


> **Caveat on `mean_diversity`.** `population_diversity` averages the per-gene standard deviation over every gene, and the rotation genes span 0-360 while the position genes span about 1.15 m. Rotation spread is therefore two orders of magnitude larger and the average is essentially rotation spread alone -- which is the one gene the repair never touches. Read this row as a check that repair leaves orientation diversity undisturbed, **not** as a measurement of whether it presses machines together onto the placement boundary. Positional collapse needs a position-only trace, and adding one to `ga.py` would change what every run tree reports, so it was measured separately instead: over 3 instances x 4 seeds x 150 generations the mean position spread is 73.8 mm without repair and 71.8 mm with it, a 2.7 % difference, and by the last generation the repaired runs hold slightly *more* spread (36.1 mm vs 34.0 mm). Both arms collapse from 377 mm to about 35 mm, which is the GA converging, not the repair flattening anything. The feared collapse does not happen.


## Per instance

| instance | pairs | both feasible | baseline s | repair s | gain s | repair wins | baseline feas. | repair feas. | baseline feas. share | repair feas. share |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| test_1 | 10 | 10 | 8.6856 | 8.5065 | 0.1791 | 9 | 10 | 10 | 0.6633 | 0.7954 |
| test_2 | 10 | 10 | 9.8346 | 9.5675 | 0.2672 | 9 | 10 | 10 | 0.6668 | 0.8299 |
| test_3 | 10 | 10 | 8.6805 | 8.5210 | 0.1596 | 7 | 10 | 10 | 0.6488 | 0.8125 |
| test_4 | 10 | 10 | 9.2620 | 9.1081 | 0.1538 | 8 | 10 | 10 | 0.6669 | 0.7971 |
| test_5 | 10 | 10 | 9.6112 | 9.4351 | 0.1762 | 9 | 10 | 10 | 0.6541 | 0.8016 |
| test_6 | 10 | 10 | 9.4280 | 9.2860 | 0.1420 | 9 | 10 | 10 | 0.6530 | 0.7876 |
| test_7 | 10 | 10 | 8.3959 | 8.2282 | 0.1678 | 7 | 10 | 10 | 0.6567 | 0.8093 |
| test_8 | 10 | 10 | 8.6057 | 8.4403 | 0.1654 | 9 | 10 | 10 | 0.6675 | 0.8273 |
| test_9 | 10 | 10 | 9.4782 | 9.2906 | 0.1875 | 8 | 10 | 10 | 0.6548 | 0.8172 |
| test_10 | 10 | 10 | 9.4967 | 9.2543 | 0.2423 | 10 | 10 | 10 | 0.6442 | 0.8359 |

## Reading it

**The budget is equal in evaluations, not in compute, and the two are far apart here.** An infeasible layout is rejected by the placement test and costs about 0.36 ms; a feasible one runs inverse kinematics, the manipulability check and the swept-path collision test over four branches, and costs about 6.1 ms -- some seventeen times more. Repair works by converting the cheap evaluations into expensive ones, so the repair arm buys strictly more computation for the same nominal budget. That is why `cpu_time` is in the table and why it should be read alongside the cycle time rather than as a footnote: a win at equal evaluations is a weaker claim than a win at equal seconds, and only the `cpu_time` row says which one this is.

The feasible-share column is the mechanism and the cycle-time column is the outcome, and they are not the same question. Repair can lift the share of the population that scores at all -- that is close to arithmetic -- without the extra evaluations landing anywhere useful, because a layout projected onto the placement boundary is a layout with the machines pressed up against each other and against the robot's column, which is where reach and manipulability are hardest to satisfy. If the share rises and the cycle time does not, that gap is the finding, not a bug.

