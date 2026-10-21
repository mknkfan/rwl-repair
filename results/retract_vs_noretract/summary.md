# Does the retract raise the feasible share of the population?

50 matched sets (10 instances x 5 seeds), population 200, 300 generations, 60000 evaluations per run. **No repair in any arm.**

Each set is one (instance, seed). All four arms start from the identical rejection-sampled population at the same seed and share every operator setting, so an arm differs only in its evaluator: whether the tour lifts to a retract height, and whether the clearance criterion is applied along the moves or only at the stations.

| arm | retract | clearance | mean feasible share | final | runs feasible | mean CPU (s) |
| --- | --- | --- | ---: | ---: | ---: | ---: |
| `retract_stations` | on | stations only | 0.5975 | 0.6122 | 50/50 | 51.5 |
| `direct_stations` | off | stations only | 0.5895 | 0.6135 | 50/50 | 45.3 |
| `retract_path` | on | whole cycle | 0.5879 | 0.6151 | 50/50 | 280.7 |
| `direct_path` | off | whole cycle | 0.0150 | 0.0185 | 2/50 | 24.6 |

## The controlled contrasts

Signed so **positive means the retract arm held the higher feasible share**.

**mean feasible share over the run**

| contrast | pairs | retract | direct | mean gain | 95% CI | retract wins | direct wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `retract_stations` vs `direct_stations` | 50 | 0.5975 | 0.5895 | 0.0080 | [-0.0013, 0.0170] | 31 | 19 | 0.0984 | 0.0733 |
| `retract_path` vs `direct_path` | 50 | 0.5879 | 0.0150 | 0.5729 | [0.5474, 0.5931] | 50 | 0 | 0.0000 | 0.0000 |

**feasible share at the last generation**

| contrast | pairs | retract | direct | mean gain | 95% CI | retract wins | direct wins | perm p | Wilcoxon p |
| --- | ---: | ---: | ---: | ---: | :---: | ---: | ---: | ---: | ---: |
| `retract_stations` vs `direct_stations` | 50 | 0.6122 | 0.6135 | -0.0013 | [-0.0192, 0.0155] | 22 | 25 | 0.8946 | 0.9493 |
| `retract_path` vs `direct_path` | 50 | 0.6151 | 0.0185 | 0.5966 | [0.5653, 0.6220] | 50 | 0 | 0.0000 | 0.0000 |

## The curve, generation by generation

Mean over every set of the population's feasible share.

| generation | `retract_stations` | `direct_stations` | `retract_path` | `direct_path` |
| ---: | ---: | ---: | ---: | ---: |
| 1 | 0.2790 | 0.4747 | 0.2741 | 0.0000 |
| 2 | 0.2551 | 0.3167 | 0.2514 | 0.0000 |
| 5 | 0.2567 | 0.2895 | 0.2574 | 0.0000 |
| 10 | 0.3223 | 0.3651 | 0.3190 | 0.0000 |
| 25 | 0.5541 | 0.5521 | 0.5472 | 0.0027 |
| 50 | 0.5938 | 0.5899 | 0.6043 | 0.0128 |
| 100 | 0.6299 | 0.6019 | 0.6023 | 0.0160 |
| 150 | 0.6095 | 0.6066 | 0.6007 | 0.0160 |
| 200 | 0.6191 | 0.6041 | 0.6117 | 0.0175 |
| 250 | 0.6061 | 0.6041 | 0.6076 | 0.0163 |
| 300 | 0.6122 | 0.6135 | 0.6151 | 0.0185 |

