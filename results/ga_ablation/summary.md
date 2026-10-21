# Crossover x mutation ablation -- static GA on robot cycle time

Grid: 5 crossover rates x 6 mutation rates = 30 combinations, 20 runs on each of 10 'test' instances (6000 runs recorded).

Fixed operators: population 200, 300 generations (60000 evaluations), elitism 10%, tournament k=3, mutation width 0.1 of each gene's range. **No local search, no learned control.**

Objective: the time for one full cycle (home -> every loading port in process order -> home) with a 0.3 s dwell per station, under per-joint rate and acceleration limits, subject to `sigma_min` >= 0.14 m/rad at every port.

**Cycle time is minimised** (seconds, lower is better), and infeasible runs are excluded from the cycle-time statistics; `n_feasible` reports how many runs in each cell produced a layout at all. Evaluation counts and times cover every run.

The crossover rate is the probability that a selected pair undergoes uniform crossover; the mutation rate is the probability that any one machine (x, y, rotation) receives Gaussian jitter and a re-orientation.

`mean_rank` ranks the combinations against each other within every (instance, seed) pair -- all of which start from the same initial population -- and averages those ranks. It is the scale-free comparison; the pooled cycle-time columns mix instances whose scales differ.

Lowest mean cycle time: **crossover 0.8, mutation 0.2** (mean 10.902 s, median 10.949 s, feasible 200/200).

Best mean rank: **crossover 0.8, mutation 0.2** (8.90 of 30 cells).


## Per combination

| crossover | mutation | n | feasible | cycle mean (s) | sd | median | best | min sigma mean | evals mean | wall mean (s) | CPU mean (s) | mean rank |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 0.9 | 0.3 | 200 | 200/200 | 10.972 | 0.609 | 11.063 | 9.953 | 0.1915 | 54200 | 23.2 | 22.7 | 11.71 |
| 0.9 | 0.2 | 200 | 200/200 | 10.935 | 0.622 | 11.037 | 9.795 | 0.1918 | 54200 | 33.5 | 32.8 | 10.06 |
| 0.9 | 0.1 | 200 | 200/200 | 10.964 | 0.632 | 11.063 | 9.742 | 0.1935 | 54200 | 44.3 | 43.3 | 11.21 |
| 0.9 | 0.05 | 200 | 200/200 | 11.056 | 0.639 | 11.137 | 9.788 | 0.1909 | 54200 | 40.3 | 39.3 | 13.59 |
| 0.9 | 0.01 | 200 | 200/200 | 11.287 | 0.676 | 11.324 | 9.913 | 0.1895 | 54200 | 35.4 | 34.7 | 19.10 |
| 0.9 | 0.001 | 200 | 200/200 | 11.641 | 0.677 | 11.694 | 10.207 | 0.1870 | 54200 | 26.6 | 25.8 | 24.89 |
| 0.8 | 0.3 | 200 | 200/200 | 10.978 | 0.598 | 11.077 | 9.871 | 0.1940 | 54200 | 20.9 | 20.4 | 11.86 |
| 0.8 | 0.2 | 200 | 200/200 | 10.902 | 0.594 | 10.949 | 9.759 | 0.1945 | 54200 | 26.8 | 26.3 | 8.90 |
| 0.8 | 0.1 | 200 | 200/200 | 10.941 | 0.615 | 11.019 | 9.756 | 0.1954 | 54200 | 27.9 | 27.3 | 10.20 |
| 0.8 | 0.05 | 200 | 200/200 | 11.063 | 0.651 | 11.123 | 9.836 | 0.1932 | 54200 | 18.8 | 18.4 | 13.53 |
| 0.8 | 0.01 | 200 | 200/200 | 11.337 | 0.664 | 11.370 | 9.974 | 0.1871 | 54200 | 6.9 | 6.8 | 20.18 |
| 0.8 | 0.001 | 200 | 200/200 | 11.672 | 0.652 | 11.654 | 10.336 | 0.1888 | 54200 | 3.1 | 3.0 | 25.45 |
| 0.7 | 0.3 | 200 | 200/200 | 10.956 | 0.611 | 11.075 | 9.802 | 0.1921 | 54200 | 23.6 | 23.1 | 11.16 |
| 0.7 | 0.2 | 200 | 200/200 | 10.908 | 0.596 | 11.038 | 9.735 | 0.1924 | 54200 | 32.6 | 31.9 | 9.35 |
| 0.7 | 0.1 | 200 | 200/200 | 10.992 | 0.655 | 11.041 | 9.750 | 0.1912 | 54200 | 43.3 | 42.3 | 11.16 |
| 0.7 | 0.05 | 200 | 200/200 | 11.078 | 0.629 | 11.110 | 9.905 | 0.1906 | 54200 | 39.3 | 38.5 | 14.13 |
| 0.7 | 0.01 | 200 | 200/200 | 11.350 | 0.665 | 11.392 | 9.957 | 0.1870 | 54200 | 32.5 | 31.7 | 20.24 |
| 0.7 | 0.001 | 200 | 200/200 | 11.738 | 0.777 | 11.725 | 10.132 | 0.1897 | 54200 | 27.2 | 26.6 | 25.84 |
| 0.5 | 0.3 | 200 | 200/200 | 10.958 | 0.623 | 11.057 | 9.842 | 0.1930 | 54200 | 24.6 | 24.0 | 10.74 |
| 0.5 | 0.2 | 200 | 200/200 | 10.932 | 0.583 | 11.026 | 9.818 | 0.1928 | 54200 | 33.5 | 32.7 | 10.05 |
| 0.5 | 0.1 | 200 | 200/200 | 10.980 | 0.619 | 11.035 | 9.691 | 0.1906 | 54200 | 41.6 | 40.6 | 11.11 |
| 0.5 | 0.05 | 200 | 200/200 | 11.111 | 0.685 | 11.147 | 9.850 | 0.1884 | 54200 | 38.0 | 37.1 | 14.62 |
| 0.5 | 0.01 | 200 | 200/200 | 11.403 | 0.734 | 11.436 | 9.987 | 0.1851 | 54200 | 31.5 | 30.8 | 21.07 |
| 0.5 | 0.001 | 200 | 200/200 | 11.831 | 0.737 | 11.841 | 10.154 | 0.1879 | 54200 | 30.5 | 29.8 | 26.85 |
| 0.3 | 0.3 | 200 | 200/200 | 10.978 | 0.639 | 11.126 | 9.763 | 0.1891 | 54200 | 25.6 | 25.0 | 11.45 |
| 0.3 | 0.2 | 200 | 200/200 | 10.926 | 0.615 | 11.008 | 9.748 | 0.1915 | 54200 | 33.2 | 32.5 | 9.43 |
| 0.3 | 0.1 | 200 | 200/200 | 11.026 | 0.655 | 11.071 | 9.760 | 0.1903 | 54200 | 40.2 | 39.3 | 11.98 |
| 0.3 | 0.05 | 200 | 200/200 | 11.179 | 0.732 | 11.194 | 9.867 | 0.1881 | 54200 | 39.0 | 38.1 | 15.60 |
| 0.3 | 0.01 | 200 | 200/200 | 11.471 | 0.769 | 11.433 | 10.029 | 0.1854 | 54200 | 34.0 | 33.1 | 22.00 |
| 0.3 | 0.001 | 200 | 200/200 | 11.944 | 0.767 | 11.921 | 10.465 | 0.1853 | 54200 | 29.9 | 29.2 | 27.54 |

## Mean cycle time (s) -- crossover (rows) x mutation (columns)

| crossover \ mutation | 0.3 | 0.2 | 0.1 | 0.05 | 0.01 | 0.001 |
|---|---|---|---|---|---|---|
| **0.9** | 10.972 | 10.935 | 10.964 | 11.056 | 11.287 | 11.641 |
| **0.8** | 10.978 | 10.902 | 10.941 | 11.063 | 11.337 | 11.672 |
| **0.7** | 10.956 | 10.908 | 10.992 | 11.078 | 11.350 | 11.738 |
| **0.5** | 10.958 | 10.932 | 10.980 | 11.111 | 11.403 | 11.831 |
| **0.3** | 10.978 | 10.926 | 11.026 | 11.179 | 11.471 | 11.944 |

## Median cycle time (s) -- crossover (rows) x mutation (columns)

| crossover \ mutation | 0.3 | 0.2 | 0.1 | 0.05 | 0.01 | 0.001 |
|---|---|---|---|---|---|---|
| **0.9** | 11.063 | 11.037 | 11.063 | 11.137 | 11.324 | 11.694 |
| **0.8** | 11.077 | 10.949 | 11.019 | 11.123 | 11.370 | 11.654 |
| **0.7** | 11.075 | 11.038 | 11.041 | 11.110 | 11.392 | 11.725 |
| **0.5** | 11.057 | 11.026 | 11.035 | 11.147 | 11.436 | 11.841 |
| **0.3** | 11.126 | 11.008 | 11.071 | 11.194 | 11.433 | 11.921 |

## Mean rank -- crossover (rows) x mutation (columns)

| crossover \ mutation | 0.3 | 0.2 | 0.1 | 0.05 | 0.01 | 0.001 |
|---|---|---|---|---|---|---|
| **0.9** | 11.71 | 10.06 | 11.21 | 13.59 | 19.10 | 24.89 |
| **0.8** | 11.86 | 8.90 | 10.20 | 13.53 | 20.18 | 25.45 |
| **0.7** | 11.16 | 9.35 | 11.16 | 14.13 | 20.24 | 25.84 |
| **0.5** | 10.74 | 10.05 | 11.11 | 14.62 | 21.07 | 26.85 |
| **0.3** | 11.45 | 9.43 | 11.98 | 15.60 | 22.00 | 27.54 |

## Feasible fraction -- crossover (rows) x mutation (columns)

| crossover \ mutation | 0.3 | 0.2 | 0.1 | 0.05 | 0.01 | 0.001 |
|---|---|---|---|---|---|---|
| **0.9** | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| **0.8** | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| **0.7** | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| **0.5** | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |
| **0.3** | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 | 1.00 |

---

Wall-clock time is inflated and noisy whenever the grid was run with `--workers > 1`, because parallel runs share the machine; CPU time and the evaluation count are the reliable effort measures in that case. The results themselves do not depend on the worker count: a run's seed is fixed by its (instance, run) pair alone.
