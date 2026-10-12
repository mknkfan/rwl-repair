# The penalty's constraints `g_i(x) <= 0`, numbered

Written by `experiments/make_cvd_figure.py` alongside [`cvd_example.png`](cvd_example.png).

`i` numbers the **rules**, not the violations: every layout is checked against all 45 of them. `g_i(x) <= 0` means rule `i` holds; `g_i(x) > 0` means it is broken by that many metres. Only broken rules add to

```
CVD(x) = sum_i max(0, g_i(x))
```

For `n` machines there are `4n + n + n(n-1)/2` rules -- 45 for `n = 6`. The order is the order `optimizer/penalty.py::constraint_violations` sums them in: each machine's four walls and its keep-out, machine by machine, then every pair.

Constants (test_1): cell `x, y` in [-0.70, 0.70] m, robot base at (0.00, 0.00), keep-out `R = 0.18` m, `clearance = 0.02` m.

The last two columns are the example layout drawn in the figure (M1 past the right wall, M2 inside the keep-out, M3 overlapping M1). Negative values for a pair are minus the gap beyond `clearance`; the library itself only reports the positive part.

| i | kind | rule | g_i(x) | example g_i (mm) | adds to CVD (mm) |
| ---: | --- | --- | --- | ---: | ---: |
| 1 | wall | M1 left | x_min - (min corner x) | -1289.1 | 0 |
| 2 | wall | M1 right | (max corner x) - x_max | **+30.9** | **30.9** |
| 3 | wall | M1 bottom | y_min - (min corner y) | -625.8 | 0 |
| 4 | wall | M1 top | (max corner y) - y_max | -625.8 | 0 |
| 5 | keep-out | M1 keep-out | R - (distance from base to nearest footprint point) | -409.1 | 0 |
| 6 | wall | M2 left | x_min - (min corner x) | -691.9 | 0 |
| 7 | wall | M2 right | (max corner x) - x_max | -591.9 | 0 |
| 8 | wall | M2 bottom | y_min - (min corner y) | -728.8 | 0 |
| 9 | wall | M2 top | (max corner y) - y_max | -528.8 | 0 |
| 10 | keep-out | M2 keep-out | R - (distance from base to nearest footprint point) | **+151.2** | **151.2** |
| 11 | wall | M3 left | x_min - (min corner x) | -1218.7 | 0 |
| 12 | wall | M3 right | (max corner x) - x_max | -58.7 | 0 |
| 13 | wall | M3 bottom | y_min - (min corner y) | -669.9 | 0 |
| 14 | wall | M3 top | (max corner y) - y_max | -569.9 | 0 |
| 15 | keep-out | M3 keep-out | R - (distance from base to nearest footprint point) | -338.7 | 0 |
| 16 | wall | M4 left | x_min - (min corner x) | -165.5 | 0 |
| 17 | wall | M4 right | (max corner x) - x_max | -1065.5 | 0 |
| 18 | wall | M4 bottom | y_min - (min corner y) | -1067.8 | 0 |
| 19 | wall | M4 top | (max corner y) - y_max | -167.8 | 0 |
| 20 | keep-out | M4 keep-out | R - (distance from base to nearest footprint point) | -338.5 | 0 |
| 21 | wall | M5 left | x_min - (min corner x) | -192.4 | 0 |
| 22 | wall | M5 right | (max corner x) - x_max | -1092.4 | 0 |
| 23 | wall | M5 bottom | y_min - (min corner y) | -160.6 | 0 |
| 24 | wall | M5 top | (max corner y) - y_max | -1060.6 | 0 |
| 25 | keep-out | M5 keep-out | R - (distance from base to nearest footprint point) | -353.0 | 0 |
| 26 | wall | M6 left | x_min - (min corner x) | -638.8 | 0 |
| 27 | wall | M6 right | (max corner x) - x_max | -638.8 | 0 |
| 28 | wall | M6 bottom | y_min - (min corner y) | -141.8 | 0 |
| 29 | wall | M6 top | (max corner y) - y_max | -1141.8 | 0 |
| 30 | keep-out | M6 keep-out | R - (distance from base to nearest footprint point) | -261.8 | 0 |
| 31 | pair | M1-M2 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -461.0 | 0 |
| 32 | pair | M1-M3 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | **+72.1** | **72.1** |
| 33 | pair | M1-M4 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -934.6 | 0 |
| 34 | pair | M1-M5 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -961.6 | 0 |
| 35 | pair | M1-M6 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -507.9 | 0 |
| 36 | pair | M2-M3 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -390.6 | 0 |
| 37 | pair | M2-M4 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -337.3 | 0 |
| 38 | pair | M2-M5 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -369.5 | 0 |
| 39 | pair | M2-M6 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -450.7 | 0 |
| 40 | pair | M3-M4 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -864.2 | 0 |
| 41 | pair | M3-M5 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -891.1 | 0 |
| 42 | pair | M3-M6 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -437.5 | 0 |
| 43 | pair | M4-M5 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -708.4 | 0 |
| 44 | pair | M4-M6 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -789.6 | 0 |
| 45 | pair | M5-M6 | smallest push over the 4 edge axes that leaves the boxes `clearance` apart | -311.2 | 0 |
| | | | | **CVD** | **254.1** |
