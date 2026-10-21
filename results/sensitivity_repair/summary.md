# Sensitivity of the repair gain to the feasibility thresholds

One parameter changed at a time from the defaults (sigma_min 0.14 m/rad, arm clearance 5 mm, retract 0.06 m).
Gain = wall minus repair cycle time, paired by (instance, seed); positive favours repair. Cycle times are not comparable across rows: each row is a different problem.
Holm correction over every row at both budgets, on the Wilcoxon p.

## sigma_min [m/rad]

| value | pairs | feasible share wall / repair | no layout wall / repair | T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p | gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |
| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0 | 100 | 0.256 / 0.619 | 0 / 0 | 10.931 | 10.584 | 0.347 [0.295, 0.402] | 94/100 | 0.0000 | 0.219 [0.163, 0.277] | 78/100 | 0.0000 | 44.0 / 170.1 |
| 0.07 | 100 | 0.250 / 0.612 | 0 / 0 | 10.928 | 10.605 | 0.323 [0.269, 0.379] | 93/100 | 0.0000 | 0.209 [0.153, 0.268] | 78/100 | 0.0000 | 43.2 / 171.0 |
| 0.14 (default) | 100 | 0.227 / 0.555 | 0 / 0 | 10.891 | 10.584 | 0.306 [0.259, 0.355] | 94/100 | 0.0000 | 0.173 [0.119, 0.230] | 69/100 | 0.0000 | 39.4 / 156.6 |
| 0.21 | 100 | 0.167 / 0.358 | 0 / 0 | 11.040 | 10.695 | 0.345 [0.282, 0.411] | 87/100 | 0.0000 | 0.174 [0.106, 0.245] | 66/100 | 0.0000 | 33.8 / 128.3 |

## arm_clearance [m]

| value | pairs | feasible share wall / repair | no layout wall / repair | T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p | gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |
| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0 | 100 | 0.228 / 0.594 | 0 / 0 | 10.924 | 10.587 | 0.337 [0.284, 0.390] | 94/100 | 0.0000 | 0.197 [0.134, 0.259] | 78/100 | 0.0000 | 39.3 / 162.8 |
| 0.005 (default) | 100 | 0.227 / 0.555 | 0 / 0 | 10.891 | 10.584 | 0.306 [0.259, 0.355] | 94/100 | 0.0000 | 0.173 [0.119, 0.230] | 69/100 | 0.0000 | 39.4 / 156.6 |
| 0.01 | 100 | 0.229 / 0.513 | 0 / 0 | 10.947 | 10.621 | 0.326 [0.272, 0.384] | 94/100 | 0.0000 | 0.194 [0.135, 0.258] | 79/100 | 0.0000 | 41.7 / 157.7 |
| 0.02 | 100 | 0.218 / 0.393 | 0 / 0 | 11.038 | 10.728 | 0.310 [0.251, 0.372] | 88/100 | 0.0000 | 0.180 [0.117, 0.245] | 72/100 | 0.0000 | 44.3 / 145.2 |

## retract [m]

| value | pairs | feasible share wall / repair | no layout wall / repair | T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p | gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |
| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0.03 | 100 | 0.159 / 0.277 | 0 / 0 | 10.983 | 10.694 | 0.290 [0.197, 0.382] | 76/100 | 0.0000 | 0.076 [-0.021, 0.173] | 55/100 | 0.3121 | 51.1 / 163.7 |
| 0.06 (default) | 100 | 0.227 / 0.555 | 0 / 0 | 10.891 | 10.584 | 0.306 [0.259, 0.355] | 94/100 | 0.0000 | 0.173 [0.119, 0.230] | 69/100 | 0.0000 | 39.4 / 156.6 |
| 0.1 | 100 | 0.231 / 0.541 | 0 / 0 | 11.668 | 11.339 | 0.329 [0.260, 0.395] | 91/100 | 0.0000 | 0.207 [0.135, 0.276] | 74/100 | 0.0000 | 38.3 / 152.1 |
