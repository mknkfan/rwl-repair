# Sensitivity of the repair gain to the feasibility thresholds

One parameter changed at a time from the defaults (sigma_min 0.14 m/rad, arm clearance 5 mm, retract 0.06 m).
Gain = wall minus repair cycle time, paired by (instance, seed); positive favours repair. Cycle times are not comparable across rows: each row is a different problem.
Holm correction over every row at both budgets, on the Wilcoxon p.

## sigma_min [m/rad]

| value | pairs | feasible share wall / repair | no layout wall / repair | T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p | gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |
| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0 | 50 | 0.248 / 0.616 | 0 / 0 | 10.896 | 10.574 | 0.322 [0.257, 0.392] | 47/50 | 0.0000 | 0.075 [-0.002, 0.158] | 26/50 | 1.0000 | 40.7 / 270.7 |
| 0.07 | 50 | 0.249 / 0.613 | 0 / 0 | 10.880 | 10.606 | 0.274 [0.211, 0.340] | 46/50 | 0.0000 | 0.052 [-0.019, 0.128] | 29/50 | 1.0000 | 40.2 / 270.0 |
| 0.14 (default) | 50 | 0.220 / 0.555 | 0 / 0 | 10.868 | 10.595 | 0.273 [0.213, 0.336] | 48/50 | 0.0000 | 0.003 [-0.078, 0.085] | 21/50 | 1.0000 | 36.6 / 266.4 |
| 0.21 | 50 | 0.167 / 0.361 | 0 / 0 | 11.013 | 10.685 | 0.328 [0.248, 0.416] | 44/50 | 0.0000 | -0.027 [-0.118, 0.073] | 19/50 | 1.0000 | 32.7 / 227.6 |

## arm_clearance [m]

| value | pairs | feasible share wall / repair | no layout wall / repair | T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p | gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |
| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0 | 50 | 0.217 / 0.596 | 0 / 0 | 10.873 | 10.588 | 0.286 [0.217, 0.354] | 47/50 | 0.0000 | 0.005 [-0.086, 0.090] | 24/50 | 1.0000 | 35.9 / 261.8 |
| 0.005 (default) | 50 | 0.220 / 0.555 | 0 / 0 | 10.868 | 10.595 | 0.273 [0.213, 0.336] | 48/50 | 0.0000 | 0.003 [-0.078, 0.085] | 21/50 | 1.0000 | 36.6 / 266.4 |
| 0.01 | 50 | 0.231 / 0.509 | 0 / 0 | 10.959 | 10.610 | 0.350 [0.265, 0.447] | 47/50 | 0.0000 | 0.113 [0.014, 0.222] | 30/50 | 1.0000 | 41.1 / 256.2 |
| 0.02 | 50 | 0.214 / 0.389 | 0 / 0 | 10.989 | 10.693 | 0.295 [0.219, 0.380] | 46/50 | 0.0000 | 0.048 [-0.034, 0.138] | 25/50 | 1.0000 | 41.2 / 231.7 |

## retract [m]

| value | pairs | feasible share wall / repair | no layout wall / repair | T wall [s] | T repair [s] | gain, equal gen [s] | wins | Holm p | gain, equal CPU [s] | wins | Holm p | CPU/run wall / repair [s] |
| ---: | ---: | --- | --- | ---: | ---: | --- | ---: | ---: | --- | ---: | ---: | --- |
| 0.03 | 50 | 0.159 / 0.272 | 0 / 0 | 11.047 | 10.722 | 0.325 [0.175, 0.477] | 39/50 | 0.0003 | -0.010 [-0.170, 0.151] | 24/50 | 1.0000 | 48.4 / 230.1 |
| 0.06 (default) | 50 | 0.220 / 0.555 | 0 / 0 | 10.868 | 10.595 | 0.273 [0.213, 0.336] | 48/50 | 0.0000 | 0.003 [-0.078, 0.085] | 21/50 | 1.0000 | 36.6 / 266.4 |
| 0.1 | 50 | 0.235 / 0.534 | 0 / 0 | 11.624 | 11.289 | 0.336 [0.263, 0.408] | 47/50 | 0.0000 | 0.069 [-0.021, 0.156] | 29/50 | 1.0000 | 35.4 / 251.6 |
