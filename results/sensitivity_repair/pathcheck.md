# Dense clearance re-check of every best layout

Each run's best layout re-evaluated with 100 samples per move instead of 12. 'Rejected' layouts were accepted at 12 samples and fail at 100; 'time changed' means a different posture won.

| source | layouts | rejected when dense | time changed | smallest gap [mm] | required gap [mm] |
| --- | ---: | ---: | ---: | ---: | ---: |
| default | 200 | 2 | 116 | 5.01 | 5 |
| sigma_min=0 | 200 | 3 | 116 | 5.03 | 5 |
| sigma_min=0.07 | 200 | 12 | 92 | 5.02 | 5 |
| sigma_min=0.21 | 200 | 7 | 114 | 5.00 | 5 |
| arm_clearance=0 | 200 | 6 | 115 | 0.01 | 0 |
| arm_clearance=0.01 | 200 | 7 | 112 | 10.00 | 10 |
| arm_clearance=0.02 | 200 | 17 | 110 | 20.01 | 20 |
| retract=0.03 | 200 | 200 | 0 | -- | 5 |
| retract=0.1 | 200 | 4 | 97 | 5.01 | 5 |
| popgen/p200 | 400 | 17 | 219 | 5.00 | 5 |
