# Dense clearance re-check of every best layout

Each run's best layout re-evaluated with 100 samples per move instead of 12. 'Rejected' layouts were accepted at 12 samples and fail at 100; 'time changed' means a different posture won.

| source | layouts | rejected when dense | time changed | smallest gap [mm] | required gap [mm] |
| --- | ---: | ---: | ---: | ---: | ---: |
| default | 100 | 1 | 56 | 5.03 | 5 |
| sigma_min=0 | 100 | 2 | 59 | 5.03 | 5 |
| sigma_min=0.07 | 100 | 6 | 45 | 5.02 | 5 |
| sigma_min=0.21 | 100 | 4 | 59 | 5.06 | 5 |
| arm_clearance=0 | 100 | 4 | 57 | 0.02 | 0 |
| arm_clearance=0.01 | 100 | 5 | 56 | 10.00 | 10 |
| arm_clearance=0.02 | 100 | 10 | 57 | 20.02 | 20 |
| retract=0.03 | 100 | 100 | 0 | -- | 5 |
| retract=0.1 | 100 | 3 | 47 | 5.01 | 5 |
| popgen/p200 | 400 | 17 | 219 | 5.00 | 5 |
