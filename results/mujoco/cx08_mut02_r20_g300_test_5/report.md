# MuJoCo check: `test_5` run 8 from `results/cx08_mut02_r20_g300`

Planned cycle 10.673 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **27.4 mm** (finger_a to M4, t = 7.87 s). The evaluator's bounding spheres give 5.22 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 58.88 | 97.6 | 60% | 5.277 |
| shoulder | 92.24 | 186.4 | 50% | 6.686 |
| elbow | 18.74 | 89.4 | 21% | 3.962 |
| wrist roll | 0.01 | 24.2 | 0% | 1.973 |
| wrist pitch | 2.03 | 20.1 | 10% | 1.972 |
| flange roll | 0.0 | 21.3 | 0% | 4.118 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.839 mm**, RMS 0.37 mm; at the stations max 0.064 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 7854 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
