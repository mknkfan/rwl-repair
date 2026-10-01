# MuJoCo check: `test_3` run 1 from `results/retract`

Planned cycle 9.7424 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **6.9 mm** (wrist to M1, t = 5.48 s). The evaluator's bounding spheres give 5.35 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 60.05 | 97.6 | 62% | 3.436 |
| shoulder | 92.58 | 186.4 | 50% | 6.043 |
| elbow | 19.8 | 89.4 | 22% | 4.79 |
| wrist roll | 0.01 | 24.2 | 0% | 9.473 |
| wrist pitch | 2.07 | 20.1 | 10% | 1.512 |
| flange roll | 0.0 | 21.3 | 0% | 4.66 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.942 mm**, RMS 0.343 mm; at the stations max 0.068 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 7551 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
