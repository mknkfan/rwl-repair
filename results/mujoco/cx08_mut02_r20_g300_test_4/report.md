# MuJoCo check: `test_4` run 2 from `results/cx08_mut02_r20_g300`

Planned cycle 10.6198 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **8.4 mm** (wrist to M6, t = 7.65 s). The evaluator's bounding spheres give 6.54 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 57.96 | 97.6 | 59% | 3.708 |
| shoulder | 92.76 | 186.4 | 50% | 8.365 |
| elbow | 20.45 | 89.4 | 23% | 3.708 |
| wrist roll | 0.01 | 24.2 | 0% | 1.77 |
| wrist pitch | 2.06 | 20.1 | 10% | 10.368 |
| flange roll | 0.0 | 21.3 | 0% | 9.126 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.868 mm**, RMS 0.379 mm; at the stations max 0.090 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 7022 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
