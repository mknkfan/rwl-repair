# MuJoCo check: `test_7` run 9 from `results/cx08_mut02_r20_g300`

Planned cycle 9.7484 s, posture left/up, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **32.14 mm** (wrist to M6, t = 5.74 s). The evaluator's bounding spheres give 16.44 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 59.19 | 97.6 | 61% | 3.953 |
| shoulder | 90.14 | 186.4 | 48% | 3.468 |
| elbow | 20.35 | 89.4 | 23% | 0.277 |
| wrist roll | 0.01 | 24.2 | 0% | 8.194 |
| wrist pitch | 2.03 | 20.1 | 10% | 8.194 |
| flange roll | 0.0 | 21.3 | 0% | 4.005 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.731 mm**, RMS 0.339 mm; at the stations max 0.186 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 7811 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
