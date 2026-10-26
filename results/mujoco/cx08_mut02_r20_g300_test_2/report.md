# MuJoCo check: `test_2` run 13 from `results/cx08_mut02_r20_g300`

Planned cycle 10.8363 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **11.32 mm** (wrist to M5, t = 4.37 s). The evaluator's bounding spheres give 7.51 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 58.15 | 97.6 | 60% | 5.48 |
| shoulder | 92.35 | 186.4 | 50% | 4.941 |
| elbow | 21.65 | 89.4 | 24% | 1.459 |
| wrist roll | 0.01 | 24.2 | 0% | 10.545 |
| wrist pitch | 2.76 | 20.1 | 14% | 0.047 |
| flange roll | 0.0 | 21.3 | 0% | 3.48 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **1.006 mm**, RMS 0.41 mm; at the stations max 0.068 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 7300 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
