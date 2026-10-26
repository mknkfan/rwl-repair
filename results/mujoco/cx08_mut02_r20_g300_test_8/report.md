# MuJoCo check: `test_8` run 17 from `results/cx08_mut02_r20_g300`

Planned cycle 10.2412 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **19.47 mm** (wrist to M3, t = 1.29 s). The evaluator's bounding spheres give 5.59 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 56.53 | 97.6 | 58% | 0.29 |
| shoulder | 92.86 | 186.4 | 50% | 3.743 |
| elbow | 21.18 | 89.4 | 24% | 0.29 |
| wrist roll | 0.01 | 24.2 | 0% | 0.358 |
| wrist pitch | 2.07 | 20.1 | 10% | 0.29 |
| flange roll | 0.0 | 21.3 | 0% | 7.118 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **1.088 mm**, RMS 0.386 mm; at the stations max 0.053 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 6914 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
