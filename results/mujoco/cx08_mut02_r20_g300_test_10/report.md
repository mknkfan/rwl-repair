# MuJoCo check: `test_10` run 19 from `results/cx08_mut02_r20_g300`

Planned cycle 10.6065 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **17.69 mm** (finger_b to M3, t = 4.76 s). The evaluator's bounding spheres give 5.72 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 59.53 | 97.6 | 61% | 5.869 |
| shoulder | 92.0 | 186.4 | 49% | 5.334 |
| elbow | 19.61 | 89.4 | 22% | 2.197 |
| wrist roll | 0.01 | 24.2 | 0% | 10.346 |
| wrist pitch | 2.05 | 20.1 | 10% | 2.197 |
| flange roll | 0.0 | 21.3 | 0% | 4.07 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.865 mm**, RMS 0.365 mm; at the stations max 0.072 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 8206 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
