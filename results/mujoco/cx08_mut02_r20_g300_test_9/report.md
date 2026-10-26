# MuJoCo check: `test_9` run 6 from `results/cx08_mut02_r20_g300`

Planned cycle 10.9969 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **27.34 mm** (wrist to M6, t = 1.14 s). The evaluator's bounding spheres give 11.57 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 59.06 | 97.6 | 60% | 4.0 |
| shoulder | 92.91 | 186.4 | 50% | 5.362 |
| elbow | 20.39 | 89.4 | 23% | 4.0 |
| wrist roll | 0.01 | 24.2 | 0% | 9.054 |
| wrist pitch | 2.04 | 20.1 | 10% | 9.054 |
| flange roll | 0.0 | 21.3 | 0% | 4.037 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.916 mm**, RMS 0.38 mm; at the stations max 0.062 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 7693 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
