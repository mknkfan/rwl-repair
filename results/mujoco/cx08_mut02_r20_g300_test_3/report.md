# MuJoCo check: `test_3` run 12 from `results/cx08_mut02_r20_g300`

Planned cycle 9.7164 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **15.6 mm** (forearm to M6, t = 2.39 s). The evaluator's bounding spheres give 11.15 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 57.94 | 97.6 | 59% | 4.812 |
| shoulder | 90.45 | 186.4 | 48% | 2.958 |
| elbow | 20.12 | 89.4 | 22% | 0.269 |
| wrist roll | 0.01 | 24.2 | 0% | 0.27 |
| wrist pitch | 2.05 | 20.1 | 10% | 8.264 |
| flange roll | 0.0 | 21.3 | 0% | 4.994 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.88 mm**, RMS 0.341 mm; at the stations max 0.060 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 8181 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
