# MuJoCo check: `test_1` run 14 from `results/cx08_mut02_r20_g300`

Planned cycle 9.6961 s, posture right/down, retract 0.06 m.

## Clearance, solid arm vs machines

Worst gap along the cycle: **16.99 mm** (wrist to M6, t = 2.72 s). The evaluator's bounding spheres give 5.1 mm on the same samples; the spheres enclose the solid, so theirs should be the smaller number.

## Torque the planned motion needs

Inverse dynamics (Armstrong/Corke inertias, reflected motor inertia, viscous and Coulomb friction). Limits are the commonly quoted Puma 560 figures, **not verified against a Unimation document**.

| joint | peak [N m] | limit [N m] | use | at [s] |
|---|---:|---:|---:|---:|
| waist | 59.97 | 97.6 | 61% | 2.057 |
| shoulder | 89.57 | 186.4 | 48% | 7.648 |
| elbow | 19.15 | 89.4 | 21% | 2.057 |
| wrist roll | 0.01 | 24.2 | 0% | 2.058 |
| wrist pitch | 2.07 | 20.1 | 10% | 2.057 |
| flange roll | 0.0 | 21.3 | 0% | 6.417 |

## Tracking under physics

Computed-torque control at 25.0 rad/s, Coulomb feed-forward x1.0, torques clipped at the limits.

- Tool tip error: max **0.874 mm**, RMS 0.325 mm; at the stations max 0.057 mm.
- Collision steps outside the approach corridor: **0**.
- Gripper contact with the served machine's top: 8892 steps (expected: the tool tip is planned onto the top surface).

![torques](torques.png)

![tracking](tracking.png)
