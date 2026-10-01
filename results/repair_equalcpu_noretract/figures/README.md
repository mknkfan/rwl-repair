# What these figures are

One directory per instance, each the **fastest feasible run** of that
instance in this tree, redrawn from the chromosome in its `raw/` JSON.
`best_runs.json` says which arm and seed each came from.

## These cycles collide, and are meant to

10 of 10 drawn cycles put part of the robot
**inside** a machine while moving between stations, by up to 98 mm.
That is not a gap in the collision model: the arm is bounded by
`armmesh.collision_spheres`, wrist and gripper included, and the
station poses here all clear it. It is that this tree was run with
the **path check off and no retract** (`retract: null`,
`check_path: false` in `metadata.json`), so the moves between
stations were never scored at all -- only the stations were.

With direct point-to-point moves the tool interpolates in joint
space and drags the arm through whatever stands between two ports,
which is why the retract exists. No run in this tree is path-clean,
so there is no better one to draw; the figures are faithful to what
was optimised. For cycles that clear the machines throughout, see a
tree whose `metadata.json` has `check_path: true` and a retract.

| instance | worst gap | part | inside | segments |
|---|---|---|---|---|
| test_1 | -51 mm | tool/gripper | M3 | 5 |
| test_2 | -61 mm | tool/gripper | M4 | 6 |
| test_3 | -63 mm | tool/gripper | M6 | 6 |
| test_4 | -59 mm | tool/gripper | M3 | 6 |
| test_5 | -80 mm | tool/gripper | M2 | 7 |
| test_6 | -80 mm | tool/gripper | M2 | 5 |
| test_7 | -69 mm | tool/gripper | M6 | 6 |
| test_8 | -48 mm | tool/gripper | M2 | 6 |
| test_9 | -98 mm | tool/gripper | M2 | 5 |
| test_10 | -69 mm | tool/gripper | M6 | 6 |

Gaps are from `CycleTimeEvaluator.arm_clearances`, the same call the
feasibility test uses, at 24 samples per tour segment.
Negative means that far inside the box.
