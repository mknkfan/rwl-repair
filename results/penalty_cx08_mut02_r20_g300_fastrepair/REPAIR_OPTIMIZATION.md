# The optimised placement repair

This folder reruns `results/penalty_cx08_mut02_r20_g300` after an exact speed-up of
`repair_placement` in `layout/geometry.py`. Everything else is unchanged: the same
four arms, settings, seeds and starting populations. The repaired layouts are
bit-identical to the original code, so only CPU time differs from the old study.

Reference for "original": `layout/geometry.py` at commit `23f9502f` (branch `sensitivity`).

```bash
python -m experiments.run_penalty_ab --runs 20 --generations 300 --population 200 \
    --crossover-rate 0.8 --mutation-rate 0.2 --retract 0.06 --sigma-min 0.14 \
    --workers 22 --output results/penalty_cx08_mut02_r20_g300_fastrepair
```

## Why repair needed it

The two perf commits (`9e5c734c` clearance test ~4x, `cd41a844` fitness cache and
vectorised cycle time) sped up layout scoring, which both arms share. The repair
step itself was untouched. With scoring cheap, repair became most of a repair run:

| 60-generation run, test_1, before this change | wall arm | repair arm |
|---|---:|---:|
| total CPU | 4.2 s | 21.9 s |
| scoring layouts | 3.9 s | 9.4 s |
| `repair_placement` | 0 | 11.9 s (55 %) |
| fitness cache hits | 545 / 11 000 | 555 / 11 000 |

So a repair run cost 5-7x a wall run, and at equal CPU repair got only ~52 of 300
generations.

## What changed

Two changes, both inside `repair_placement`. The arithmetic is the same IEEE
operations in the same order, so the output does not change by a single bit.

1. **Plain floats in the loop.** The loop (up to 12 passes; per pass, a column
   check per machine and 15 pairs x 4 axes for six machines) ran numpy on arrays
   of 2-4 numbers, where per-call overhead dominates. The constants are converted
   to Python lists once before the loop and the per-pair work is done on floats.
   `np.argmin`'s first-minimum rule is kept with a strict `<`, and the early exit
   on a separating axis matches the old `depths.min() <= 0` test.

   *Exception:* the projection of centres onto the axes (`centres @ axes.T`) stays
   a numpy matmul. Plain `x*a + y*b` differs from it in the last bit
   (435 554 mismatches in a 20 000-layout test, most likely fused multiply-adds),
   so rewriting it would have changed results.

2. **Lazy re-projection.** After a pair is pushed apart, both machines used to be
   re-projected immediately. Now they are marked stale and re-projected only when
   they next take part in a pair close enough to test. Positions do not change in
   between, so the values are identical.

### Left alone on purpose

- **The 12-pass cap.** About 75 % of repairs are placeable after 1-3 passes, but
  the loop keeps making sub-micrometre moves until the cap (median last move
  3e-7 m). Stopping early would be faster but would change the repaired layouts,
  that is, the method. The evaluator's speed-ups were held to "results
  unchanged", so this one was too.
- **The initial `layout_is_placeable` check.** It is shared with the evaluator, and
  its corner calculation has the same matmul rounding issue.

## Verification

| check | result |
|---|---|
| repaired layout, original vs new, bit-for-bit: every repair call from a 20-generation run plus random and clustered layouts on all instances | 0 mismatches in 78 600 |
| full 100-generation GA runs (test_1, 4, 7, 10): best chromosome, best fitness, best-so-far and feasible-share traces | 4 / 4 identical |
| `python -m tests.selftest` | all checks passed |
| this study at equal generations vs `penalty_cx08_mut02_r20_g300` | identical to every digit |

## Speed

| | repair step alone | repair run / wall run CPU |
|---|---:|---:|
| original | 1.0x | 5.3x (bench), 6.8x (old study, Windows) |
| optimised | 3.1x faster | 3.45x (bench), 3.9x (this study) |

Bench = 4 paired 100-generation runs, run one after another on one core.
Removing repair's own cost completely would leave about 2.3x, because repair
produces more feasible layouts and those are the expensive ones to score. That
part is the method working, not overhead.

## Result at equal CPU

Each (instance, seed) is cut at the smallest total CPU any arm spent, about 41 s,
and every arm is read at its last generation inside that budget. Each run is
single-threaded, and wall-clock and CPU time agree within ~2 % in every study, so
this is effectively equal wall-clock time.

| repair vs | generations (arm / repair) | repair faster by | 95 % CI | repair wins | p |
|---|---|---:|---|---:|---:|
| hard wall (+inf) | 298 / 83 | 0.180 s | [0.142, 0.220] | 146 / 200 | < 0.0001 |
| `penalty_cvd` | 167 / 83 | 0.290 s | [0.253, 0.328] | 178 / 200 | < 0.0001 |
| `penalty_count` | 165 / 83 | 0.303 s | [0.264, 0.343] | 178 / 200 | < 0.0001 |

The same comparison with the original repair (`penalty_cx08_mut02_r20_g300`):
repair vs wall +0.059 s [0.018, 0.101], 104 / 200, permutation p = 0.006, Wilcoxon
p = 0.096. Repair vs the penalties was +0.17-0.19 s.

At equal generations nothing changes: repair beats all three by 0.31-0.32 s
(183-186 / 200).

## Caveats

- **Equal-time results depend on implementation speed.** There have been three sets
  of timings so far: the original code (repair clearly ahead, `repair_equalcpu`),
  after the evaluator speed-ups (near tie), and now (clearly ahead again). Equal
  generations is the stable comparison. Equal time should be reported together
  with the note that repair received the same kind of exact optimisation as the
  evaluator.
- **Different machine.** CPU here is from Linux with 22 workers on 24 cores. The
  old study ran on Windows (wall arm 23 s per run there, 41 s here), so CPU
  seconds are not comparable across the two folders; only the ratios within each
  folder are.
- **`results/sensitivity/` used the original repair.** Its equal-CPU columns are
  out of date; its equal-generation columns are not affected.
