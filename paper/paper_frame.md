# Paper frame: repair vs wall for robot workcell layout GA

Oct 14, 2026

A six-page frame for a conference paper on handling constraints in a GA that minimises a Puma 560 cell's cycle time. Placement repair beats the +inf wall and two graded penalties at equal generations and at equal evaluations, and it still wins at equal CPU, by a smaller margin.

## Key results to show

The headline is one fact read at three budgets: repair wins by about 0.31 s at equal generations and equal evaluations, and still wins at equal CPU, by about 0.06 to 0.15 s. Six pages hold roughly two tables and three figures, so everything below has to fit those five exhibits.

**T1 — Four constraint handlers at three budgets** (6 machines, 10 instances × 10 seeds, paired). The main table of the paper.

| Budget | `baseline` (+inf wall) [s] | `repair` [s] | `penalty_cvd` [s] | `penalty_count` [s] | Repair gain vs wall |
| --- | ---: | ---: | ---: | ---: | --- |
| Equal generations (P=200, G=300) | 10.891 | **10.584** | 10.924 | 10.907 | +0.306 s [0.259, 0.356], 94/100 wins |
| Equal evaluations (40 000, P=200) | 10.930 | **10.597** | 10.958 | 10.950 | about +0.33 s |
| Equal CPU (40 s per run, best P per arm) | 10.830 | **10.698** | 10.922 | 10.943 | about +0.13 s |
| Equal CPU (20 s per run, best P per arm) | 10.921 | **10.838** | 11.078 | 11.098 | about +0.08 s |

Source: `results/popgen/summary.md`. Holm-corrected paired permutation tests; repair beats every other arm in all 96 equal-generation cells (p < 0.001). Before submitting, recompute the gains at equal evaluations and equal CPU as paired values with confidence intervals; the table gives differences of means.

**T2 — Why the retract is part of the model** (`results/retract_ab`, 50 matched sets, no repair).

| Motion model | Clearance checked | Feasible share of population | Runs that found a layout | CPU per run [s] |
| --- | --- | ---: | ---: | ---: |
| Retract | whole cycle | 0.588 | 50/50 | 280.7 |
| Direct point-to-point | whole cycle | 0.015 | 2/50 | 24.6 |
| Retract | stations only | 0.598 | 50/50 | 51.5 |
| Direct point-to-point | stations only | 0.590 | 50/50 | 45.3 |

Without the retract, whole-cycle clearance is almost unsatisfiable: 2 of 50 runs found any layout. Stations-only checking hides this, because the two motion models then differ by 0.008 in feasible share (p = 0.10).

**F1 — Cycle time against budget.** Two panels: best cycle time against evaluations, and against CPU seconds, one line per arm (from `results/popgen/grid.csv`). It shows the repair curve sitting below the others against evaluations, with the gap narrowing against CPU. This is the figure reviewers remember, so draw it new rather than reusing `popgen.png`.

**F2 — Gain against machine count** (4 to 8 machines, 200 pairs per count). Two lines: gain at equal generations, which grows from +0.11 s to +0.90 s, and gain at equal CPU, which stays between +0.06 and +0.15 s. The penalties match the wall at equal generations (Holm p = 1.0 at every count) and fall behind it at equal CPU from 6 machines up (−0.11 to −0.37 s). Source: `results/penalty_cx08_mut02_r20_g300_machines`.

**F3 — Mechanism and validation**, one row of small panels: (a) one repair round on a real mutated child (`results/model/repair_example.png`); (b) the σ_min-rejected neighbour of a converged layout (`results/popgen/sigma_case`), which illustrates the flat plateau no placement-only method can see; (c) MuJoCo check of a solved cell: solid-model gap 6.9 mm against 5.35 mm for the bounding spheres, peak torque use 62 % (waist), tracking error 0.94 mm.

Numbers for the text rather than an exhibit:

- Repair raises the feasible share of the population from 0.578 to 0.741 (150/150 pairs).
- A placement rejection costs about 0.36 ms and a full kinematic evaluation about 6.1 ms, roughly 17 times more. This is why equal evaluations and equal CPU disagree.
- Per run, repair costs about 6.5 times the wall's CPU at equal generations (155.7 s against 23.8 s at P=200, G=300).
- Repair's gene diversity is lower by 0.57 (p < 0.001); the metric is dominated by the rotation genes, so treat it as a caveat, not a result.
- Operator setting from the ablation: crossover 0.8, mutation 0.2 gave the best mean rank of the 30 cells.

## Page budget

At six pages including references, the methodology and the results get about four pages between them, and the related work gets well under one. Figures are sized to a single column except F1.

| Part | Pages | Holds |
| --- | ---: | --- |
| Title, abstract, keywords | 0.35 | 150–200-word abstract |
| 1 Introduction | 0.6 | motivation, gap, three contributions |
| 2 Related Work | 0.6 | three paragraphs, Table R1 optional |
| 3 Methodology | 1.5 | formulation, feasibility, retract, repair, penalties, protocol; F3a |
| 4 Results and Discussion | 2.0 | T1, T2, F1 (two columns wide), F2, F3b–c |
| 5 Limitations and Future Work | 0.3 | five bullets |
| 6 Conclusion | 0.15 | one paragraph |
| References | 0.6 | 20–25 entries |

What to cut first if over: the operator ablation (one sentence), the per-instance table (supplementary), and the population-size sweep (one sentence plus the grid in supplementary).

## Title, abstract and contributions

Working titles, from most to least specific:

- *Repair, Reject or Penalise? Constraint Handling for Cycle-Time Optimal Robot Workcell Layout*
- *Placement Repair Pays at Equal Compute: A Paired Study of Constraint Handling in Robotic Cell Layout*

Abstract skeleton, one sentence per line:

1. Problem: placing machines around a 6-DOF arm to minimise cycle time is constrained by placement, reach, joint limits, conditioning (σ_min) and clearance along the whole cycle.
2. Gap: GA layout studies handle constraints by rejection or a penalty, and compare methods at equal generations, which hides the cost of each evaluation.
3. Method: a GA over (x, y, θ) per machine, a retract motion model that makes whole-cycle clearance satisfiable, and four constraint handlers: the +inf wall, placement repair, and two layered penalties.
4. Protocol: paired runs from identical populations, read at equal generations, equal evaluations and equal CPU, over 4–8 machines and a population × generation grid.
5. Result: repair shortens the cycle by 0.31 s (≈ 2.8 %) at equal generations and keeps a 0.06–0.15 s lead at equal CPU; graded penalties match the wall and fall behind it at equal CPU.
6. Validation: solved cells pass an independent MuJoCo check of clearance, torque and tracking.

Contributions:

1. A cycle-time layout model for a Puma 560 cell with five feasibility criteria and a retract motion that raises the feasible share under whole-cycle clearance from 0.015 to 0.588.
2. A paired comparison of four constraint handlers at three budget definitions, showing that the ranking depends on which budget is held equal.
3. Evidence that repair's advantage grows with problem size at equal generations (0.11 → 0.90 s from 4 to 8 machines) but not at equal CPU, plus open code and per-run traces.

## 1 Introduction

Four paragraphs, about 0.6 pages. Numbers in brackets point to the References section at the end.

1. **Why layout matters.** In a single-robot cell the arm is often the bottleneck, so where the machines sit sets the cycle time [1], [3], [5]. Layout is still mostly placed by hand in a simulator and checked by trial [3], [7], [16].
2. **Why it is hard.** Layout is a continuous facility-layout problem, NP-hard in general [9], [29], with robot-specific constraints on top: every port must be reachable inside joint limits, away from singular poses [12], and the arm must clear every machine along the whole motion, not only at the stations [13]. Recent work shows that proxy objectives such as distance or area approximate cycle time poorly, which argues for timing the actual motion [5].
3. **The gap.** GA layout studies handle infeasible layouts by rejecting them or by an ad hoc penalty [1], [2], [6], [10]. The constraint-handling literature offers repair as a third option [19], [21], and geometric repair has recently been built into a GA layout loop [34], but it rarely accounts for cost. A repaired child is usually a feasible one, and a feasible evaluation is far more expensive than a rejected one. Comparisons at a fixed number of evaluations can therefore favour the method that spends the most compute [23]–[25].
4. **This paper.** State the three contributions from the list above, then one line naming the instances (10 test cells, 4–8 machines) and that the code and traces are released.

One-sentence hook for paragraph 1, if wanted: "a layout that saves 0.3 s on a 10.9 s cycle is about 3 % more output from the same robot".

## 2 Related Work

Three paragraphs, one per theme, each ending with how this paper differs. Table R1 is optional and can move to supplementary if space runs out. Claims marked *(check)* come from abstracts only; confirm them against the full text.

**2.1 Robot workcell layout optimisation.** Most RWL studies use a GA. Early work places machines around a single robot to minimise cycle time or arm travel [1], [8], [36]. Later work adds objectives and compares optimisers. Lim et al. optimise area, operation time and manipulability with NSGA-II and test five nature-inspired algorithms (GA, DE, ABC, CSS, PSO) on sequence-pair and B*-tree encodings [2]. Others minimise footprint and transfer time with a GA [3], combine a GA with ant-colony path optimisation [4], trade travel distance against manipulability for a dual-arm cell [35], optimise layout and trajectory together [37], use surrogate-assisted motion planning [6], or drive a commercial simulator [7]. Baumgärtner et al. time the actual time-optimal trajectory instead of a proxy, and show that distance and area proxies rank layouts poorly [5]. The parent problem is the continuous facility layout problem [9], [29]. *Difference:* these studies vary the optimiser, the encoding or the objective. The constraint handler is fixed and rarely reported.

**2.2 Constraints in RWL and how they are handled.** Four families of constraints recur (Table R1).
- **Placement:** machines inside the cell, not overlapping, clear of the robot base. Usually removed by the encoding: sequence pair and B*-tree [2], constructive spiral placement [8], or slicing structures and local moves borrowed from facility layout [30]. Some studies instead test simplified envelopes, such as circle overlap [37], and reject or penalise violations [3], [38] *(check [3])*.
- **Reachability:** every port within reach and joint limits. Checked by inverse kinematics as a hard constraint [35], or built into engineering rules that shrink the search space [36].
- **Conditioning:** distance from singular poses, measured by manipulability [12]. In RWL it is usually an objective to maximise [2], [35], not a floor. Base-placement methods also use it as a criterion [13]–[16], [28].
- **Collision along the motion:** checked at the stations or along the path [1], [37]. Qiu et al. repair collided trajectory points by retreating them radially or vertically out of the device envelopes [37]. Xu et al. use a penalty [38].

*Difference:* this paper treats all four families as hard feasibility, checked in cost order (§3.2). Repair handles only the placement family, and the kinematic families are left to rejection or penalty. This makes explicit which constraints a geometric repair can fix and which it cannot (§4.6).

*Table R1 (optional).*

| Constraint family | Typical handling in RWL | Here |
|---|---|---|
| Placement (boundary, overlap, column) | Encoding [2], [8]; envelope test with reject or penalty [37], [38] | Repair (§3.5) vs wall vs two penalties |
| Reachability (IK, joint limits) | Hard constraint [35]; rule-based search-space reduction [36] | Wall |
| Conditioning (manipulability) | Objective [2], [35]; base-placement criterion [13]–[16] | Hard floor σ_min ≥ 0.14 m/rad, wall |
| Collision along the motion | Station check [1]; path repair [37]; penalty [38] | Whole cycle with retract, wall |

**2.3 Constraint handling in GAs.** Surveys sort the options into rejection, penalties, feasibility rules and repair [17], [19], [27].
- **Penalties** need a weight, and several variants have been compared numerically [20].
- **Feasibility rules:** Deb's feasibility-first tournament removes the penalty weight [18]. A layered penalty with a feasible-always-wins floor behaves the same under rank selection, which is why the wall and the penalties tie here.
- **Repair** maps an infeasible child onto a feasible one, and can write it back into the chromosome (Lamarckian) [19]. Adaptive repair has been shown to reach feasibility in far fewer generations [21]. Closest to this paper, Zhang et al. [34] embed a boundary clip and separating-axis overlap repair in a hybrid GA–SA for equipment layout, applied after every crossover, mutation and SA move, with the repaired solution kept in the search. They compare the full GA–SA against a standard GA and SA, so repair's own effect is never isolated, and their run times differ (1.22 s vs 0.54 s), so the budgets are not matched on CPU. In RWL itself, Sánchez-Sosa et al. [3] keep the GA's stations free of interference *(describe their mechanism here: repair, rejection or penalty)*.
- **Fair comparison:** benchmarking work shows that rankings change with the budget [23], [26], and that budgets counted in evaluations hide per-evaluation cost [24], [25].

*Difference:* this paper isolates repair against rejection and two penalties on the same paired runs, at equal generations, evaluations and CPU.

## 3 Methodology

About 1.5 pages in seven subsections. Every constant below comes from the code (`optimizer/evaluator.py`, `optimizer/penalty.py`, `layout/geometry.py`).

**3.1 Problem formulation.** A cell holds n machines (n = 4–8) around a fixed Puma 560 at the origin. The decision vector is x = (xᵢ, yᵢ, θᵢ) for i = 1..n, with each machine's centre inside a 1.4 m × 1.4 m cell. The tool visits every machine's loading port in a fixed process order, starting and ending at home. The objective is the cycle time:

```latex
\min_{x}\; T(x) = \sum_{k} t_k^{\text{move}}(x) + n\,t_{\text{dwell}} \quad \text{s.t.}\; x \in \mathcal{F}
```

Each move is a coordinated joint-space trapezoidal profile limited by the Puma's joint speed and acceleration limits. The dwell is 0.3 s per station.

**3.2 Feasibility 𝓕 (five criteria, checked in cost order).**

1. Placement: every footprint inside the cell, at least R = 0.18 m from the robot column, and at least 0.02 m from every other footprint.
2. Reach: a vertical-approach inverse kinematics solution exists at every port, inside joint travel.
3. Conditioning: σ_min of the position Jacobian is at least 0.14 m/rad at every port [12].
4. Clearance: the arm, bounded by collision spheres fitted to its solid model, stays 5 mm from every machine at the stations and at 12 samples along every move.
5. One posture: a single IK branch (shoulder left/right, elbow up/down) satisfies 2–4 for the whole cycle; the fastest such branch defines T(x).

**3.3 Retract motion.** Each station is bracketed by a lift pose 0.06 m above the tallest machine (descend, dwell, retract, traverse). Point-to-point joint moves sweep the forearm through neighbouring machines; T2 shows this leaves 2 of 50 runs feasible.

**3.4 GA.** Real-coded GA with tournament selection (k = 3), uniform crossover (p = 0.8), per-machine Gaussian mutation (p = 0.2, σ = 0.1 of the gene range, rotation redrawn), and 10 % elitism. Default population is 200 and default generations 300. Fitness is cached by chromosome, and every call counts toward the evaluation budget, including cache hits.

**3.5 Placement repair (proposed).** The operator belongs to the boundary-clip and separating-axis family used by [34]; what follows fixes the details that [34] leaves open (push size and split, keep-out zone, pass limit, margin). After mutation, a child that fails the placement criterion (§3.2, item 1) is moved back onto the placeable set before it is evaluated. The repaired layout replaces the child in the population (Lamarckian). Each pass makes three moves: (i) clip every centre into the cell, inset by the footprint half-diagonal; (ii) push any machine that encroaches on the robot column radially outward by its shortfall; (iii) for every overlapping pair, find the separating axis needing the smallest push (the minimum translation from the separating-axis test) and move both machines apart by half that push each. The moves interact, since pushing a machine off the column can push it into a neighbour, so the pass repeats until nothing moves, up to 12 passes. Each constraint is cleared with an extra 5 mm, because a layout sitting exactly on a constraint boundary fails later checks on rounding error. Repair has three properties. It leaves a layout that is already placeable unchanged. It changes positions only, never rotations, so the gene that decides where each loading port faces stays as mutation drew it. It never evaluates the cycle time, so it is a projection, not a local search. It does not grade reach, σ_min or clearance; those are left to the feasibility test. The pass limit means a crowded layout can come back still unplaceable, and the feasibility test then rejects it. Show F3a here: one repair round on a mutated child.

**3.6 Baseline constraint handlers.**

- *Wall (baseline):* any infeasible layout scores +inf.
- *Layered penalties:* the formula below, with CVD summed over the 4n + n + n(n−1)/2 placement constraints (45 for n = 6) and N the number of machines in an overlap; `penalty_cvd` uses m = 0.

```latex
F(x) = \begin{cases} T(x) & x \in \mathcal{F} \\ T_\infty + w\,\mathrm{CVD}(x) + m\,N(x) & \text{otherwise} \end{cases}, \quad T_\infty = 1000\,\text{s}
```

State that w has no effect under rank-based selection and that a placeable but unreachable layout scores exactly T∞, so the penalties grade only what repair fixes (§3.5).

**3.7 Experimental protocol.**

- 10 test instances × 10–20 seeds; run seed = 42 + 100 × instance + run.
- All arms of a set start from the identical rejection-sampled population, checked by fingerprint.
- Three budgets, read off per-generation traces: equal generations, equal evaluations (P × G), and equal CPU seconds. Explain why a G-generation run equals the first G generations of a longer run: no schedule depends on G.
- Statistics: paired differences, bootstrap 95 % CIs, paired permutation and Wilcoxon tests, Holm correction within each table.
- External check: MuJoCo [31] replay of solved cells for exact solid-model clearance, inverse-dynamics torque against Puma limits [32], and computed-torque tracking.

## 4 Results and Discussion

About 2 pages in seven short subsections. Each one opens with its finding and its number, then points to the exhibit.

**4.1 The retract makes the problem solvable (T2).** With whole-cycle clearance, direct moves leave 1.5 % of the population feasible and 2/50 runs with any layout. The retract restores 58.8 % and 50/50, at about 11× the CPU. Checking stations only would have hidden the difference entirely.

**4.2 Fitness quality at equal generations (T1 row 1).** Repair finds a 0.306 s shorter cycle than the wall [0.259, 0.356] and wins 94/100 pairs. The penalties do not differ from the wall. Repair's lead holds in all 96 cells of the population × generation grid and shrinks with budget, from 0.83 s at G = 25 to 0.26 s at G = 500 (P = 200).

**4.3 Equal evaluations (F1, left panel).** At 40 000 evaluations repair still leads by about 0.33 s. The grid also gives a sizing rule: at 5 000 evaluations, P = 50–100 beats P = 200 by about 0.24–0.39 s, while from 20 000 evaluations up, P = 200 is as good as or better than the alternatives. One sentence, with details in supplementary.

**4.4 Equal CPU (F1, right panel; T1 rows 3–4).** This is the honest comparison, and the margin shrinks to about 0.08 s at 20 s and 0.13 s at 40 s per run. The mechanism: a placement rejection costs about 0.36 ms and a full kinematic evaluation about 6.1 ms, and repair turns cheap rejections into expensive evaluations (feasible share 0.58 → 0.74). Per run, repair spends about 6.5× the wall's CPU at equal generations. Report CPU seconds as the budget and give wall-clock time once for the hardware. The raw JSON records both (`cpu_time`, `execution_time`). State that repair received the same kind of exact, result-preserving speed-up as the evaluator (`results/cx08_mut02_r20_g300/REPAIR_OPTIMIZATION.md`), since equal-CPU results depend on how fast each piece of code is.

**4.5 Scaling with machine count (F2).** At equal generations the gain grows from 0.11 s (4 machines) to 0.90 s (8 machines), because placement gets tighter as the cell fills. At equal CPU it stays between 0.06 and 0.15 s and is significant at every count. The penalties fall behind the wall at equal CPU from 6 machines up (−0.11 to −0.37 s): they pay for the full kinematic check on more layouts without ranking better.

**4.6 Why graded penalties do not help (F3b).** Under rank selection with a feasible-always-wins floor, the weight w cannot change any ranking, and the penalty grades only placement. Every layout that fails reach, σ_min or clearance sits on the same plateau at T∞. F3b shows a converged layout's neighbour, 51 mm away, rejected only because σ_min = 0.014 m/rad (threshold 0.14) at one station. It scores the same as a layout that misses by 0.001 m/rad. The penalties and repair alike are blind to this, which motivates the future work on kinematic constraints.

**4.7 Physical validation (F3c).** A solved cell replayed in MuJoCo keeps a 6.9 mm worst gap on the solid model, against 5.35 mm for the evaluator's spheres, so the spheres are conservative. Peak torque use is 62 % (waist) and tool error 0.94 mm max under computed-torque control. Note the caveat that the torque limits are commonly quoted figures, not verified against a Unimation document.

**Discussion paragraph.** Repair is worth using when evaluations are expensive relative to rejections only if the comparison is made at equal CPU, and here it survives that test. Also state the counter-effect: repaired children sit on the placement boundary, where reach and σ_min are hardest. The diversity metric cannot show this, because rotation spread dominates it.

## 5 Limitations and Future Work

Five bullets, each a limitation with the next step that addresses it.

- **Placement-only repair and penalties.** Neither grades reach, σ_min or clearance failures (F3b). *Next:* a kinematic penalty such as max(0, 0.14 − σ_min), or a Jacobian-guided repair step, priced at equal CPU.
- **One robot, one process order.** Puma 560, fixed sequence and fixed home pose. *Next:* co-optimise the sequence [1], and test a modern 6-axis arm.
- **Simplified motion.** Joint-space trapezoids with a fixed retract height, not time-optimal trajectories [5]. *Next:* time-optimal timing in the loop, or a learned surrogate for it [6].
- **Diversity metric.** Dominated by the rotation genes, it cannot measure positional collapse at the placement boundary. *Next:* a position-only diversity trace.
- **Validation depth.** MuJoCo was checked on a few solved cells, against unverified torque limits, with no physical cell. *Next:* replay every best layout, and use vendor limits.

## 6 Conclusion

One paragraph of four sentences. (1) Restate the problem and the retract model that makes it solvable. (2) The result: placement repair shortens the cycle by 0.31 s at equal generations and keeps a 0.06–0.15 s lead at equal CPU, while graded placement penalties match the +inf wall and lose to it at equal CPU. (3) The lesson for the field: report equal-CPU results whenever constraint handlers change what an evaluation costs. (4) Point to the code and the future work on kinematic constraints.

## Related subtopics

The first three fit in this paper as a paragraph each if space allows. The rest are follow-up papers.

| Subtopic | Fits where | What it adds | Data already in the repo? |
| --- | --- | --- | --- |
| Budget definition changes the ranking | §4.4 discussion | Evaluations vs CPU as a general benchmarking point [23]–[25] | Yes (popgen grid) |
| Population sizing at a fixed budget | §4.3, one sentence | Small P wins below about 10 000 evaluations | Yes (popgen) |
| Operator sensitivity | §3.4, one sentence | Crossover × mutation grid; 0.8 / 0.2 best | Yes (ga_ablation_retract) |
| Grading kinematic constraints | Follow-up | Penalty or repair on σ_min and reach, closing the T∞ plateau | Partly (sigma_case) |
| Feasibility rules vs repair | Follow-up | Deb's tournament [18] or stochastic ranking as a fifth arm | No |
| Joint layout and sequence optimisation | Follow-up | Process order as a permutation gene | No |
| Time-optimal or surrogate cycle time | Follow-up | Replace trapezoids with time-optimal timing [5] or a learned model [6] | No |
| Multi-objective layout | Follow-up | Cycle time vs footprint vs mean σ_min, NSGA-II [2] | No |
| Physics-in-the-loop acceptance | Follow-up | MuJoCo torque and tracking as constraints, not a post-check | Partly (run_mujoco) |

## References

The numbers match the citations above. Entries 1–30 were found through a Consensus literature search on 14 Oct 2026; check each against its DOI before submission. Entries 31–33 are from memory and are not yet verified. Entry 34 was checked against the PDF in the repo (`RepairGA-SA.pdf`). Entries 35–38 were found through a Consensus search on 19 Oct 2026 (abstracts only).

1. J. Zhang et al., [Genetic Algorithm for Robot Workcell Layout Problem](https://consensus.app/papers/details/8bb11f2bfef1559891b59c73f562da7b/?utm_source=claude_desktop), WRI World Congress on Software Engineering, 2009.
2. Z.-Y. Lim et al., [Nature inspired algorithms to optimize robot workcell layouts](https://consensus.app/papers/details/fb8273846c7255ec883610ad78152d87/?utm_source=claude_desktop), Applied Soft Computing, 2016.
3. R.-A. Sánchez-Sosa et al., [Robotic Cell Layout Optimization Using a Genetic Algorithm](https://consensus.app/papers/details/b0051adf12455b17a586043e2086b9a8/?utm_source=claude_desktop), Applied Sciences, 2024.
4. R.-A. Sánchez-Sosa et al., [Evolutionary Algorithms for the Optimal Design of Robotic Cells: A Dual Approximation for Space and Time](https://consensus.app/papers/details/08c031577d8257a98aa5b0dc10a23f88/?utm_source=claude_desktop), Applied Sciences, 2025.
5. J. Baumgärtner et al., [Beyond proxies: A direct time-optimal approach to robot cell layout optimization](https://consensus.app/papers/details/09472a93d465529cac9f38b839c8e8d9/?utm_source=claude_desktop), CIRP Annals, 2025.
6. T. Kawabe et al., [Surrogate-assisted motion planning and layout design of robotic cellular manufacturing systems](https://consensus.app/papers/details/076dcc077f25522b9c78b96eb575de05/?utm_source=claude_desktop), Eng. Appl. Artif. Intell., 2025.
7. J. Beck et al., [Robot Assembly Cell Layout Optimization Using Genetic Algorithm: Method and Evaluation](https://consensus.app/papers/details/03a775f1ffb45ac396ae457842dd8040/?utm_source=claude_desktop), Procedia CIRP, 2026.
8. S. Sim et al., [Optimisation of a Robotic Workcell Layout Using Genetic Algorithms](https://consensus.app/papers/details/2f964a6b879155588029a501bb411208/?utm_source=claude_desktop), ASME, 2005.
9. M. Anjos et al., [Mathematical optimization approaches for facility layout problems: The state-of-the-art and future research directions](https://consensus.app/papers/details/0546ec263dfe515e8c8649b09f573ada/?utm_source=claude_desktop), EJOR, 2017.
10. J. M. Palomo-Romero et al., [An island model genetic algorithm for unequal area facility layout problems](https://consensus.app/papers/details/90963bb27ecd5f0f823fbcf9fbfd7bb6/?utm_source=claude_desktop), Expert Syst. Appl., 2017.
11. F. G. Paes et al., [A hybrid genetic algorithm with decomposition phases for the Unequal Area Facility Layout Problem](https://consensus.app/papers/details/42478051256050d4b646858ac723b11f/?utm_source=claude_desktop), EJOR, 2017.
12. T. Yoshikawa, [Manipulability of Robotic Mechanisms](https://consensus.app/papers/details/abc27c956e4b5f6db54181011e377245/?utm_source=claude_desktop), Int. J. Robotics Research, 1985.
13. N. C. N. Doan et al., [Optimal robot placement with consideration of redundancy problem for wrist-partitioned 6R articulated robots](https://consensus.app/papers/details/76683eef7de856b580348c858bc3a9f8/?utm_source=claude_desktop), Robot. Comput.-Integr. Manuf., 2017.
14. S.-W. Son et al., [A convex programming approach to the base placement of a 6-DOF articulated robot with a spherical wrist](https://consensus.app/papers/details/f00286a755505917bdda90333bd2f3ad/?utm_source=claude_desktop), Int. J. Adv. Manuf. Technol., 2019.
15. Z. Zhao et al., [B*: Efficient and Optimal Base Placement for Fixed-Base Manipulators](https://consensus.app/papers/details/b48d1df846d954ca9f28a29d2ac66328/?utm_source=claude_desktop), IEEE RA-L, 2025.
16. F. Aggogeri et al., [6-DOFs Robot Placement Based on the Multi-Criteria Procedure for Industrial Applications](https://consensus.app/papers/details/6c3302116f36561099dfb1297792814f/?utm_source=claude_desktop), Robotics, 2024.
17. C. A. Coello Coello, [Theoretical and numerical constraint-handling techniques used with evolutionary algorithms: a survey of the state of the art](https://consensus.app/papers/details/2f5bc841d82b5d7eb0c593684da43384/?utm_source=claude_desktop), CMAME, 2002.
18. K. Deb, [An Efficient Constraint Handling Method for Genetic Algorithms](https://consensus.app/papers/details/87a74a21e3c05bc9b7466eb3b8e35800/?utm_source=claude_desktop), CMAME, 2000.
19. S. Salcedo-Sanz, [A survey of repair methods used as constraint handling techniques in evolutionary algorithms](https://consensus.app/papers/details/cb765f8289325016a9d9dade3b250b00/?utm_source=claude_desktop), Comput. Sci. Rev., 2009.
20. K. Miettinen et al., [Numerical Comparison of Some Penalty-Based Constraint Handling Techniques in Genetic Algorithms](https://consensus.app/papers/details/5c09bdbda7c556da8312b291689ec9ee/?utm_source=claude_desktop), J. Global Optim., 2003.
21. F. Samanipour et al., [Adaptive repair method for constraint handling in multi-objective genetic algorithm based on relationship between constraints and variables](https://consensus.app/papers/details/4d4bf367ccad569aa0d758b7dcd45230/?utm_source=claude_desktop), Applied Soft Computing, 2020.
22. C. A. Coello Coello et al., [Constraint-handling in genetic algorithms through the use of dominance-based tournament selection](https://consensus.app/papers/details/c71290fe375a55288ce3092aa7c7fee8/?utm_source=claude_desktop), Adv. Eng. Informatics, 2002.
23. A. Piotrowski et al., [Swarm Intelligence and Evolutionary Algorithms: Performance versus speed](https://consensus.app/papers/details/da19607946225ba391b7e7e5653e9991/?utm_source=claude_desktop), Information Sciences, 2017.
24. G. Lan et al., [Time Efficiency in Optimization with a Bayesian-Evolutionary Algorithm](https://consensus.app/papers/details/c32ef30b93da51c987f78e87783caba8/?utm_source=claude_desktop), Swarm Evol. Comput., 2020.
25. H. Ishibuchi et al., [Difficulties in Fair Performance Comparison of Multi-Objective Evolutionary Algorithms](https://consensus.app/papers/details/8fece76c322e5af993f144d6d6967d56/?utm_source=claude_desktop), IEEE Comput. Intell. Mag., 2022.
26. J. Kůdela, [A critical problem in benchmarking and analysis of evolutionary computation methods](https://consensus.app/papers/details/419d65746abd512bac45a92df0629ea6/?utm_source=claude_desktop), Nature Machine Intelligence, 2022.
27. I. Rahimi et al., [A Review on Constraint Handling Techniques for Population-based Algorithms](https://consensus.app/papers/details/19ca1122f14e50338e42d93bf72c98de/?utm_source=claude_desktop), Arch. Comput. Methods Eng., 2022.
28. N. Vahrenkamp et al., [Representing the robot's workspace through constrained manipulability analysis](https://consensus.app/papers/details/be91dbbefe165a928a8646e08e9254ef/?utm_source=claude_desktop), Autonomous Robots, 2014.
29. I. B. Hunagund et al., [A survey on discrete space and continuous space facility layout problems](https://consensus.app/papers/details/efa7414de3fa5c27937e54e8516c640c/?utm_source=claude_desktop), J. Facilities Management, 2021.
30. J. Liu et al., [Multi-objective particle swarm optimization algorithm based on objective space division for the unequal-area facility layout problem](https://consensus.app/papers/details/d93d03be8a705c85b73ea949ece39c8c/?utm_source=claude_desktop), Expert Syst. Appl., 2018.
31. E. Todorov, T. Erez, Y. Tassa, "MuJoCo: A physics engine for model-based control," IROS, 2012. *(from memory, verify)*
32. B. Armstrong, O. Khatib, J. Burdick, "The explicit dynamic model and inertial parameters of the PUMA 560 arm," ICRA, 1986. *(from memory, verify)*
33. P. Corke, *Robotics, Vision and Control*, Springer (Puma 560 kinematic parameters). *(from memory, verify edition)*
34. L. Zhang, G. Yang, B. Pan et al., "Synergistic optimization of power matching and layout for fracturing–drilling equipment in extremely limited spaces via a hybrid GA–SA algorithm," *Computational Geosciences*, vol. 30, art. 36, 2026. [doi:10.1007/s10596-026-10430-9](https://doi.org/10.1007/s10596-026-10430-9)
35. T. Bachmann et al., [Automated Planning of Workcell Layouts Considering Task Sequences](https://consensus.app/papers/details/a6e4be8722ab5a6e93bb66a9480d9ae9/?utm_source=claude_desktop), IEEE ICRA, 2021.
36. B. Xiang et al., [Robot Workcell Layout Optimization Based on Genetic Algorithm](https://consensus.app/papers/details/1ead84f802f45af99a450ca362d1f153/?utm_source=claude_desktop), J. Shanghai Jiaotong Univ., 2008.
37. B. Qiu et al., [Concurrent layout and trajectory optimization for robot workcell toward energy-efficient and collision-free automation](https://consensus.app/papers/details/2633daa12b3c5c94a5cc3d63e8f3e498/?utm_source=claude_desktop), Int. J. Adv. Manuf. Technol., 2022.
38. D. Xu et al., [Computer-Assisted Optimization of Spatial Layout in Dual-Robot Systems for Upper Limb Rehabilitation Using Genetic Algorithms](https://consensus.app/papers/details/6435bad7f59459f1ae95dba9a1d2d4c6/?utm_source=claude_desktop), Int. J. High Speed Electron. Syst., 2025.
