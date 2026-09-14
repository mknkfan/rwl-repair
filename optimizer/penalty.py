"""Graded penalties on the geometric constraints, as an alternative to the
``+inf`` wall.

The wall
--------
:class:`~evaluator.CycleTimeEvaluator` scores every infeasible layout ``+inf``.
That is a *death penalty*: the individual survives in the population but
carries no information about how badly it missed, so selection sees one flat
plateau covering most of the search space and can only rank the layouts that
already work.  The repair operator attacks the same problem from the other
side, by moving the individual until it is legal.

What a penalty does instead is put a slope on the plateau.  Each geometric
constraint is written as ``g_i(x) <= 0`` and the *constraint violation degree*

    CVD(x) = sum_i max(0, g_i(x))

is added to the score, so an infeasible layout is now ranked by how close to
legal it is and selection can walk downhill towards the feasible set without
ever having landed in it.

The formulation
---------------
The objective is cycle time and it stays the objective; the penalty is never
allowed to trade seconds against metres.  So the score is *layered*::

    F(x) = T(x)                                  x fully feasible
         = T_inf + w * CVD(x) + m * N(x)         otherwise

with ``T_inf`` a constant above any attainable cycle time.  Every feasible
layout therefore beats every infeasible one, whatever the weights, and among
the infeasible ones the ranking is by violation.  Three consequences worth
being explicit about:

1. **The layers make the weight scale immaterial for the CVD-only variant.**
   The GA's selection is tournament, its elitism is ``argsort``, and its
   best-so-far is a ``min`` -- all rank-based.  Multiplying ``CVD`` by any
   positive ``w`` leaves every one of those unchanged, so ``w`` cannot affect
   the search.  This is what disposes of the standard objection to penalties
   (that the metres-to-seconds exchange rate is an unstated preference): here
   there is no exchange rate, because the layers never let the two trade.
2. **The ratio m/w is not immaterial** in the second variant, because it
   decides whether several shallow overlaps rank above or below one deep one.
   That is a real modelling choice and it is swept rather than asserted.
3. **A placeable-but-unreachable layout scores exactly** ``T_inf``.  Its CVD is
   zero -- the geometry is legal -- and the robot constraints are not penalised
   here.  They could be, but grading them costs the full inverse-kinematics
   pass on every individual, which is far more than a placement test on this
   problem and would make the arm incomparable on compute.  So this grades
   what repair repairs, and nothing else.

The constraints
---------------
All in metres, all read straight off the functions the feasibility test
already uses, so the penalty cannot drift away from the thing it is a
relaxation of.  For ``n`` machines::

    4n        bounds     how far the footprint juts past each of the four walls
    n         keep-out   keep_out_shortfall: how far into the robot's column
    n(n-1)/2  overlap    overlap_depth: the minimum translation separating a
                         pair, clearance included

45 constraints for the six-machine instances.  ``CVD`` is their positive parts
summed, so it is zero exactly when ``layout_is_placeable`` is true.

Counting
--------
Both evaluators count a call to :meth:`fitness` as one evaluation, as the base
class does, and additionally count how many of those calls landed on a
genuinely feasible layout.  The share is the one feasibility measure that
means the same thing in all four arms: the ``feasible_fraction`` trace in
``ga.py`` is ``isfinite(fitness)``, which is identically 1.0 once a penalty
makes every score finite.
"""

from __future__ import annotations

import math
from typing import Sequence, Tuple

import numpy as np

from evaluator import CycleTimeEvaluator
from geometry import (
    Machine,
    keep_out_shortfall,
    overlap_depth,
    rectangles_overlap,
)
from problems import Instance

#: Score floor for an infeasible layout.  Any constant above every attainable
#: cycle time does; the value is checked against the objective at runtime, so
#: a mis-set floor is an exception rather than a silently inverted ranking.
INFEASIBLE_BASE = 1000.0


def constraint_violations(
    machines: Sequence[Machine],
    bounds: Tuple[float, float, float, float],
    base_xy: Sequence[float],
    keep_out_radius: float,
    clearance: float = 0.0,
) -> Tuple[float, int, int]:
    """``(CVD, machines_overlapping, overlapping_pairs)`` for one layout.

    ``CVD`` is ``sum_i max(0, g_i)`` over every geometric constraint, in
    metres.  It is zero exactly when :func:`~geometry.layout_is_placeable`
    returns ``True``, which is what makes it a relaxation of that test rather
    than a different test that happens to correlate with it.
    """
    min_x, max_x, min_y, max_y = bounds
    total = 0.0

    for machine in machines:
        corners = machine.corners()
        # Four walls, each a separate inequality: a machine that is out on two
        # sides at once is twice as wrong as one that is out on one.
        total += max(0.0, min_x - corners[:, 0].min())
        total += max(0.0, corners[:, 0].max() - max_x)
        total += max(0.0, min_y - corners[:, 1].min())
        total += max(0.0, corners[:, 1].max() - max_y)
        total += keep_out_shortfall(machine, base_xy, keep_out_radius)

    involved = set()
    pairs = 0
    for i in range(len(machines)):
        for j in range(i + 1, len(machines)):
            # The guard is what keeps ``CVD == 0`` and ``layout_is_placeable``
            # the same predicate, and it is not redundant: with a non-zero
            # clearance the two disagree on roughly one layout in 3000.
            # ``rectangles_overlap`` short-circuits on a bound on the true
            # distance between the footprints, while ``overlap_depth`` maximises
            # the projection gap over the four edge normals -- and for two boxes
            # nearest corner-to-corner that maximum understates the distance, so
            # ``overlap_depth`` reports a few millimetres of violation on a pair
            # the study's own test calls clear.  Deferring to the test means the
            # penalty is a relaxation of the constraint the other arms enforce,
            # rather than of a slightly stricter one of its own.
            if not rectangles_overlap(machines[i], machines[j], clearance):
                continue
            depth, _ = overlap_depth(machines[i], machines[j], clearance)
            if depth > 0.0:
                total += depth
                involved.update((i, j))
                pairs += 1

    return total, len(involved), pairs


class PenaltyEvaluator(CycleTimeEvaluator):
    """Cycle time with the geometric plateau replaced by a slope.

    ``violation_weight`` scales ``CVD`` and ``overlap_count_weight`` scales the
    number of machines caught in an overlap.  Setting the second to zero gives
    the pure CVD variant; the two are one class because they differ in a
    coefficient, and making them two would invite them to drift apart.
    """

    def __init__(
        self,
        instance: Instance,
        *,
        violation_weight: float = 1.0,
        overlap_count_weight: float = 0.0,
        infeasible_base: float = INFEASIBLE_BASE,
        **kwargs,
    ):
        super().__init__(instance, **kwargs)
        self.violation_weight = float(violation_weight)
        self.overlap_count_weight = float(overlap_count_weight)
        self.infeasible_base = float(infeasible_base)
        self.n_feasible_evals = 0

    def reset_counter(self) -> None:
        super().reset_counter()
        self.n_feasible_evals = 0

    def fitness(self, chromosome: np.ndarray) -> float:
        self.n_evals += 1
        instance = self.instance
        machines = self.decode(chromosome)

        cvd, n_machines_overlapping, _pairs = constraint_violations(
            machines, instance.bounds, instance.base_xy,
            instance.keep_out_radius, instance.clearance,
        )
        if cvd > 0.0:
            # Geometrically illegal: graded, and never evaluated on the robot.
            # This is the branch that keeps the arm's cost profile comparable
            # with the death-penalty arm -- both stop before the kinematics.
            return (
                self.infeasible_base
                + self.violation_weight * cvd
                + self.overlap_count_weight * n_machines_overlapping
            )

        outcome = self.evaluate(machines)
        if not outcome.feasible:
            # Legal boxes, impossible robot.  Nothing here is graded, so this
            # is the flat part of the landscape the penalty does not reach.
            return self.infeasible_base

        if outcome.cycle_time >= self.infeasible_base:
            raise ValueError(
                f"cycle time {outcome.cycle_time:.3f}s reached the infeasible "
                f"floor {self.infeasible_base:.1f}s; raise INFEASIBLE_BASE or "
                "the ranking is inverted"
            )
        self.n_feasible_evals += 1
        return float(outcome.cycle_time)


class CountingEvaluator(CycleTimeEvaluator):
    """The ordinary ``+inf`` evaluator, plus the feasible-evaluation counter.

    The baseline and repair arms use this rather than the base class so that
    the feasible-evaluation share is computed by identical code in all four
    arms; a metric that is defined differently per arm is not a comparison.
    """

    def __init__(self, instance: Instance, **kwargs):
        super().__init__(instance, **kwargs)
        self.n_feasible_evals = 0

    def reset_counter(self) -> None:
        super().reset_counter()
        self.n_feasible_evals = 0

    def fitness(self, chromosome: np.ndarray) -> float:
        value = super().fitness(chromosome)
        if math.isfinite(value):
            self.n_feasible_evals += 1
        return value
