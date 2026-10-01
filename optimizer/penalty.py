"""Graded penalties on the geometric constraints, as an alternative to ``+inf``.

:class:`~evaluator.CycleTimeEvaluator` scores every infeasible layout ``+inf``,
so selection sees one flat plateau and can only rank the layouts that already
work.  A penalty puts a slope on that plateau: each geometric constraint is
written ``g_i(x) <= 0`` and the *constraint violation degree*
``CVD(x) = sum_i max(0, g_i(x))`` ranks an infeasible layout by how close to
legal it is.

The score is *layered*, so the penalty never trades seconds against metres::

    F(x) = T(x)                                  x fully feasible
         = T_inf + w * CVD(x) + m * N(x)         otherwise

``T_inf`` is above any attainable cycle time, so every feasible layout beats
every infeasible one and cycle time stays the objective.  Three consequences:

1. ``w`` cannot affect the search in the CVD-only variant.  Selection,
   elitism and best-so-far are all rank-based, and scaling ``CVD`` by a
   positive constant leaves every ranking unchanged.  That disposes of the
   usual objection to penalties -- the metres-to-seconds exchange rate is an
   unstated preference -- because the layers never let the two trade.
2. ``m/w`` *is* material: it decides whether several shallow overlaps rank
   above or below one deep one.  A real modelling choice, so it is swept.
3. A placeable-but-unreachable layout scores exactly ``T_inf``.  Grading the
   robot constraints would cost a full IK pass per individual, so this grades
   what repair repairs and nothing else.

The constraints are read straight off the functions the feasibility test uses,
so the penalty cannot drift from the thing it relaxes.  For ``n`` machines:
``4n`` bounds excursions, ``n`` keep-out shortfalls, ``n(n-1)/2`` overlap
depths -- 45 for the six-machine instances.  ``CVD`` is zero exactly when
``layout_is_placeable`` is true.

Both evaluators also count how many calls landed on a genuinely feasible
layout.  That share is the one feasibility measure meaning the same thing in
every arm, since ``ga.py``'s ``feasible_fraction`` is ``isfinite(fitness)``,
which a penalty makes identically 1.0.
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

    ``CVD`` is ``sum_i max(0, g_i)`` in metres, zero exactly when
    :func:`~geometry.layout_is_placeable` is true -- which is what makes it a
    relaxation of that test rather than a different test that correlates.
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
            # Not redundant: with a non-zero clearance the two disagree on
            # about one layout in 3000.  ``overlap_depth`` maximises the
            # projection gap over the four edge normals, which understates the
            # distance for boxes nearest corner-to-corner, so it reports a few
            # millimetres of violation on a pair the feasibility test calls
            # clear.  Deferring to that test keeps ``CVD == 0`` and
            # ``layout_is_placeable`` the same predicate.
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

    ``violation_weight`` scales ``CVD``; ``overlap_count_weight`` scales the
    number of machines caught in an overlap.  Zero for the second gives the
    pure CVD variant -- one class, because they differ only in a coefficient.
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
        value = super().fitness(chromosome)
        # Every infeasible score is at least the base, every feasible one
        # below it, so this counts exactly the calls that reached a feasible
        # layout -- cached or not.
        if value < self.infeasible_base:
            self.n_feasible_evals += 1
        return value

    def _score(self, chromosome: np.ndarray) -> float:
        instance = self.instance
        machines = self.decode(chromosome)

        cvd, n_machines_overlapping, _pairs = constraint_violations(
            machines, instance.bounds, instance.base_xy,
            instance.keep_out_radius, instance.clearance,
        )
        if cvd > 0.0:
            # Geometrically illegal: graded, and never evaluated on the
            # robot -- so this arm's cost profile stays comparable with the
            # ``+inf`` arm, both stopping before the kinematics.
            return (
                self.infeasible_base
                + self.violation_weight * cvd
                + self.overlap_count_weight * n_machines_overlapping
            )

        outcome = self.evaluate(machines)
        if not outcome.feasible:
            # Legal boxes, impossible robot: the flat part of the landscape
            # the penalty does not reach.
            return self.infeasible_base

        if outcome.cycle_time >= self.infeasible_base:
            raise ValueError(
                f"cycle time {outcome.cycle_time:.3f}s reached the infeasible "
                f"floor {self.infeasible_base:.1f}s; raise INFEASIBLE_BASE or "
                "the ranking is inverted"
            )
        return float(outcome.cycle_time)


class CountingEvaluator(CycleTimeEvaluator):
    """The ordinary ``+inf`` evaluator, plus the feasible-evaluation counter.

    The ``+inf`` arms use this rather than the base class so the feasible
    share is computed by identical code everywhere; a metric defined
    differently per arm is not a comparison.
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
