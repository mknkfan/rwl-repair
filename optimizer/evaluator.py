"""The objective: cycle time of a layout, and the tests deciding whether a
layout has one at all.

**Feasibility** needs all five.  The first two are geometric and cheap; the
rest need the robot and are reached only once those pass, which is what keeps
60 000 evaluations per run affordable.

1. **Placement** -- every footprint inside the cell, clear of the robot's
   column by ``keep_out_radius``, and ``clearance`` apart from every other.
2. **Reachability** -- the arm can put its tool on every loading port with a
   vertical approach, inside every joint's travel, and can retract from it.
3. **Conditioning** -- ``sigma_min`` of the position Jacobian at least
   ``sigma_min_threshold`` at every port.  The anti-singularity test: it
   rejects poses with the elbow locked out or folded, near which the joint
   rates for a given tool motion blow up.
4. **Clearance** -- no part of the robot within ``arm_clearance`` of any
   machine, at every station and (with ``check_path``) at samples along every
   move.  The arm is bounded by :func:`armmesh.collision_spheres`, measured
   off the solid model that gets drawn, so a layout that passes cannot show
   the robot inside a box.
5. **One posture for the whole cycle** -- a single IK branch must satisfy 2-4
   at *every* station.  A real cell does not reconfigure mid-cycle, and
   without this the optimiser buys cycle time with elbow flips no controller
   would execute.

**The motion.** Each station is bracketed by a *lift pose*, the same tool
position raised ``retract`` metres above the tallest machine, so the cycle is
descend / dwell / retract / traverse.  Not decoration: with direct
point-to-point moves the tool interpolates in joint space between ports and
drags the forearm through whatever stands between them, making criterion 4
over the whole cycle *unsatisfiable* in a cell this tight -- the GA finds no
feasible layout at all.  With the retract, 99 % of the layouts that clear the
stations clear the cycle.  ``retract=None`` restores the direct moves.

Anything that fails scores ``+inf`` -- a hard wall rather than a penalty,
because a penalty needs a scale relating metres of overlap to seconds, and
any such choice is an unstated preference that shows up in the answer.
(:mod:`penalty` builds the graded alternative and measures the difference.)

**The objective** is the fastest branch that passes -- the cell would be
commissioned in its best working posture.  Cycle time is the trapezoidal total
from :mod:`trajectory`: home, every station in process order with its lift
poses either side, back home, plus a dwell at each station.

``n_evals`` counts calls to :meth:`fitness`, so the evaluation budget is one
currency regardless of how much work each call did.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from armmesh import collision_spheres
from geometry import (
    Machine,
    box_frames,
    layout_is_placeable,
    points_over_boxes,
    points_to_boxes_distance,
)
from problems import Instance
from robot import BRANCHES, BRANCH_NAMES, Puma560Arm
from trajectory import cycle_time, plan_cycle

#: Why a layout was rejected -- reported in the results so a run's failures
#: can be attributed rather than guessed at.
REJECTIONS: Tuple[str, ...] = (
    "placement", "reach", "limits", "sigma_min", "clearance", "path", "none",
)


@dataclass
class LayoutOutcome:
    """Everything the evaluator learned about one layout.

    :meth:`CycleTimeEvaluator.fitness` keeps only the cycle time; the
    reporting and plotting code asks for this instead.
    """

    feasible: bool
    cycle_time: float
    rejection: str
    branch: Optional[Tuple[int, int]] = None
    configurations: Optional[np.ndarray] = None
    sigma_min: Optional[np.ndarray] = None
    waypoints: Optional[np.ndarray] = None
    machines: Optional[List[Machine]] = None
    #: Indices into ``configurations`` that are stations -- where the arm
    #: dwells.  With a retract the tour also holds a lift pose either side of
    #: each station, and those are passed through, not held at.
    station_index: Optional[np.ndarray] = None

    @property
    def branch_name(self) -> str:
        return BRANCH_NAMES.get(self.branch, "-") if self.branch else "-"

    @property
    def stations(self) -> Optional[np.ndarray]:
        """The station configurations alone, without home or the lift poses."""
        if self.configurations is None:
            return None
        if self.station_index is None:
            return self.configurations[1:-1]
        return self.configurations[self.station_index]


class CycleTimeEvaluator:
    """Chromosome -> cycle time, with the feasibility wall in front of it."""

    def __init__(
        self,
        instance: Instance,
        arm: Optional[Puma560Arm] = None,
        sigma_min_threshold: float = 0.14,
        dwell: float = 0.30,
        check_path: bool = True,
        path_samples: int = 12,
        check_arm: bool = True,
        arm_clearance: float = 0.005,
        retract: float = 0.06,
    ):
        self.instance = instance
        self.arm = arm or Puma560Arm(base=instance.base_xy)
        self.sigma_min_threshold = float(sigma_min_threshold)
        self.dwell = float(dwell)
        self.check_path = bool(check_path)
        self.path_samples = int(path_samples)
        self.check_arm = bool(check_arm)
        self.arm_clearance = float(arm_clearance)
        self.n_evals = 0

        #: Spheres bounding the drawn solid, for the clearance test.
        self._spheres = collision_spheres(self.arm)

        # Only machine heights enter this, and those are fixed by the
        # instance, so it is one number per instance rather than per layout.
        #: Metres above the tallest machine that the tool retracts to.  Zero
        #: (or ``None``) restores the direct point-to-point moves.
        self.retract = None if retract is None else float(retract)
        self.retract_height: Optional[float] = None
        if self.retract:
            self.retract_height = (
                max(m.height for m in instance.machines) + self.retract
            )

        _, _, self.velocity_limit, self.acceleration_limit = self.arm.limits.as_arrays()

        # The home pose is a fixed task point, so its IK depends on the branch
        # but not on the layout: solve it once per branch here rather than
        # 60 000 times inside the run.
        home = np.asarray(instance.home_point, dtype=float)[None, :]
        self._home: Dict[Tuple[int, int], Optional[np.ndarray]] = {}
        for branch in BRANCHES:
            q, ok = self.arm.ik_vertical(home, *branch)
            usable = bool(ok[0]) and bool(self.arm.within_limits(q)[0])
            usable = usable and float(self.arm.sigma_min(q)[0]) >= self.sigma_min_threshold
            self._home[branch] = q[0] if usable else None
        if all(value is None for value in self._home.values()):
            raise ValueError(
                f"home point {instance.home_point} is not reachable in any posture; "
                "the instance is misconfigured"
            )

    # -- decoding ------------------------------------------------------

    def decode(self, chromosome: np.ndarray) -> List[Machine]:
        """``[x, y, rotation] * n`` -> placed machines."""
        genes = np.asarray(chromosome, dtype=float).reshape(-1, 3)
        return [
            machine.copy_at(row[0], row[1], row[2])
            for machine, row in zip(self.instance.machines, genes)
        ]

    def reset_counter(self) -> None:
        self.n_evals = 0

    # -- the objective -------------------------------------------------

    def fitness(self, chromosome: np.ndarray) -> float:
        """Cycle time in seconds, or ``+inf`` for an infeasible layout."""
        self.n_evals += 1
        return self.evaluate(self.decode(chromosome)).cycle_time

    def evaluate(self, machines: Sequence[Machine]) -> LayoutOutcome:
        """The full outcome for a placed layout, tests in cost order."""
        instance = self.instance

        if not layout_is_placeable(
            machines,
            instance.bounds,
            instance.base_xy,
            instance.keep_out_radius,
            instance.clearance,
        ):
            return LayoutOutcome(False, math.inf, "placement", machines=list(machines))

        # home -> every port in process order -> home
        ports = np.stack([machines[i].port() for i in instance.sequence])

        best: Optional[LayoutOutcome] = None
        # When every branch fails, report the stage the *most successful*
        # branch reached, not whichever happened to be tried last -- that is
        # the one that says what actually stood in the way.
        furthest = 0
        boxes = None
        for branch in BRANCHES:
            home_q = self._home[branch]
            if home_q is None:
                continue

            q, reachable = self.arm.ik_vertical(ports, *branch)
            if not reachable.all():
                continue
            if not self.arm.within_limits(q).all():
                furthest = max(furthest, 1)
                continue
            sigma = self.arm.sigma_min(q)
            if float(sigma.min()) < self.sigma_min_threshold:
                furthest = max(furthest, 2)
                continue

            built = self._build_tour(home_q, q, ports, branch)
            if built is None:
                # No lift pose over some port: the arm cannot get above it and
                # still be inside its travel.  That is a reach failure, and it
                # is the retract that causes it, so it is reported as one.
                continue
            tour, station_index = built

            # Cycle time is cheap and clearance is not, so time the branch
            # first: one that cannot beat the best branch so far cannot change
            # the answer, and its clearance need not be tested.  Only a
            # feasible branch sets ``best``, so the rejection reported for an
            # infeasible layout is unaffected.
            total = cycle_time(tour, self.velocity_limit, self.acceleration_limit,
                               self.dwell, station_index)
            if best is not None and total >= best.cycle_time:
                continue

            if boxes is None:
                boxes = box_frames(machines)
            if self.check_arm and self._arm_hits_a_machine(q, boxes):
                furthest = max(furthest, 3)
                continue
            if self.check_path and self._path_hits_a_machine(tour, station_index, boxes):
                furthest = max(furthest, 4)
                continue
            if best is None or total < best.cycle_time:
                best = LayoutOutcome(
                    feasible=True,
                    cycle_time=total,
                    rejection="none",
                    branch=branch,
                    configurations=tour,
                    sigma_min=sigma,
                    waypoints=ports,
                    machines=list(machines),
                    station_index=station_index,
                )

        if best is None:
            rejection = ("reach", "limits", "sigma_min", "clearance", "path")[furthest]
            return LayoutOutcome(
                False, math.inf, rejection, waypoints=ports, machines=list(machines)
            )
        return best

    # -- the tour ------------------------------------------------------

    def _build_tour(
        self,
        home_q: np.ndarray,
        stations: np.ndarray,
        ports: np.ndarray,
        branch: Tuple[int, int],
    ) -> Optional[Tuple[np.ndarray, np.ndarray]]:
        """The whole cycle as configurations, plus which of them are stations.

        Without a retract, ``home -> station -> ... -> home``, the tool taking
        whatever curve joint-space interpolation gives it between two ports.
        With one, each station is bracketed by a lift pose at
        :attr:`retract_height`, so the traverse happens with the whole arm
        above everything it could hit.

        ``None`` when a lift pose is unreachable or outside the joint travel:
        a layout the robot cannot retract from is not one it can run.
        """
        n = stations.shape[0]
        if not self.retract_height:
            tour = np.vstack([home_q[None, :], stations, home_q[None, :]])
            return tour, np.arange(1, n + 1)

        above = ports.copy()
        above[:, 2] = self.retract_height
        lifts, reachable = self.arm.ik_vertical(above, *branch)
        if not reachable.all() or not self.arm.within_limits(lifts).all():
            return None
        # sigma_min is deliberately *not* required at a lift pose: it is a via
        # point the arm passes through, not one it has to control fine motion
        # at, and imposing it there would reject layouts for a condition the
        # task never encounters.

        rows = [home_q]
        station_index = []
        for k in range(n):
            rows.append(lifts[k])
            station_index.append(len(rows))
            rows.append(stations[k])
            rows.append(lifts[k])
        rows.append(home_q)
        return np.stack(rows), np.asarray(station_index, dtype=int)

    # -- clearance between the robot and the machines --------------------

    def arm_clearances(
        self, configurations: np.ndarray, machines: Sequence[Machine]
    ) -> np.ndarray:
        """Gap between the robot and every machine, at every configuration.

        ``configurations`` is ``(n, 4)``; the result is ``(n, n_machines)`` in
        metres, negative where the arm is *inside* that machine.  The reporting
        code uses it to say how much margin a layout actually has; the
        feasibility test below is the same computation with an early exit.
        """
        return self._clearances(np.atleast_2d(configurations), box_frames(machines))

    def _clearances(
        self, configurations: np.ndarray, boxes: Tuple[np.ndarray, ...]
    ) -> np.ndarray:
        """The worst gap between the arm and each machine, per configuration.

        The arm is bounded by :func:`armmesh.collision_spheres`, measured off
        the drawn solid, so a layout that clears here cannot show the robot
        inside a box in any figure.

        One exemption: the wrist and gripper spheres are ignored for a machine
        the tool is *directly above* -- the vertical approach corridor.  The
        tool tip rests on the top surface at a station, so its sphere must dip
        below it, and counting that would make every legal pick a collision.
        Nothing else is exempt -- the gripper still clears every other
        machine, and the forearm and upper arm clear the served one too.
        """
        centres, half, rotations = boxes
        points = self.arm.points_along_arm(configurations, self._spheres.coefficients)
        distance = points_to_boxes_distance(points, centres, half, rotations)
        gap = distance - self._spheres.radii[None, :, None]

        tips = self.arm.fk(configurations)[:, :3]
        # (n_config, n_machines) -> broadcast over the spheres of each config.
        approaching = points_over_boxes(tips, centres, half, rotations)
        exempt = approaching[:, None, :] & self._spheres.is_tool[None, :, None]
        gap = np.where(exempt, np.inf, gap)
        return gap.min(axis=1)

    def _arm_hits_a_machine(
        self, configurations: np.ndarray, boxes: Tuple[np.ndarray, ...]
    ) -> bool:
        """Is any part of the robot inside a machine, or too close to one?

        The same test as :meth:`_clearances`, with a broad phase in front.  A
        sphere whose underside is above the tallest machine's top face by at
        least the clearance cannot be too close to any machine -- its distance
        to a box is at least its height above that box's top -- so only the
        rest go through the exact box distance.  During a lifted traverse that
        is most of the arm.  The ``1e-9`` keeps the cull clear of round-off,
        so the answer is the one the full test gives, not an approximation.
        """
        centres, half, rotations = boxes
        points = self.arm.points_along_arm(configurations, self._spheres.coefficients)
        radii = self._spheres.radii
        top = float((centres[:, 2] + half[:, 2]).max())
        near = points[..., 2] - radii < top + self.arm_clearance + 1e-9
        if not near.any():
            return False
        config, sphere = np.nonzero(near)
        gap = points_to_boxes_distance(points[config, sphere], centres, half, rotations)
        gap -= radii[sphere][:, None]

        # The approach-corridor exemption, as in _clearances.
        tool = self._spheres.is_tool[sphere]
        if tool.any():
            tips = self.arm.fk(configurations[config[tool]])[:, :3]
            approaching = points_over_boxes(tips, centres, half, rotations)
            gap[tool] = np.where(approaching, np.inf, gap[tool])
        return bool((gap < self.arm_clearance).any())

    def _path_hits_a_machine(
        self, tour: np.ndarray, station_index: np.ndarray,
        boxes: Tuple[np.ndarray, ...],
    ) -> bool:
        """Does the robot sweep through a machine between waypoints?

        Moves interpolate in *joint* space, so the tool follows a curve and
        the forearm swings above it: clearance at the endpoints says nothing
        about the middle.  This samples every segment and applies the same
        whole-arm test.  Endpoints are excluded -- they are the waypoints,
        already covered at the stations.

        Every segment is sampled at once and tested in one broadcast: the
        arrays are small, so a call per segment spends its time on numpy's
        per-call overhead rather than on the arithmetic.
        """
        fractions = np.linspace(0.0, 1.0, self.path_samples + 2)[1:-1]
        q0 = tour[:-1]
        samples = q0[:, None, :] + fractions[None, :, None] * (tour[1:] - q0)[:, None, :]
        return self._arm_hits_a_machine(samples.reshape(-1, tour.shape[1]), boxes)

    # -- reporting helpers ---------------------------------------------

    def segment_times(self, outcome: LayoutOutcome) -> np.ndarray:
        """Duration of each move in a feasible cycle.

        One entry per segment of the tour, so with a retract there are three
        per station (descend, lift, traverse) rather than one.
        """
        if not outcome.feasible or outcome.configurations is None:
            return np.array([])
        segments = plan_cycle(
            outcome.configurations, self.velocity_limit, self.acceleration_limit
        )
        return np.array([segment.duration for segment in segments])


def feasibility_rate(
    instance: Instance,
    samples: int = 2000,
    seed: int = 7,
    sigma_min_threshold: float = 0.14,
    check_path: bool = False,
    check_arm: bool = True,
) -> Dict[str, float]:
    """How hard the instance is: what share of random layouts survives each test.

    Measures how much the robot narrows the problem beyond packing alone.
    """
    from problems import random_layout

    evaluator = CycleTimeEvaluator(
        instance, sigma_min_threshold=sigma_min_threshold, check_path=check_path,
        check_arm=check_arm,
    )
    rng = np.random.default_rng(seed)
    counts = {name: 0 for name in REJECTIONS}
    for _ in range(samples):
        outcome = evaluator.evaluate(evaluator.decode(random_layout(instance, rng)))
        counts[outcome.rejection] += 1
    return {
        "feasible_rate": counts["none"] / samples,
        "rejected_placement": counts["placement"] / samples,
        "rejected_reach": counts["reach"] / samples,
        "rejected_limits": counts["limits"] / samples,
        "rejected_sigma_min": counts["sigma_min"] / samples,
        "rejected_clearance": counts["clearance"] / samples,
        "rejected_path": counts["path"] / samples,
    }
