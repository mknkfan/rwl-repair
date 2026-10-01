"""The instance family: workcells sized to a Puma 560.

The cell is sized to the robot, which is what makes reachability a real
constraint rather than one always or never satisfied.  A Puma 560 reaches
0.864 m from the shoulder; the floor is 1.4 m square, machines are benchtop
units 0.10-0.18 m across and 0.10-0.40 m tall, and the arm stands in the
middle on a 0.67 m pedestal.  The corners sit at radius 0.99 m, outside the
envelope, so they are genuinely unusable.

Only an annulus of the floor is workable: a top-down pick is possible for port
radii of roughly 0.16-0.71 m at the lowest machine height and 0.21-0.85 m at
the highest, and ``sigma_min >= 0.14`` removes a further 19 % of those poses
-- the outer band where the elbow approaches lock-out.  The middle is the
robot's own column.

Instances vary in machine footprints, heights, port positions and process
order.  The cell size and the robot do not vary, and within one study neither
does the machine count, so cycle times are comparable across instances of a
split.  The count is six unless a study asks for another
(:func:`build_instances`); cycle times at different counts are not comparable,
since more machines means more stations per cycle.  Splits are drawn from
disjoint seed ranges, so an instance used to develop the operators is never
one that is reported on.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Dict, List, Sequence, Tuple

import numpy as np

from geometry import Bounds, Machine, layout_is_placeable

#: Seed ranges, disjoint so no instance appears in two splits.
_SPLIT_SEEDS: Dict[str, Tuple[int, int]] = {
    "test": (1000, 1010),
    "train": (2000, 2015),
    "generalization": (4000, 4006),
}

ALL_SPLITS: Tuple[str, ...] = tuple(_SPLIT_SEEDS)


@dataclass
class Instance:
    """One workcell: the machines, the order they are served, and the cell."""

    name: str
    split: str
    index: int
    machines: List[Machine]
    sequence: List[int]
    bounds: Bounds
    base_xy: Tuple[float, float] = (0.0, 0.0)
    keep_out_radius: float = 0.18
    clearance: float = 0.02
    #: Tool-tip position the cycle starts and ends at.
    home_point: Tuple[float, float, float] = (0.40, 0.0, 0.45)

    @property
    def n_machines(self) -> int:
        return len(self.machines)

    def gene_bounds(self) -> Tuple[np.ndarray, np.ndarray]:
        """Per-gene box for ``[x, y, rotation]`` of every machine.

        Positions are inset by each machine's half-diagonal, so any rotation
        of a machine placed inside the box still has its footprint inside the
        cell and the position and rotation genes stay independent.
        """
        min_x, max_x, min_y, max_y = self.bounds
        lower: List[float] = []
        upper: List[float] = []
        for machine in self.machines:
            radius = machine.footprint_radius
            lower.extend([min_x + radius, min_y + radius, 0.0])
            upper.extend([max_x - radius, max_y - radius, 360.0])
        return np.asarray(lower, dtype=float), np.asarray(upper, dtype=float)


def build_instance(seed: int, name: str, split: str, index: int, n_machines: int = 6) -> Instance:
    """One reproducible instance from a seed."""
    rng = np.random.default_rng(seed)

    machines: List[Machine] = []
    for k in range(n_machines):
        width = float(rng.uniform(0.10, 0.18))
        depth = float(rng.uniform(0.10, 0.18))
        height = float(rng.uniform(0.10, 0.40))
        # The port sits off-centre on the top face, so a machine's rotation
        # changes where the robot must reach and the rotation gene matters.
        port = (0.30 * width, 0.0)
        machines.append(
            Machine(
                name=f"M{k + 1}",
                width=width,
                depth=depth,
                height=height,
                port_local=port,
            )
        )

    sequence = list(rng.permutation(n_machines))
    return Instance(
        name=name,
        split=split,
        index=index,
        machines=machines,
        sequence=[int(s) for s in sequence],
        bounds=(-0.70, 0.70, -0.70, 0.70),
    )


#: The machine count every study so far has used.
DEFAULT_N_MACHINES = 6


def build_instances(split: str = "test", n_machines: int = DEFAULT_N_MACHINES) -> List[Instance]:
    """Every instance of a split, in a fixed order.

    ``n_machines`` changes the cell, not the seeds: instance ``k`` is drawn
    from the same seed at every count, so ``test_3`` with four machines and
    with eight are two different cells, and the default of six reproduces
    every instance the existing results were run on.
    """
    if split not in _SPLIT_SEEDS:
        raise ValueError(f"unknown split {split!r} (expected one of {ALL_SPLITS})")
    if n_machines < 1:
        raise ValueError(f"n_machines must be positive, got {n_machines}")
    first, stop = _SPLIT_SEEDS[split]
    return [
        build_instance(seed, f"{split}_{k + 1}", split, k, n_machines=n_machines)
        for k, seed in enumerate(range(first, stop))
    ]


def random_layout(instance: Instance, rng: np.random.Generator) -> np.ndarray:
    """A uniformly random chromosome inside the gene box."""
    lower, upper = instance.gene_bounds()
    return lower + rng.random(lower.size) * (upper - lower)


def placement_rate(instance: Instance, samples: int = 2000, seed: int = 7) -> float:
    """Fraction of random layouts that pass the *geometric* tests alone.

    Reported next to the full feasibility rate so the two constraint families
    can be told apart: this one measures how crowded the cell is, and the gap
    to the full rate measures how much the robot itself rules out.
    """
    rng = np.random.default_rng(seed)
    passed = 0
    for _ in range(samples):
        genes = random_layout(instance, rng).reshape(-1, 3)
        placed = [
            machine.copy_at(row[0], row[1], row[2])
            for machine, row in zip(instance.machines, genes)
        ]
        if layout_is_placeable(
            placed, instance.bounds, instance.base_xy,
            instance.keep_out_radius, instance.clearance,
        ):
            passed += 1
    return passed / samples


def describe(instance: Instance) -> Dict[str, float]:
    """A few scalars per instance, for the results metadata."""
    footprint = sum(m.width * m.depth for m in instance.machines)
    min_x, max_x, min_y, max_y = instance.bounds
    floor = (max_x - min_x) * (max_y - min_y)
    heights = [m.height for m in instance.machines]
    return {
        "n_machines": float(instance.n_machines),
        "footprint_area": footprint,
        "floor_area": floor,
        "occupancy": footprint / floor,
        "min_height": min(heights),
        "max_height": max(heights),
        "mean_height": float(np.mean(heights)),
    }
