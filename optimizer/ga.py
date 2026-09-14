"""The genetic algorithm: generational, elitist, no local search.

A plain static GA -- tournament selection, uniform crossover, Gaussian
mutation with re-orientation, elitism -- so that differences in the results
are attributable to the objective and the constraints, not to the search.

Chromosome: ``[x, y, rotation]`` per machine.  The rotation gene is not
decorative -- the loading port sits off-centre on each machine's top face, so
rotating a machine moves the point the arm must reach, and with it the joint
angles, the manipulability and the cycle time.

**Seeding the first generation** is the one departure from a textbook GA.
Drawn uniformly, only about 5 % of layouts place without overlap and well
under 1 % clear the robot's constraints too, so a random population of 200
would hold essentially no finite fitness and selection would have nothing to
rank.  The initial population is therefore rejection-sampled for *placement*
only: draw until the boxes fit, up to a cap, then keep the last draw.

That buys a foothold in the geometrically legal region and nothing more --
reachability and manipulability the search still has to find for itself.  The
cap keeps a crowded instance terminating, and ``seeded_fraction`` records how
many members really came back placeable, so seeding cannot hide a broken
instance.
"""

from __future__ import annotations

import math
import os
import random
import time
from dataclasses import asdict, dataclass, field
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from evaluator import CycleTimeEvaluator
from geometry import Machine, layout_is_placeable, repair_placement
from problems import Instance


@dataclass
class GAConfig:
    """Operator settings."""

    label: str = "GA (no LS)"
    population_size: int = 200
    generations: int = 300
    crossover_rate: float = 0.9
    mutation_rate: float = 0.1
    elitism_rate: float = 0.1
    tournament_k: int = 3
    #: Gaussian jitter width for a position gene, as a fraction of its range.
    mutation_sigma: float = 0.10
    #: Project unplaceable children back onto the placeable set after
    #: mutation.  Off by default, so the baseline is the plain GA.
    repair: bool = False
    #: Slack projected past each placement constraint when repairing.
    repair_margin: float = 0.005
    max_evaluations: Optional[int] = None
    record_diversity: bool = True
    verbose: int = 0

    def evaluation_budget(self) -> int:
        if self.max_evaluations is not None:
            return int(self.max_evaluations)
        return int(self.population_size * self.generations)

    def to_dict(self) -> Dict:
        d = asdict(self)
        d["max_evaluations"] = self.evaluation_budget()
        return d


@dataclass
class RunResult:
    """One completed run: the answer, the accounting and the traces."""

    label: str
    instance_name: str
    seed: int
    best_fitness: float
    best_chromosome: np.ndarray
    feasible: bool
    generations: int
    total_evaluations: int
    execution_time: float
    cpu_time: float
    seeded_fraction: float
    best_so_far: np.ndarray
    gen_best: np.ndarray
    gen_avg: np.ndarray
    diversity: np.ndarray
    evaluations: np.ndarray
    elapsed: np.ndarray
    feasible_fraction: np.ndarray
    #: Cumulative *process* CPU seconds at the end of each generation.
    #: ``elapsed`` is wall clock, which stops measuring work once runs share a
    #: machine, and every experiment here fans out over a process pool.  The
    #: per-generation CPU is what makes an equal-compute comparison a slice of
    #: an existing trace rather than a second set of runs.  Optional, so a run
    #: reconstructed from JSON without it is still a valid ``RunResult``.
    cpu_elapsed: Optional[np.ndarray] = None

    def trace_dict(self) -> Dict[str, np.ndarray]:
        return {
            "best_so_far": self.best_so_far,
            "gen_best": self.gen_best,
            "gen_avg": self.gen_avg,
            "diversity": self.diversity,
            "evaluations": self.evaluations,
            "elapsed": self.elapsed,
            "feasible_fraction": self.feasible_fraction,
            **({} if self.cpu_elapsed is None else {"cpu_elapsed": self.cpu_elapsed}),
        }


def population_diversity(population: np.ndarray) -> float:
    """Mean spread of the genes, as a single scalar.

    The per-gene standard deviation averaged over genes.  It is only ever
    compared against itself across generations, so the mixed units (metres and
    degrees) do not matter.
    """
    return float(np.mean(np.std(np.asarray(population, dtype=float), axis=0)))


def make_initial_population(
    instance: Instance,
    size: int,
    seed: Optional[int] = None,
    max_attempts: int = 200,
) -> Tuple[np.ndarray, float]:
    """``size`` layouts, rejection-sampled to be geometrically placeable.

    Returns the population and the fraction of members that came back
    genuinely placeable within ``max_attempts`` draws.
    """
    rng = np.random.default_rng(seed)
    lower, upper = instance.gene_bounds()
    span = upper - lower

    members: List[np.ndarray] = []
    placeable = 0
    for _ in range(size):
        genes = lower + rng.random(lower.size) * span
        for _attempt in range(max_attempts):
            candidate = lower + rng.random(lower.size) * span
            rows = candidate.reshape(-1, 3)
            placed = [
                machine.copy_at(row[0], row[1], row[2])
                for machine, row in zip(instance.machines, rows)
            ]
            if layout_is_placeable(
                placed,
                instance.bounds,
                instance.base_xy,
                instance.keep_out_radius,
                instance.clearance,
            ):
                genes = candidate
                placeable += 1
                break
        members.append(genes)
    return np.array(members), placeable / max(size, 1)


class ConfigurableGA:
    """The static GA on the cycle-time objective."""

    def __init__(
        self,
        instance: Instance,
        evaluator: Optional[CycleTimeEvaluator] = None,
        config: Optional[GAConfig] = None,
        seed: Optional[int] = None,
    ):
        self.instance = instance
        self.evaluator = evaluator or CycleTimeEvaluator(instance)
        self.config = config or GAConfig()
        self.seed = seed

        self.rng = random.Random(seed)
        self.np_rng = np.random.default_rng(seed)

        self.lower, self.upper = instance.gene_bounds()
        self.n_genes = self.lower.size
        #: Every third gene is a rotation, which wraps rather than clips.
        self.is_rotation = np.zeros(self.n_genes, dtype=bool)
        self.is_rotation[2::3] = True

    # -- operators -----------------------------------------------------

    def apply_bounds(self, chromosome: np.ndarray) -> np.ndarray:
        """Clip positions into the cell, wrap rotations onto the circle."""
        repaired = np.asarray(chromosome, dtype=float).copy()
        positions = ~self.is_rotation
        repaired[positions] = np.clip(
            repaired[positions], self.lower[positions], self.upper[positions]
        )
        repaired[self.is_rotation] = repaired[self.is_rotation] % 360.0
        return repaired

    def repair(self, chromosome: np.ndarray) -> np.ndarray:
        """Push an unplaceable layout back onto the placeable set.

        :meth:`apply_bounds` already clips into the cell, but that is the one
        constraint separable per gene -- it says nothing about the robot's
        column or a neighbour the machine is standing in.  This closes the
        other two, so mutation stops spending evaluations on layouts whose
        only fault is geometric.

        Positions only; rotations come back exactly as drawn, and an already
        placeable chromosome is returned unchanged.  The input is copied, not
        viewed: ``np.asarray`` would hand back the caller's buffer and the row
        assignment below would edit a parent in place.
        """
        if not self.config.repair:
            return chromosome
        instance = self.instance
        rows = np.array(chromosome, dtype=float).reshape(-1, 3)
        placed = repair_placement(
            [machine.copy_at(row[0], row[1], row[2])
             for machine, row in zip(instance.machines, rows)],
            instance.bounds,
            instance.base_xy,
            instance.keep_out_radius,
            instance.clearance,
            margin=self.config.repair_margin,
        )
        rows[:, 0] = [machine.x for machine in placed]
        rows[:, 1] = [machine.y for machine in placed]
        return rows.reshape(-1)

    def tournament(self, population: np.ndarray, fitness: np.ndarray) -> np.ndarray:
        """Pick the best of ``tournament_k`` random members.

        Ties are common on the ``+inf`` plateau; taking the first is unbiased
        among equals, because the contestants were drawn at random.
        """
        contenders = self.rng.sample(range(population.shape[0]), self.config.tournament_k)
        best = min(contenders, key=lambda i: fitness[i])
        return population[best].copy()

    def crossover(
        self, parent_a: np.ndarray, parent_b: np.ndarray
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Uniform crossover, gene by gene."""
        if self.rng.random() >= self.config.crossover_rate:
            return parent_a.copy(), parent_b.copy()
        mask = self.np_rng.random(self.n_genes) < 0.5
        child_a = np.where(mask, parent_a, parent_b)
        child_b = np.where(mask, parent_b, parent_a)
        return child_a, child_b

    def mutate(self, chromosome: np.ndarray) -> np.ndarray:
        """Per machine: jitter the position and re-draw the rotation.

        The unit of mutation is the machine, not the gene, because moving a
        machine's x without its y is rarely the useful move in a layout.
        """
        mutated = chromosome.copy()
        span = self.upper - self.lower
        for block in range(self.n_genes // 3):
            if self.rng.random() >= self.config.mutation_rate:
                continue
            i = 3 * block
            sigma = self.config.mutation_sigma
            mutated[i] += self.np_rng.normal(0.0, sigma * span[i])
            mutated[i + 1] += self.np_rng.normal(0.0, sigma * span[i + 1])
            mutated[i + 2] = self.np_rng.random() * 360.0
        return self.repair(self.apply_bounds(mutated))

    # -- the loop ------------------------------------------------------

    def optimize(
        self, initial_population: Optional[np.ndarray] = None, seeded_fraction: float = float("nan")
    ) -> RunResult:
        cfg = self.config

        if initial_population is None:
            initial_population, seeded_fraction = make_initial_population(
                self.instance, cfg.population_size, seed=self.seed
            )
        population = np.array([self.apply_bounds(row) for row in initial_population])

        n = cfg.generations
        traces = {
            key: np.full(n, np.nan)
            for key in (
                "best_so_far",
                "gen_best",
                "gen_avg",
                "diversity",
                "evaluations",
                "elapsed",
                "feasible_fraction",
                "cpu_elapsed",
            )
        }

        self.evaluator.reset_counter()
        start_time = time.perf_counter()
        start_cpu = time.process_time()
        budget = cfg.evaluation_budget()

        fitness = np.array([self.evaluator.fitness(row) for row in population])
        best_fitness = math.inf
        best_chromosome: Optional[np.ndarray] = None
        generations_run = 0

        for generation in range(n):
            order = np.argsort(fitness)
            if fitness[order[0]] < best_fitness:
                best_fitness = float(fitness[order[0]])
                best_chromosome = population[order[0]].copy()

            finite = fitness[np.isfinite(fitness)]
            traces["best_so_far"][generation] = best_fitness
            traces["gen_best"][generation] = float(fitness[order[0]])
            traces["gen_avg"][generation] = float(finite.mean()) if finite.size else np.nan
            traces["diversity"][generation] = (
                population_diversity(population) if cfg.record_diversity else np.nan
            )
            traces["evaluations"][generation] = self.evaluator.n_evals
            traces["elapsed"][generation] = time.perf_counter() - start_time
            traces["cpu_elapsed"][generation] = time.process_time() - start_cpu
            traces["feasible_fraction"][generation] = finite.size / fitness.size

            if cfg.verbose and generation % cfg.verbose == 0:
                print(
                    f"[{cfg.label}] gen {generation:>4}: best={best_fitness:.4f}s "
                    f"feasible={finite.size}/{fitness.size} "
                    f"evals={self.evaluator.n_evals}"
                )

            generations_run = generation + 1
            if self.evaluator.n_evals >= budget:
                break

            # -- next generation: elites carried over untouched ---------
            n_elite = max(1, int(round(cfg.elitism_rate * cfg.population_size)))
            children = [population[i].copy() for i in order[:n_elite]]

            while len(children) < cfg.population_size:
                parent_a = self.tournament(population, fitness)
                parent_b = self.tournament(population, fitness)
                child_a, child_b = self.crossover(parent_a, parent_b)
                children.append(self.mutate(child_a))
                if len(children) < cfg.population_size:
                    children.append(self.mutate(child_b))

            population = np.array(children)
            # The elites keep their fitness; only the offspring are new work.
            fitness = np.concatenate(
                [
                    fitness[order[:n_elite]],
                    np.array(
                        [self.evaluator.fitness(row) for row in population[n_elite:]]
                    ),
                ]
            )

        execution_time = time.perf_counter() - start_time
        cpu_time = time.process_time() - start_cpu
        if best_chromosome is None:
            best_chromosome = population[int(np.argmin(fitness))].copy()

        trimmed = {key: value[:generations_run] for key, value in traces.items()}
        return RunResult(
            label=cfg.label,
            instance_name=self.instance.name,
            seed=self.seed if self.seed is not None else -1,
            best_fitness=float(best_fitness),
            best_chromosome=best_chromosome,
            feasible=bool(math.isfinite(best_fitness)),
            generations=int(generations_run),
            total_evaluations=int(self.evaluator.n_evals),
            execution_time=float(execution_time),
            cpu_time=float(cpu_time),
            seeded_fraction=float(seeded_fraction),
            **trimmed,
        )
