"""The command-line experiments; each writes one tree under ``results/``.

``run_ga`` -- the main run: run, aggregate, figures.
``run_ablation`` -- the crossover x mutation ablation.
``run_repair_ab`` -- the paired A/B of the placement repair.
``equal_cpu_check`` -- the equal-compute control for that A/B.
``run_penalty_ab`` -- four constraint-handling arms on identical runs.
``penalty_weights`` -- the weight checks behind the penalty arms.
``holm`` -- family-wise correction over the arm comparisons."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
