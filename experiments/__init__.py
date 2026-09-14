"""The command-line studies; each writes one tree under ``results/``.

``run_ga`` -- the main study: run, aggregate, figures.
``run_ablation`` -- the crossover x mutation ablation.
``run_repair_ab`` -- the paired A/B of the placement repair.
``equal_cpu_check`` -- the equal-compute control for that A/B."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
