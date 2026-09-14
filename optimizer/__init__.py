"""Scoring a layout and searching for a better one.

``evaluator`` -- the feasibility wall, the retracted tour, the cycle-time objective.
``ga`` -- ``ConfigurableGA``, ``GAConfig`` and the seeded initial population."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
