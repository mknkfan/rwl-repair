"""Pictures of a layout, a trajectory and a run.

``visualize`` -- layout, 3-D trajectory, joint/rate profiles, convergence, GIF.
``viewer`` -- the standalone drag-to-rotate HTML viewer."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
