"""The cell in a physics engine.

``puma_mjcf`` -- the Puma 560 and a layout as a MuJoCo model.
``mujoco_cell`` -- replay a cycle in it: mesh clearance, joint torques, tracking."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
