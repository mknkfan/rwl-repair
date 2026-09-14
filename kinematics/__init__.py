"""The robot itself: kinematics, motion timing and the solid model.

``robot`` -- FK, closed-form IK, Jacobians and sigma_min for the Puma 560.
``trajectory`` -- trapezoidal time scaling, segment and cycle times.
``armmesh`` -- the solid arm: parts per link frame, shading, clearance spheres."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
