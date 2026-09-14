"""The cell: what a layout is and which layouts the study asks about.

``geometry`` -- machines as rotated rectangles, overlap, bounds, keep-out.
``problems`` -- the instance family and its three disjoint splits."""

from __future__ import annotations

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs  # noqa: E402

add_source_dirs()
