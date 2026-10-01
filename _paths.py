"""Put the source directories on ``sys.path``.

The modules import each other by bare name (``from robot import ...``) but are
grouped into topic directories, so those have to be on the path first.  Every
entry point under ``experiments/`` and ``tests/`` calls :func:`add_source_dirs`
before its first such import, and each package's ``__init__`` does too, so
``import viz.visualize`` works on its own.

Nothing here, or anywhere else, depends on what the repository's root folder
is called: every path is found from this file's own location.  A clone into
``rwl-repair/`` or ``My Study/`` behaves exactly like one into ``arm_study/``.
"""

from __future__ import annotations

import os
import sys

#: The project root -- the directory this file lives in.
ROOT = os.path.dirname(os.path.abspath(__file__))

#: Where the source lives, in the order a bare import should search it.
SOURCE_DIRS = ("kinematics", "layout", "optimizer", "sim", "viz", "experiments", "tests")

#: Where every run writes: ``results/<name>/``.
RESULTS_ROOT = os.path.join(ROOT, "results")


def add_source_dirs() -> None:
    """Prepend the root and every source directory to ``sys.path``, once each."""
    for name in reversed(SOURCE_DIRS):
        directory = os.path.join(ROOT, name)
        if directory not in sys.path:
            sys.path.insert(0, directory)
    if ROOT not in sys.path:
        sys.path.insert(0, ROOT)


def results_dir(name: str) -> str:
    """The absolute path of the results tree called ``name``."""
    return os.path.join(RESULTS_ROOT, name)
