"""Put the study's source directories on ``sys.path``.

The modules import each other by bare name (``from robot import ...``).  They
are grouped into topic directories now -- ``kinematics/``, ``layout/``,
``optimizer/``, ``viz/`` -- so those directories have to be on the path before
the first such import runs.

Every entry point under ``experiments/`` and ``tests/`` calls
:func:`add_source_dirs` before it imports anything from the study, and each
topic package's ``__init__`` calls it as well, so ``import arm_study.viz.visualize``
also works on its own.
"""

from __future__ import annotations

import os
import sys

#: The study's root -- the directory this file lives in.
ROOT = os.path.dirname(os.path.abspath(__file__))

#: Where the source lives, in the order a bare import should search it.
SOURCE_DIRS = ("kinematics", "layout", "optimizer", "viz", "experiments", "tests")

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
