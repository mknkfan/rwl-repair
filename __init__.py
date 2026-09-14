"""Robot-aware layout study: cycle time, reachability and manipulability.

Self-contained -- imports nothing from the rest of the repository.

The source is grouped by use case::

    kinematics/   the arm: FK, IK, Jacobians, trajectories, the solid model
    layout/       the cell: machines, placement geometry, the instance family
    optimizer/    the cycle-time objective and the GA that searches it
    viz/          figures, the GIF and the standalone HTML viewer
    experiments/  the command-line studies; each writes one tree under results/
    tests/        the self-test

Importing this package puts those directories on ``sys.path``, because the
modules inside import each other by bare name.
"""

from __future__ import annotations

from ._paths import ROOT, RESULTS_ROOT, add_source_dirs, results_dir  # noqa: F401

add_source_dirs()
