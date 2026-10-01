"""The robot's specification figure: dimensions, links, joints, travel.

    python -m experiments.make_arm_figure
    python -m experiments.make_arm_figure --spheres
    python -m experiments.make_arm_figure --out results/model/arm_model.png

Draws the Puma 560 as this study models it -- the elevation with every link
length dimensioned, the plan with the shoulder offset and the workable
annulus, and the joint and DH tables -- all read from :mod:`robot`, so the
figure is a view of the code rather than a redrawing of it.  ``--spheres``
overlays the bounding spheres the clearance test actually uses.
"""

from __future__ import annotations

import argparse
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from _paths import add_source_dirs, results_dir  # noqa: E402

add_source_dirs()

from armdiagram import figure_arm_model  # noqa: E402

#: Anchored on the repository, not the working directory, like every
#: other default output.
DEFAULT_OUT = os.path.join(results_dir("model"), "arm_model.png")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description=__doc__.splitlines()[0],
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("--out", default=DEFAULT_OUT, help="where the PNG goes")
    parser.add_argument("--spheres", action="store_true",
                        help="overlay the collision spheres the clearance test uses")
    return parser


def main() -> None:
    args = build_parser().parse_args()
    path = args.out
    if not os.path.isabs(path):
        path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), path)
    print(figure_arm_model(path, spheres=args.spheres))


if __name__ == "__main__":
    main()
