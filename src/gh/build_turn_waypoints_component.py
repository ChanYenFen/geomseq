"""
Build Turn Waypoints Component for GeomSeq
GH entry point: smooth turn waypoints between adjacent strokes (geometry_utils.build_turn_waypoints_native).
"""

import os
import sys

import Rhino.Geometry as rg
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path

try:
    import geomseq_core
except ImportError:
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")

from geomseq_core.geometry_utils import build_turn_waypoints_native


if __name__ == "__main__":
    if points_tree is None:  # type: ignore
        print("[geomseq_core] points_tree input is None -- check it's actually connected/internalized")
        nested_groups = []
    elif hasattr(points_tree, "Branches"):  # type: ignore
        nested_groups = [list(branch) for branch in points_tree.Branches]  # type: ignore
    else:
        nested_groups = [list(points_tree)]  # type: ignore

    all_turn_points = []

    for i in range(len(nested_groups) - 1):
        stroke, next_stroke = nested_groups[i], nested_groups[i + 1]

        # A stroke needs >= 2 points to define a heading; skip turns touching a shorter one.
        if len(stroke) < 2 or len(next_stroke) < 2:
            continue

        E, S = stroke[-1], next_stroke[0]
        a_vec = E - stroke[-2]      # heading leaving E
        b_vec = next_stroke[1] - S  # heading into S

        exit_pts, entry_pts = build_turn_waypoints_native(
            E.X, E.Y, a_vec.X, a_vec.Y,
            S.X, S.Y, b_vec.X, b_vec.Y,
            theta_max_deg, step_len, extend_len,  # type: ignore
        )

        all_turn_points.append([rg.Point3d(x, y, 0.0) for x, y in exit_pts + entry_pts])

    # Built manually: th.list_to_tree() returns None past ~5000 items.
    waypoints_tree = DataTree[object]()
    for i, turn_points in enumerate(all_turn_points):
        waypoints_tree.AddRange(turn_points, GH_Path(i))
