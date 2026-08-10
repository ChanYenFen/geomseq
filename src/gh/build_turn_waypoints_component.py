"""
Turn Waypoints Component for GeomSeq
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
    # Dev mode: add src/ (two levels up) to sys.path so geomseq_core is importable.
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

# Hot-reload geometry_utils/misc on every run; skip native_bridge (would reload the dll).
_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")

from geomseq_core.geometry_utils import build_turn_waypoints_native


if __name__ == "__main__":
    # th.tree_to_list() returns None on large (~5000 item) DataTrees; read
    # .Branches directly instead (see sort_points_component.py).
    if points_tree is None:  # type: ignore
        print("[geomseq_core] points_tree input is None -- check it's actually connected/internalized")
        nested_groups = []
    elif hasattr(points_tree, "Branches"):  # type: ignore
        nested_groups = [list(branch) for branch in points_tree.Branches]  # type: ignore
    else:
        nested_groups = [list(points_tree)]  # type: ignore

    # theta_max_deg / step_len / extend_len are GH inputs, not constants -- the
    # turn shape needs tuning per path.
    all_turn_points = []  # [[waypoints for turn 0], ...]

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

        # geomseq_core is 2D-only, so z is not carried through.
        all_turn_points.append([rg.Point3d(x, y, 0.0) for x, y in exit_pts + entry_pts])

    # --- Output to Grasshopper ---
    # Built manually: th.list_to_tree() returns None past ~5000 items.
    waypoints_tree = DataTree[object]()
    for i, turn_points in enumerate(all_turn_points):
        waypoints_tree.AddRange(turn_points, GH_Path(i))
