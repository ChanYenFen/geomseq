"""
Turn Waypoints Component for GeomSeq
GH entry point: smooth polygonal turn waypoints between adjacent strokes
(geometry_utils.build_turn_waypoints_native).
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
    # .Branches directly instead. Falls back to treating points_tree as one
    # group if GH already simplified a single-branch tree into a plain list.
    if points_tree is None:  # type: ignore
        print("[geomseq_core] points_tree input is None -- check it's actually connected/internalized")
        nested_groups = []
    elif hasattr(points_tree, "Branches"):  # type: ignore
        nested_groups = [list(branch) for branch in points_tree.Branches]  # type: ignore
    else:
        nested_groups = [list(points_tree)]  # type: ignore

    # theta_max_deg / step_len / extend_len are real GH inputs here (unlike
    # sort_curves_component's hardcoded KNN_K etc.) -- callers tune the turn
    # shape per path, so they aren't baked into the script.

    all_turn_points = []  # one entry per computed turn; strokes with < 2 points skip their adjacent turn(s)

    for i in range(len(nested_groups) - 1):
        stroke = nested_groups[i]
        next_stroke = nested_groups[i + 1]

        # Mirrors the single-point-stroke skip from the original
        # smooth-turn-waypoints prototype: a stroke needs >= 2 points to
        # define a heading, so a turn touching a shorter stroke is skipped.
        if len(stroke) < 2 or len(next_stroke) < 2:
            continue

        E, E_prev = stroke[-1], stroke[-2]
        S, S_next = next_stroke[0], next_stroke[1]

        a_vec = E - E_prev       # heading leaving E (current stroke's own direction)
        b_vec = S_next - S       # desired heading into S (next stroke's own direction)

        # Unlike the old prototype, step_len/extend_len are passed through
        # unscaled (no per-turn decay factor -- that was a bug, not a
        # feature), and both returned point lists are used (the old
        # prototype's `entry_rev, _ = ...` silently dropped the entry side).
        exit_pts, entry_pts = build_turn_waypoints_native(
            E.X, E.Y, a_vec.X, a_vec.Y,
            S.X, S.Y, b_vec.X, b_vec.Y,
            theta_max_deg, step_len, extend_len,  # type: ignore
        )

        # geomseq_core is 2D-only; z is not carried through to these points.
        turn_points = [rg.Point3d(x, y, 0.0) for x, y in exit_pts]
        turn_points += [rg.Point3d(x, y, 0.0) for x, y in entry_pts]

        all_turn_points.append(turn_points)

    # --- Output to Grasshopper ---
    # Built manually via Grasshopper.DataTree rather than th.list_to_tree(),
    # which silently returns None past ~5000 items (see sort_points_component.py).
    waypoints_tree = DataTree[object]()
    for i, turn_points in enumerate(all_turn_points):
        waypoints_tree.AddRange(turn_points, GH_Path(i))
