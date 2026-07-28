"""
Curve Sorting Component for GeomSeq
GH entry point: greedy k-NN + 2-opt curve ordering (geometry_utils.sort_curves_native).
"""

import os
import sys

import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th

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

from geomseq_core.geometry_utils import sort_curves_native


def _resolve_start_point():
    """Unconnected optional GH input (missing or None) reads as the origin."""
    try:
        value = start_point
    except NameError:
        return rg.Point3d(0, 0, 0)
    return rg.Point3d(0, 0, 0) if value is None else value


if __name__ == "__main__":
    # th.tree_to_list() returns None on large (~5000 item) DataTrees; read
    # .Branches directly instead. Falls back to treating curves_tree as one
    # group if GH already simplified a single-branch tree into a plain list.
    if curves_tree is None:  # type: ignore
        print("[geomseq_core] curves_tree input is None -- check it's actually connected/internalized")
        nested_groups = []
    elif hasattr(curves_tree, "Branches"):  # type: ignore
        nested_groups = [list(branch) for branch in curves_tree.Branches]  # type: ignore
    else:
        nested_groups = [list(curves_tree)]  # type: ignore

    final_nested_curves = []        # [[sorted_group0], [sorted_group1], ...]
    final_nested_travel_lines = []  # [[travel_lines for group0], ...]
    final_nested_reversed = []      # [[bool per curve in group0], ...]
    final_nested_full_path = []     # [[travel0, curve0, travel1, curve1, ...], ...]
    final_nested_full_types = []    # [["travel", "path", "travel", "path", ...], ...]

    current_start_pt = _resolve_start_point()

    KNN_K      = 12
    USE_2OPT   = True
    MAX_PASSES = 10

    for i, group in enumerate(nested_groups):
        # Sort this group; chain start_pt from the previous group's end point.
        sorted_group, order, travel_points = sort_curves_native(
            group, current_start_pt,
            use_two_opt=USE_2OPT,
            two_opt_max_passes=MAX_PASSES,
            knn_k=KNN_K,
            if_flip=if_flip,
            return_travel_points=True,
        )

        # travel_points is plain (x,y,z) tuples (geometry_utils is Rhino-free);
        # LineCurve (not Line) so it's the same GH type as the sorted curves.
        travel_lines = [rg.LineCurve(rg.Point3d(*p1), rg.Point3d(*p2)) for p1, p2 in travel_points]

        # Detect reversal by comparing each sorted curve's start point to its
        # original (pre-sort) curve's start point -- not exposed by the native call.
        is_reversed = [
            sorted_group[k].PointAtStart.DistanceTo(group[order[k]].PointAtStart) > 1e-6
            for k in range(len(sorted_group))
        ]

        # Interleave travel/curve into actual path order, with a parallel type
        # label, so callers don't have to weave two trees together downstream.
        full_path = []
        full_types = []
        for k in range(len(sorted_group)):
            full_path.append(travel_lines[k])
            full_types.append("travel")
            full_path.append(sorted_group[k])
            full_types.append("path")

        final_nested_curves.append(sorted_group)
        final_nested_travel_lines.append(travel_lines)
        final_nested_reversed.append(is_reversed)
        final_nested_full_path.append(full_path)
        final_nested_full_types.append(full_types)

        # Update the start point for the next group
        if sorted_group:
            current_start_pt = sorted_group[-1].PointAtEnd

    # --- Output to Grasshopper (tree only, preserves group structure) ---
    # sorted_curves_tree (not curves_tree) avoids colliding with the curves_tree input.
    sorted_curves_tree = th.list_to_tree(final_nested_curves)
    travel_lines_tree = th.list_to_tree(final_nested_travel_lines)
    reversed_tree = th.list_to_tree(final_nested_reversed)
    full_path_tree = th.list_to_tree(final_nested_full_path)
    full_types_tree = th.list_to_tree(final_nested_full_types)
