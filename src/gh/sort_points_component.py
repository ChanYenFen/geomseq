"""
Point Sorting Component for GeomSeq
GH entry point: greedy k-NN + 2-opt point ordering (geometry_utils.sort_points_native).
"""

import os
import sys

import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th

try:
    import geomseq_core
except ImportError:
    # Dev mode: geomseq_core isn't installed / not on sys.path yet -- add its
    # parent src/ dir (this file lives in src/gh/, geomseq_core lives in src/geomseq_core/).
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

# Force-reload geometry_utils/misc so edits show up without restarting Rhino.
# Skip native_bridge -- reloading it would reload geomseq_core.dll every recompute (slow).
_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")

from geomseq_core.geometry_utils import sort_points_native


def _resolve_start_point():
    """Unconnected optional GH input (missing or None) reads as the origin."""
    try:
        value = start_point
    except NameError:
        return rg.Point3d(0, 0, 0)
    return rg.Point3d(0, 0, 0) if value is None else value


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

    final_nested_points = []   # [[sorted_group0], [sorted_group1], ...]
    final_nested_indices = []  # [[order0], [order1], ...] -- original index per sorted point
    flattened_points = []
    flattened_points = []

    current_start_pt = _resolve_start_point()

    KNN_K      = 12
    USE_2OPT   = True
    MAX_PASSES = 10


    for i, group in enumerate(nested_groups):
        # Sort the current group, using the last point of the previous group as start_pt
        sorted_group, order = sort_points_native(group, current_start_pt,
                                                   use_two_opt=USE_2OPT,
                                                   two_opt_max_passes=MAX_PASSES,
                                                   knn_k=KNN_K)

        final_nested_points.append(sorted_group)
        final_nested_indices.append(order)

        # Update the start point for the next group
        if sorted_group:
            current_start_pt = sorted_group[-1]

    # --- Output to Grasshopper (tree only, preserves group structure) ---
    sorted_points_tree = th.list_to_tree(final_nested_points)
    sorted_indices_tree = th.list_to_tree(final_nested_indices)