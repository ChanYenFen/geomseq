"""
Sort Points Component for GeomSeq
GH entry point: greedy + 2-opt point ordering (geometry_utils.sort_points_native).
"""

import os
import sys

import Rhino.Geometry as rg
import ghpythonlib.treehelpers as th

try:
    import geomseq_core
except ImportError:
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")

from geomseq_core.geometry_utils import sort_points_native


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


if __name__ == "__main__":
    if points_tree is None:  # type: ignore
        print("[geomseq_core] points_tree input is None -- check it's actually connected/internalized")
        nested_groups = []
    elif hasattr(points_tree, "Branches"):  # type: ignore
        nested_groups = [list(branch) for branch in points_tree.Branches]  # type: ignore
    else:
        nested_groups = [list(points_tree)]  # type: ignore

    final_nested_points = []
    final_nested_indices = []

    current_start_pt = _optional("start_point", rg.Point3d(0, 0, 0))

    KNN_K      = 12
    USE_2OPT   = True
    MAX_PASSES = 20   # 2-opt converges by 20 on every shape measured

    for i, group in enumerate(nested_groups):
        sorted_group, order = sort_points_native(group, current_start_pt,
                                                   use_two_opt=USE_2OPT,
                                                   two_opt_max_passes=MAX_PASSES,
                                                   knn_k=KNN_K)

        final_nested_points.append(sorted_group)
        final_nested_indices.append(order)

        if sorted_group:
            current_start_pt = sorted_group[-1]

    sorted_points_tree = th.list_to_tree(final_nested_points)
    sorted_indices_tree = th.list_to_tree(final_nested_indices)
