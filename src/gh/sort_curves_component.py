"""
Sort Curves Component for GeomSeq
GH entry point: greedy + 2-opt curve ordering (geometry_utils.sort_curves_native).
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

from geomseq_core.geometry_utils import sort_curves_native


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


if __name__ == "__main__":
    if curves_tree is None:  # type: ignore
        print("[geomseq_core] curves_tree input is None -- check it's actually connected/internalized")
        nested_groups = []
    elif hasattr(curves_tree, "Branches"):  # type: ignore
        nested_groups = [list(branch) for branch in curves_tree.Branches]  # type: ignore
    else:
        nested_groups = [list(curves_tree)]  # type: ignore

    final_nested_curves = []
    final_nested_travel_lines = []
    final_nested_reversed = []
    final_nested_full_path = []
    final_nested_full_types = []

    current_start_pt = _optional("start_point", rg.Point3d(0, 0, 0))

    KNN_K      = 12
    USE_2OPT   = True
    MAX_PASSES = 20   # 2-opt converges by 20 on every shape measured

    for i, group in enumerate(nested_groups):
        sorted_group, order, travel_points = sort_curves_native(
            group, current_start_pt,
            use_two_opt=USE_2OPT,
            two_opt_max_passes=MAX_PASSES,
            knn_k=KNN_K,
            if_flip=if_flip,
            return_travel_points=True,
        )

        travel_lines = [rg.LineCurve(rg.Point3d(*p1), rg.Point3d(*p2)) for p1, p2 in travel_points]

        is_reversed = [
            sorted_group[k].PointAtStart.DistanceTo(group[order[k]].PointAtStart) > 1e-6
            for k in range(len(sorted_group))
        ]

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

        if sorted_group:
            current_start_pt = sorted_group[-1].PointAtEnd

    sorted_curves_tree = th.list_to_tree(final_nested_curves)
    travel_lines_tree = th.list_to_tree(final_nested_travel_lines)
    reversed_tree = th.list_to_tree(final_nested_reversed)
    full_path_tree = th.list_to_tree(final_nested_full_path)
    full_types_tree = th.list_to_tree(final_nested_full_types)
