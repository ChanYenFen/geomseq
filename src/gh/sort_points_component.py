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


if __name__ == "__main__":
    nested_groups = th.tree_to_list(points_tree)  # type: ignore

    final_nested_points = []   # [[sorted_group0], [sorted_group1], ...]
    all_flattened_points = []  # [pt, pt, pt, ...]

    current_start_pt = rg.Point3d(0, 0, 0)

    KNN_K      = 12
    USE_2OPT   = True
    MAX_PASSES = 10

    # import time                      # ← 加這行
    # t0 = time.perf_counter()         # ← 加這行

    for i, group in enumerate(nested_groups):
        # Sort the current group, using the last point of the previous group as start_pt
        sorted_group, _ = sort_points_native(group, current_start_pt,
                                              use_two_opt=USE_2OPT,
                                              two_opt_max_passes=MAX_PASSES,
                                              knn_k=KNN_K)

        final_nested_points.append(sorted_group)
        all_flattened_points.extend(sorted_group)

        # Update the start point for the next group
        if sorted_group:
            current_start_pt = sorted_group[-1]

    # t1 = time.perf_counter()         # ← 加這行
    # print("total: %.3fs, groups: %d, sizes: %s" %              # ← 加這行
    #       (t1 - t0, len(nested_groups),
    #        [len(g) for g in nested_groups]))

    # --- Output to Grasshopper ---
    # DataTree output (preserves group structure)
    out_points_tree = th.list_to_tree(final_nested_points)

    # Flat list output
    out_points_flat = all_flattened_points
