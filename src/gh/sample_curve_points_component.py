"""
Sample Curve Points Component for GeomSeq
GH entry point: wraps rhino_utils.sample_curve_points (evaluates points at arc lengths).
"""

import os
import sys

import ghpythonlib.treehelpers as th

try:
    import rhino_utils
except ImportError:
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from rhino_utils.sample_curve_points import sample_curve_points


if __name__ == "__main__":
    nested_arc_lengths = th.tree_to_list(arc_lengths_tree)  # type: ignore

    out_nested_points = []
    for curve, arc_lengths in zip(curves, nested_arc_lengths):
        out_nested_points.append(sample_curve_points(curve, arc_lengths))

    out_points_tree = th.list_to_tree(out_nested_points)
