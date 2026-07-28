"""
Sample Curve Points Component for GeomSeq
GH entry point: thin GH-input wiring around rhino_utils.sample_curve_points
(evaluates points on each curve at its arc-length lookups).
"""

import os
import sys

import ghpythonlib.treehelpers as th

try:
    import rhino_utils
except ImportError:
    # Dev mode: rhino_utils isn't installed / not on sys.path yet -- add its
    # parent src/ dir (this file lives in src/gh/, rhino_utils in src/rhino_utils/).
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from rhino_utils.sample_curve_points import sample_curve_points


if __name__ == "__main__":
    nested_lookups = th.tree_to_list(lookups_tree)  # type: ignore

    out_nested_points = []
    for curve, lookups in zip(curves, nested_lookups):
        out_nested_points.append(sample_curve_points(curve, lookups))

    out_points_tree = th.list_to_tree(out_nested_points)
