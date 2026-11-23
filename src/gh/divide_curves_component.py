"""
Divide Curves Component for GeomSeq
GH entry point: thin wiring around rhino_utils.divide_curves (DivideCurves + process_curve do the work).
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

from rhino_utils.divide_curves import DivideCurves, process_curve


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


def _resolve_segment_length(segment_lengths, i):
    """A scalar, or a per-curve list whose last entry repeats."""
    if not hasattr(segment_lengths, "__len__"):
        return segment_lengths
    if i < len(segment_lengths):
        return segment_lengths[i]
    return segment_lengths[-1]


if __name__ == "__main__":
    overlap = _optional("overlap", False)
    join_ends = _optional("join_ends", True)

    dc = DivideCurves(curves)
    pts_nested, arc_lengths_nested, corner_indices_nested = [], [], []

    for i, crv in enumerate(dc.curves):
        seg_length = _resolve_segment_length(segment_lengths, i)
        this_overlap_length = _optional("overlap_length", seg_length)
        crv_pts, crv_arc_lengths, crv_corners = process_curve(
            dc, crv, seg_length, join_ends, overlap, this_overlap_length
        )
        pts_nested.append(crv_pts)
        arc_lengths_nested.append(crv_arc_lengths)
        corner_indices_nested.append(crv_corners)

    pts_tree = th.list_to_tree(pts_nested)
    arc_lengths_tree = th.list_to_tree(arc_lengths_nested)
    corner_indices_tree = th.list_to_tree(corner_indices_nested)
