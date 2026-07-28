"""
Divide Curves Component for GeomSeq
GH entry point: thin GH-input wiring around rhino_utils.divide_curves
(DivideCurves + process_curve there do the actual work).
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

from rhino_utils.divide_curves import DivideCurves, process_curve


def _resolve_overlap_flag():
    """Unconnected optional GH input (missing or None) reads as False."""
    try:
        value = overlap
    except NameError:
        return False
    return False if value is None else value


def _resolve_join_ends_flag():
    """Unconnected optional GH input (missing or None) reads as True."""
    try:
        value = join_ends
    except NameError:
        return True
    return True if value is None else value


def _resolve_segment_length(segment_lengths, i):
    """Accepts a scalar (broadcast) or a per-curve list; reuses the last entry if the list is shorter than the curve count."""
    if not hasattr(segment_lengths, "__len__"):
        return segment_lengths
    if i < len(segment_lengths):
        return segment_lengths[i]
    return segment_lengths[-1]


def _resolve_overlap_length(seg_length):
    """Unconnected optional GH input (missing or None) reads as this curve's seg_length."""
    try:
        value = overlap_length
    except NameError:
        return seg_length
    return seg_length if value is None else value


if __name__ == "__main__":
    overlap = _resolve_overlap_flag()
    join_ends = _resolve_join_ends_flag()

    dc = DivideCurves(curves)
    pts_nested, lookups_nested, corner_indices_nested = [], [], []

    for i, crv in enumerate(dc.curves):
        seg_length = _resolve_segment_length(segment_lengths, i)
        this_overlap_length = _resolve_overlap_length(seg_length)
        crv_pts, crv_lookups, crv_corners = process_curve(
            dc, crv, seg_length, join_ends, overlap, this_overlap_length
        )
        pts_nested.append(crv_pts)
        lookups_nested.append(crv_lookups)
        corner_indices_nested.append(crv_corners)

    pts_tree = th.list_to_tree(pts_nested)
    lookups_tree = th.list_to_tree(lookups_nested)
    corner_indices_tree = th.list_to_tree(corner_indices_nested)
