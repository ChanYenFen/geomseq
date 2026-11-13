"""
Shatter Crossings Component for GeomSeq
GH entry point: cuts polylines where they cross and opens a gap at each crossing
(geometry_utils.shatter_at_crossings_native).

Polylines only. They are exploded into segments, shattered, and the pieces that
were never cut are rejoined into polylines.
"""

import os
import sys

import Rhino
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

from geomseq_core.geometry_utils import shatter_at_crossings_native

# How close two piece ends must be to rejoin. Only absorbs the plane round-trip.
JOIN_TOL = 1e-9


class _Seg(object):
    """What curves_to_endpoint_buffer needs: two points, in plane space."""

    __slots__ = ("PointAtStart", "PointAtEnd")

    def __init__(self, a, b):
        self.PointAtStart, self.PointAtEnd = a, b


def _resolve_plane():
    """Unconnected optional GH input reads as world XY."""
    try:
        value = plane  # type: ignore
    except NameError:
        value = None
    return rg.Plane.WorldXY if value is None else value


def _resolve_gap_distance():
    """D is the whole gap: how far apart the two cut ends end up. Each crossing
    loses D/2 to either side of itself."""
    try:
        value = gap_distance  # type: ignore
    except NameError:
        value = None
    if value is None:
        print("[geomseq_core] D not connected -- using 0.0, so polylines are cut "
              "at crossings but no gap is opened")
        return 0.0
    return float(value)


def _resolve_touch_tolerance():
    """How close counts as touching. Unconnected reads as the document tolerance."""
    try:
        value = touch_tolerance  # type: ignore
    except NameError:
        value = None
    if value is not None:
        return float(value)
    try:
        return Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
    except Exception:
        return 0.0


def _resolve_self_intersection():
    try:
        value = self_intersection  # type: ignore
    except NameError:
        value = None
    return bool(value) if value is not None else False


def _to_polylines(curves):
    """Returns (polylines, rejected): rejected holds the input indices that are
    not polylines."""
    polylines, rejected = [], []
    for i, crv in enumerate(curves):
        if crv is None:
            rejected.append(i)
            continue
        ok, pl = crv.TryGetPolyline()
        if not ok or pl is None or pl.Count < 2:
            rejected.append(i)
        else:
            polylines.append((i, pl))
    return polylines, rejected


if __name__ == "__main__":
    src_plane = _resolve_plane()
    gap_d = _resolve_gap_distance()
    touch_tol = _resolve_touch_tolerance()
    test_self = _resolve_self_intersection()

    raw = list(curves) if curves else []  # type: ignore

    polylines, rejected = _to_polylines(raw)

    if rejected:
        # Any non-polyline refuses the whole input.
        print("[geomseq_core] %d of %d input curves are not polylines (indices %s). "
              "Shatter Crossings handles polylines only -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
        segments_tree = th.list_to_tree([[]])
        piece_counts_tree = th.list_to_tree([[]])
    else:
        # --- explode, in plane space ---
        # The crossing test is 2D, in plane coordinates; the height above the
        # plane rides along as z.
        segs, owners, spans = [], [], []
        for owner, (src_index, pl) in enumerate(polylines):
            local = []
            for k in range(pl.Count):
                p = pl[k]
                ok, lp = src_plane.RemapToPlaneSpace(p)
                local.append(lp if ok else p)

            start = len(segs)
            for k in range(len(local) - 1):
                segs.append(_Seg(local[k], local[k + 1]))
                owners.append(owner)
            spans.append((src_index, start, len(segs)))

        shattered = shatter_at_crossings_native(segs, gap_d,
                                                touch_tol=touch_tol,
                                                segment_owner=owners,
                                                test_self=test_self)

        # --- rejoin, back in world space ---
        # A run of pieces that touch end-to-start is one surviving polyline.
        nested_pieces, nested_counts = [], []
        for src_index, start, stop in spans:
            runs, cur = [], []
            for s_i in range(start, stop):
                for k, (s, e) in enumerate(shattered[s_i]):
                    # Only the first piece of a segment can continue the run:
                    # later pieces follow a cut, and at D = 0 a cut still touches.
                    joins = (k == 0 and cur and
                             all(abs(cur[-1][1][d] - s[d]) <= JOIN_TOL for d in (0, 1, 2)))
                    if joins:
                        cur.append((s, e))
                    else:
                        if cur:
                            runs.append(cur)
                        cur = [(s, e)]
            if cur:
                runs.append(cur)

            out_curves = []
            for run in runs:
                pts = [run[0][0]] + [e for _, e in run]
                world = [src_plane.PointAt(p[0], p[1], p[2]) for p in pts]
                out_curves.append(rg.PolylineCurve(world))

            nested_pieces.append(out_curves)
            nested_counts.append(len(out_curves))

        # Path {branch; input index}; a polyline swallowed whole keeps an empty path.
        segments_tree = th.list_to_tree([nested_pieces])
        piece_counts_tree = th.list_to_tree([nested_counts])
