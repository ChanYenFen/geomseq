"""
Shatter Crossings Component for GeomSeq
GH entry point: cuts polylines where they cross and opens a gap at each crossing
(geometry_utils.shatter_at_crossings_native).

Polylines only. They are exploded into segments, shattered, and the pieces that
were never cut are rejoined -- so a 100-segment polyline crossed once comes back
as two polylines, not a hundred loose lines. Exploding is also what keeps the
door open for a sweep line later: it would replace how crossings are found,
without touching the explode/rejoin either side of it.
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

# Pieces are built from the same doubles the vertices came from, so an uncut
# segment reproduces its endpoints exactly and a shared vertex is bit-identical.
# This only has to absorb the plane round-trip, which both sides of a joint go
# through together.
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
    """How close counts as touching rather than missing. A T-junction -- one
    polyline ending on another -- is a contact, and real ones are drawn a hair
    out, so the document tolerance is the right default rather than 0."""
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
    """(polylines, rejected) -- rejected holds the input indices that are not
    polylines. Arcs and general NURBS have no straight segments to cross, so
    they are refused rather than approximated by their control structure."""
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
        # Refusing rather than warning: a non-polyline silently reduced to the
        # line between its ends would move the crossings, and nothing in the
        # output would say so.
        print("[geomseq_core] %d of %d input curves are not polylines (indices %s). "
              "Shatter Crossings handles polylines only -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
        segments_tree = th.list_to_tree([[]])
        piece_counts_tree = th.list_to_tree([[]])
    else:
        # --- explode, in plane space ---
        # The crossing test is 2D. Working in plane coordinates puts it on the
        # plane the caller chose, and the out-of-plane offset rides along as z:
        # the native side interpolates z down each segment, so a cut point keeps
        # the height it would have had.
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
        # A run of pieces that touch end-to-start is one surviving polyline. But
        # contiguity alone is not enough to decide that: at D = 0 the two sides
        # of a cut touch as well, and welding them back would make the whole
        # component look like a no-op. Where the piece came from settles it --
        # see below.
        nested_pieces, nested_counts = [], []
        for src_index, start, stop in spans:
            runs, cur = [], []
            for s_i in range(start, stop):
                for k, (s, e) in enumerate(shattered[s_i]):
                    # Only the FIRST piece of a segment can continue the run
                    # before it. Later pieces of the same segment follow a cut
                    # by construction, and at D = 0 a cut is contiguous -- so
                    # testing contiguity alone would silently weld them back
                    # together and the component would appear to do nothing.
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

        # Path {branch; input index}: a polyline cut in two stays addressable by
        # the index it went in with, and one the gaps swallowed whole keeps its
        # path and holds nothing rather than shifting every index after it.
        segments_tree = th.list_to_tree([nested_pieces])
        piece_counts_tree = th.list_to_tree([nested_counts])
