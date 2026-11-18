"""
Plan Coverage Path Component for GeomSeq
GH entry point: the whole stitch path over one closed region, cells in
order, each filled in one of its four walks, joined by travels down the
middle of the cells (geometry: coverage_path.plan).

Stage 4 of the coverage path. No trims: every travel exists, and the order
is chosen to show as little of them as possible, then to travel least. A
travel back across the cell just finished shows like any other. Greedy
seed, then a local search over the order with the best walks for each
order found exactly.

Inputs
    curves           closed curves: one outer boundary and any holes
    spacing          distance between fill lines; must be > 0
    angle            direction of the fill lines in degrees. Optional, 0
    phase            offset of the line grid. Optional, 0
    start            where the needle starts. Optional; unconnected, the
                     plan picks the first cell and walk itself
    plane            working plane. Optional, world XY
    chord_tolerance  largest gap allowed between a curve and its polyline
                     when it is flattened. Optional, 0 = document tolerance
Outputs
    path             the whole stitch path as one polyline
    pieces           the path cut into fills and travels, in order
    kinds            one per piece: 0 fill, 1 travel
    order            cell indices in the order stitched
    directions       one per cell in order: the walk it is filled in,
                     0 forward, 1 reverse, 2 mirrored, 3 mirrored reverse
    visible          total travel length over stitched cells, which no
                     later stitching covers; 0 when every travel is hidden
    travel_length    total length of the travels
"""

import math
import os
import sys

import Rhino
import Rhino.Geometry as rg

try:
    import geomseq_core
except ImportError:
    # Dev mode: geomseq_core isn't installed / not on sys.path yet -- add its
    # parent src/ dir (this file lives in src/gh/, geomseq_core lives in src/geomseq_core/).
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

# Force-reload so edits show up without restarting Rhino. coverage_path is
# pure Python, so nothing native is reloaded.
_reload.unload_modules("geomseq_core.coverage_path")

from geomseq_core.coverage_path import decompose, fill_cells
from geomseq_core.coverage_path import plan as make_plan


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


def _resolve_chord_tolerance():
    """Unconnected or 0 reads as the document tolerance."""
    value = _optional("chord_tolerance", 0.0)
    if float(value) > 0.0:
        return float(value)
    try:
        return Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
    except Exception:
        return 0.01


def _flatten(curve, chord_tol):
    """A polyline's vertices are exact; anything else is straightened to
    within chord_tol. Mirrors Discretize in src/gha/Native/GeomSeqCore.cs."""
    ok, pl = curve.TryGetPolyline()
    if ok and pl is not None:
        return pl
    # The zeros switch off every constraint but the chord height.
    pc = curve.ToPolyline(0, 0, 0, 0, 0, chord_tol, 0, 0, True)
    return None if pc is None else pc.ToPolyline()


def _to_rings(curves, src_plane, chord_tol):
    """Returns (rings, rejected): rings in plane coordinates, rejected the
    input indices that are missing, open, or could not be flattened."""
    rings, rejected = [], []
    for i, crv in enumerate(curves):
        if crv is None or not crv.IsClosed:
            rejected.append(i)
            continue
        pl = _flatten(crv, chord_tol)
        if pl is None or pl.Count < 4:
            rejected.append(i)
            continue
        ring = []
        for k in range(pl.Count):
            ok, lp = src_plane.RemapToPlaneSpace(pl[k])
            p = lp if ok else pl[k]
            ring.append((p.X, p.Y))
        rings.append(ring)
    return rings, rejected


def _polyline(points, src_plane):
    world = [src_plane.PointAt(x, y, 0.0) for x, y in points]
    return rg.PolylineCurve(world) if len(world) > 1 else None


if __name__ == "__main__":
    src_plane = _optional("plane", rg.Plane.WorldXY)
    sweep = math.radians(float(_optional("angle", 0.0)))
    grid_phase = float(_optional("phase", 0.0))
    chord_tol = _resolve_chord_tolerance()

    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = _to_rings(raw, src_plane, chord_tol)

    path, pieces, kinds, order, directions = None, [], [], [], []
    visible, travel_length = 0.0, 0.0

    if rejected:
        # One region is one input; a bad ring refuses the whole thing.
        print("[geomseq_core] %d of %d input curves are missing, open, or could not "
              "be flattened (indices %s) -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
    elif rings:
        try:
            dec = decompose(rings, angle=sweep)
            fills = fill_cells(dec, float(spacing), phase=grid_phase)  # type: ignore
            needle = globals().get("start")
            if needle is not None:
                ok, lp = src_plane.RemapToPlaneSpace(needle)
                needle = (lp.X, lp.Y) if ok else (needle.X, needle.Y)
            result = make_plan(dec, fills, start=needle)
        except (ValueError, NameError, TypeError) as e:
            print("[geomseq_core] %s" % e)
            result = None

        if result is not None:
            path = _polyline(result.path, src_plane)
            for piece in result.pieces:
                pieces.append(_polyline(piece.points, src_plane))
                kinds.append(piece.kind)
            order = [c for c, _ in result.order]
            directions = [d for _, d in result.order]
            visible = result.visible_length
            travel_length = result.travel_length
            if visible > 0.0:
                print("[geomseq_core] %.4g of the travel shows over stitched cells" % visible)
