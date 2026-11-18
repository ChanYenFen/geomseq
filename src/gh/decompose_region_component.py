"""
Decompose Region Component for GeomSeq
GH entry point: cuts one closed region (outer curve plus holes) into monotone
cells along a sweep direction (geometry: coverage_path.decompose).

Stage 1 of the coverage path: no fill lines and no travel yet, just the cells
and the portals between them, to look at on the canvas.

Inputs
    curves           closed curves: one outer boundary and any holes; nesting
                     is read by the even-odd rule, so orientation is free
    angle            sweep direction in degrees, in the plane; the fill lines
                     of a cell will run along it. Optional, 0
    plane            working plane. Optional, world XY
    chord_tolerance  largest gap allowed between a curve and its polyline
                     when it is flattened. Optional, 0 = document tolerance
Outputs
    cells            closed polyline per cell, counter-clockwise in the plane
    portals          one line per portal: the open interval two cells share
    neighbours       tree, branch i = indices of the cells adjacent to cell i
"""

import math
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

# Force-reload so edits show up without restarting Rhino. coverage_path is
# pure Python, so nothing native is reloaded.
_reload.unload_modules("geomseq_core.coverage_path")

from geomseq_core.coverage_path import decompose


def _resolve_plane():
    """Unconnected optional GH input reads as world XY."""
    try:
        value = plane  # type: ignore
    except NameError:
        value = None
    return rg.Plane.WorldXY if value is None else value


def _resolve_angle():
    """Degrees in, radians out. Unconnected reads as 0: sweep along the plane's y."""
    try:
        value = angle  # type: ignore
    except NameError:
        value = None
    return 0.0 if value is None else math.radians(float(value))


def _resolve_chord_tolerance():
    """Unconnected or 0 reads as the document tolerance."""
    try:
        value = chord_tolerance  # type: ignore
    except NameError:
        value = None
    if value is not None and float(value) > 0.0:
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


if __name__ == "__main__":
    src_plane = _resolve_plane()
    sweep = _resolve_angle()
    chord_tol = _resolve_chord_tolerance()

    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = _to_rings(raw, src_plane, chord_tol)

    cells, portals, nested_neighbours = [], [], []

    if rejected:
        # One region is one input; a bad ring refuses the whole thing.
        print("[geomseq_core] %d of %d input curves are missing, open, or could not "
              "be flattened (indices %s) -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
    elif rings:
        try:
            dec = decompose(rings, angle=sweep)
        except ValueError as e:
            print("[geomseq_core] %s" % e)
            dec = None

        if dec is not None:
            for cell in dec.cells:
                world = [src_plane.PointAt(x, y, 0.0) for x, y in cell.outline]
                world.append(world[0])
                cells.append(rg.PolylineCurve(world))
            for p in dec.portals:
                a = src_plane.PointAt(p.p0[0], p.p0[1], 0.0)
                b = src_plane.PointAt(p.p1[0], p.p1[1], 0.0)
                portals.append(rg.Line(a, b))
            nested_neighbours = [dec.neighbours(i) for i in range(len(dec.cells))]

    neighbours = th.list_to_tree(nested_neighbours)
