"""
Fill Cells Component for GeomSeq
GH entry point: cuts one closed region into monotone cells and lays a zig-zag
of fill lines across each (geometry: coverage_path.decompose + fill_cells).

Stage 2 of the coverage path. The cut is the same as Decompose Region, so the
inputs repeat; no travel between cells yet, every cell is its own polyline.

Inputs
    curves           closed curves: one outer boundary and any holes
    spacing          distance between fill lines; must be > 0
    angle            direction of the fill lines in degrees, in the plane.
                     Optional, 0
    phase            offset of the line grid across the lines, in the plane.
                     Optional, 0
    plane            working plane. Optional, world XY
    chord_tolerance  largest gap allowed between a curve and its polyline
                     when it is flattened. Optional, 0 = document tolerance
    direction        how every cell is walked. Optional, 0
                     0  bottom to top, from the left end of the lowest line
                     1  0 walked backwards, top to bottom
                     2  bottom to top, from the right end of the lowest line
                     3  2 walked backwards
                     Left, right, bottom and top are in the sweep frame.
Outputs
    fills            one open polyline per cell, walked in direction
    line_counts      fill lines per cell
    cells            closed polyline per cell, as Decompose Region gives them
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


if __name__ == "__main__":
    src_plane = _optional("plane", rg.Plane.WorldXY)
    sweep = math.radians(float(_optional("angle", 0.0)))
    grid_phase = float(_optional("phase", 0.0))
    walk = int(_optional("direction", 0))
    chord_tol = _resolve_chord_tolerance()

    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = _to_rings(raw, src_plane, chord_tol)

    fills, line_counts, cells = [], [], []

    if rejected:
        # One region is one input; a bad ring refuses the whole thing.
        print("[geomseq_core] %d of %d input curves are missing, open, or could not "
              "be flattened (indices %s) -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
    elif rings:
        try:
            if walk not in (0, 1, 2, 3):
                raise ValueError("direction must be 0, 1, 2 or 3")
            dec = decompose(rings, angle=sweep)
            result = fill_cells(dec, float(spacing), phase=grid_phase)  # type: ignore
        except (ValueError, NameError, TypeError) as e:
            print("[geomseq_core] %s" % e)
            result = None

        if result is not None:
            for cell in dec.cells:
                world = [src_plane.PointAt(x, y, 0.0) for x, y in cell.outline]
                world.append(world[0])
                cells.append(rg.PolylineCurve(world))
            for f in result:
                world = [src_plane.PointAt(x, y, 0.0) for x, y in f.walks[walk]]
                fills.append(rg.PolylineCurve(world) if len(world) > 1 else None)
                line_counts.append(f.line_count)
