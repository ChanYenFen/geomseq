"""
Travel Between Cells Component for GeomSeq
GH entry point: the travel from the end of one cell's fill to the start of
another's, down the middle of the cells not yet stitched (geometry:
coverage_path.travel).

Stage 3 of the coverage path. Cells and fills are as Fill Cells gives them;
the stitch order is still chosen by hand, through from_cell and to_cell. The
travel runs through the midpoints of the fill lines, cell to cell through
the midpoints of the portals, so it keeps to the middle of the fabric and
never touches a boundary corner. There is always a travel, never a trim:
it stays under unstitched cells when they join the two ends, and otherwise
crosses stitched ones, showing as little as it can.

Inputs
    curves           closed curves: one outer boundary and any holes
    spacing          distance between fill lines; must be > 0
    angle            direction of the fill lines in degrees. Optional, 0
    phase            offset of the line grid. Optional, 0
    from_cell        index of the cell stitched last; the travel starts at
                     the end of its fill, walked in from_dir
    to_cell          index of the cell stitched next; the travel ends at the
                     start of its fill, walked in to_dir
    stitched         indices of cells already stitched. Optional; the travel
                     avoids them and, where it cannot, crosses them as
                     visible length. from_cell is always stitched: a travel
                     back across it shows like over any other
    plane            working plane. Optional, world XY
    chord_tolerance  largest gap allowed between a curve and its polyline
                     when it is flattened. Optional, 0 = document tolerance
    from_dir         how from_cell was walked, as Fill Cells' direction:
                     0 forward, 1 reverse, 2 mirrored, 3 mirrored reverse.
                     Optional, 0
    to_dir           how to_cell will be walked, same numbering. Optional, 0
Outputs
    travel           polyline from the end of from_cell's fill to the start
                     of to_cell's. None only when the region is in separate
                     pieces and the two cells are in different ones
    length           length of the travel, or 0 when there is none
    visible          length of the travel over stitched cells, which no
                     later stitching covers; 0 when it is fully hidden.
                     Appended output: add it to the component in GH
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

# The output is named `travel`; the function must not shadow it.
from geomseq_core.coverage_path import decompose, fill_cells, visible_length
from geomseq_core.coverage_path import travel as find_travel


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
    chord_tol = _resolve_chord_tolerance()

    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = _to_rings(raw, src_plane, chord_tol)

    travel, length, visible = None, 0.0, 0.0

    if rejected:
        # One region is one input; a bad ring refuses the whole thing.
        print("[geomseq_core] %d of %d input curves are missing, open, or could not "
              "be flattened (indices %s) -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
    elif rings:
        try:
            dec = decompose(rings, angle=sweep)
            fills = fill_cells(dec, float(spacing), phase=grid_phase)  # type: ignore
            start_cell, goal_cell = int(from_cell), int(to_cell)  # type: ignore
            done = set(int(i) for i in _optional("stitched", []))
            done.discard(goal_cell)
            done.add(start_cell)
            unstitched = [i for i in range(len(dec.cells)) if i not in done]

            start_dir = int(_optional("from_dir", 0))
            goal_dir = int(_optional("to_dir", 0))
            if start_dir not in (0, 1, 2, 3) or goal_dir not in (0, 1, 2, 3):
                raise ValueError("from_dir and to_dir must be 0, 1, 2 or 3")
            start = fills[start_cell].walks[start_dir][-1]
            goal = fills[goal_cell].walks[goal_dir][0]
            path = find_travel(dec, fills, unstitched, start, goal)
            visible = visible_length(path, dec, fills, unstitched)
        except (ValueError, NameError, TypeError, IndexError) as e:
            print("[geomseq_core] %s" % e)
            path = None

        if path is not None:
            world = [src_plane.PointAt(p[0], p[1], 0.0) for p in path]
            travel = rg.PolylineCurve(world) if len(world) > 1 else None
            length = sum(a.DistanceTo(b) for a, b in zip(world, world[1:]))
            if visible > 0.0:
                print("[geomseq_core] %.4g of the travel shows over stitched cells" % visible)
