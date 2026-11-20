"""
Hatch Region Component for GeomSeq
GH entry point: cuts one closed region into monotone cells, lays parallel
fill lines across each, and links each cell's lines into one polyline
(geometry: coverage_path.hatch_region, run in C++ through
geometry_utils.hatch_region_native; the Python module is its reference).

No order between cells and no travel: that is Plan Coverage. Use this when
the cells and their hatching are what you need, e.g. to sequence them
yourself.

Inputs
    curves           closed curves: one outer boundary and any holes
    spacing          distance between fill lines; must be > 0
    angle            direction of the fill lines in degrees, in the plane.
                     Optional, 0
    phase            offset of the line grid across the lines, in the plane.
                     Optional, 0
    link             how each cell's lines join into its path. Optional, 0
                     0  zig-zag: lines alternate direction, each joined to
                        the next along the cell's side
                     1  one-way: lines all run one direction, each joined
                        to the next by the shortest way across the cell
    direction        how every cell's path is walked. Optional, 0
                     0  bottom to top, from the left end of the lowest line
                     1  0 walked backwards, top to bottom
                     2  bottom to top, from the right end of the lowest line
                     3  2 walked backwards
                     Left, right, bottom and top are in the sweep frame.
    plane            working plane. Optional, world XY
    chord_tolerance  largest gap allowed between a curve and its polyline
                     when it is flattened. Optional, 0 = document tolerance
Outputs
    cells            closed polyline per cell
    lines            fill lines, branch i for cell i, bottom to top, each
                     from its left end to its right end
    paths            one open polyline per cell: its lines linked as link
                     says, walked as direction says
"""

import math
import os
import sys

import Rhino
import Rhino.Geometry as rg
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path

try:
    import geomseq_core
except ImportError:
    # Dev mode: geomseq_core isn't installed / not on sys.path yet -- add its
    # parent src/ dir (this file lives in src/gh/, geomseq_core lives in src/geomseq_core/).
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

# Force-reload so edits show up without restarting Rhino. The DLL itself
# stays loaded: a rebuilt one needs Rhino restarted.
_reload.unload_modules("geomseq_core.coverage_path")
_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")

from geomseq_core.coverage_path import ZIGZAG
from geomseq_core.geometry_utils import hatch_region_native


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
    link_mode = int(_optional("link", ZIGZAG))
    walk = int(_optional("direction", 0))
    chord_tol = _resolve_chord_tolerance()

    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = _to_rings(raw, src_plane, chord_tol)

    cells, paths = [], []
    # Built manually: th.list_to_tree() returns None past ~5000 items.
    lines = DataTree[object]()

    def at(p):
        return src_plane.PointAt(p[0], p[1], 0.0)

    if not raw:
        print("[geomseq_core] no curves reached the curves input -- nothing was computed.")
    elif rejected:
        # One region is one input; a bad ring refuses the whole thing.
        print("[geomseq_core] %d of %d input curves are missing, open, or could not "
              "be flattened (indices %s) -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
    elif rings:
        try:
            if walk not in (0, 1, 2, 3):
                raise ValueError("direction must be 0, 1, 2 or 3")
            result = hatch_region_native(rings, float(spacing), angle=sweep,  # type: ignore
                                         phase=grid_phase, link=link_mode)
        except (ValueError, NameError, TypeError) as e:
            print("[geomseq_core] %s" % e)
            result = None

        if result is not None:
            outlines, fills = result
            for outline in outlines:
                world = [at(p) for p in outline]
                world.append(world[0])
                cells.append(rg.PolylineCurve(world))
            for f in fills:
                branch = GH_Path(f.cell)
                lines.EnsurePath(branch)
                for a, b in f.lines:
                    lines.Add(rg.LineCurve(at(a), at(b)), branch)
                world = [at(p) for p in f.walks[walk]]
                paths.append(rg.PolylineCurve(world) if len(world) > 1 else None)
            print("[geomseq_core] %d cells, %d fill lines" % (len(outlines), sum(len(f.lines) for f in fills)))
