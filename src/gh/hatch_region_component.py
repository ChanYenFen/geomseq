"""
Hatch Region Component for GeomSeq
GH entry point: cells, lines and per-cell paths over one closed region (geometry_utils.hatch_region_native).
"""

import math
import os
import sys

import Rhino.Geometry as rg
from Grasshopper import DataTree
from Grasshopper.Kernel.Data import GH_Path

try:
    import geomseq_core
    import rhino_utils
except ImportError:
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

_reload.unload_modules("geomseq_core.coverage_path")
_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")
_reload.unload_modules("rhino_utils.plane_rings")

from geomseq_core.coverage_path import ZIGZAG
from geomseq_core.geometry_utils import hatch_region_native
from rhino_utils.plane_rings import resolve_chord_tolerance, to_rings


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


if __name__ == "__main__":
    src_plane = _optional("plane", rg.Plane.WorldXY)
    sweep = math.radians(float(_optional("angle", 0.0)))
    grid_phase = float(_optional("phase", 0.0))
    link_mode = int(_optional("link", ZIGZAG))
    walk = int(_optional("direction", 0))
    chord_tol = resolve_chord_tolerance(_optional("chord_tolerance", 0.0))

    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = to_rings(raw, src_plane, chord_tol)

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
            print("[geomseq_core] %d cells, %d lines" % (len(outlines), sum(len(f.lines) for f in fills)))
