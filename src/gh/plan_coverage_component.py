"""
Plan Coverage Component for GeomSeq
GH entry point: one path over a closed region, cells in order joined by travels (geometry_utils.plan_coverage_native).
"""

import math
import os
import time
import sys

import Rhino.Geometry as rg

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

from geomseq_core.coverage_path import KICKS, ZIGZAG
from geomseq_core.geometry_utils import plan_coverage_native
from rhino_utils.plane_rings import resolve_chord_tolerance, to_rings


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


def _polyline(points, src_plane):
    world = [src_plane.PointAt(x, y, 0.0) for x, y in points]
    return rg.PolylineCurve(world) if len(world) > 1 else None


if __name__ == "__main__":
    src_plane = _optional("plane", rg.Plane.WorldXY)
    sweep = math.radians(float(_optional("angle", 0.0)))
    grid_phase = float(_optional("phase", 0.0))
    link_mode = int(_optional("link", ZIGZAG))
    restarts = int(_optional("kicks", KICKS))
    chord_tol = resolve_chord_tolerance(_optional("chord_tolerance", 0.0))

    t0 = time.perf_counter()
    raw = list(curves) if curves else []  # type: ignore
    rings, rejected = to_rings(raw, src_plane, chord_tol)

    path, pieces, kinds, order, directions = None, [], [], [], []
    visible, travel_length = 0.0, 0.0

    if not raw:
        print("[geomseq_core] no curves reached the curves input -- nothing was computed.")
    elif rejected:
        # One region is one input; a bad ring refuses the whole thing.
        print("[geomseq_core] %d of %d input curves are missing, open, or could not "
              "be flattened (indices %s) -- nothing was computed."
              % (len(rejected), len(raw), rejected[:20]))
    elif rings:
        try:
            origin = globals().get("start")
            if origin is not None:
                ok, lp = src_plane.RemapToPlaneSpace(origin)
                origin = (lp.X, lp.Y) if ok else (origin.X, origin.Y)
            _, _, result = plan_coverage_native(rings, float(spacing), angle=sweep,  # type: ignore
                                                phase=grid_phase, link=link_mode,
                                                start=origin, kicks=restarts)
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
            print("[geomseq_core] %d cells in order, %d pieces, travel %.4g, visible %.4g, "
                  "%.2f s" % (len(order), len(result.pieces), travel_length, visible,
                              time.perf_counter() - t0))
            if visible > 0.0:
                print("[geomseq_core] %.4g of the travel lies over filled cells" % visible)
