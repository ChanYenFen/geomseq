"""
Resample On Lattice Component for GeomSeq
GH entry point: Plan Coverage's pieces onto a staggered lattice (resample_on_lattice, pure Python).
"""

import math
import os
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

_reload.unload_modules("geomseq_core.resample_on_lattice")
_reload.unload_modules("rhino_utils.plane_rings")

from geomseq_core.resample_on_lattice import resample_on_lattice
from rhino_utils.plane_rings import plane_coords

TRAVEL = 1


def _optional(name, default):
    """Unconnected optional GH input (missing or None) reads as default."""
    value = globals().get(name)
    return default if value is None else value


if __name__ == "__main__":
    src_plane = _optional("plane", rg.Plane.WorldXY)
    sweep = math.radians(float(_optional("angle", 0.0)))
    grid_phase = float(_optional("phase", 0.0))
    rows_per_repeat = float(_optional("stagger", 4.0))
    free = float(_optional("free_step", 0.0))
    shortest = float(_optional("min_step", 0.0))

    points, polyline = [], None
    curves = list(pieces) if pieces else []  # type: ignore
    tags = list(kinds) if kinds else []  # type: ignore
    if not curves:
        print("[geomseq_core] no curves reached the pieces input -- nothing was computed.")
    elif len(tags) != len(curves):
        print("[geomseq_core] %d pieces but %d kinds -- connect Plan Coverage's pieces "
              "and kinds together." % (len(curves), len(tags)))
    else:
        runs, bad = [], []
        for i, (crv, kind) in enumerate(zip(curves, tags)):
            ok, pl = crv.TryGetPolyline() if crv is not None else (False, None)
            if not ok or pl is None:
                bad.append(i)
            else:
                runs.append((plane_coords(pl, src_plane), int(kind) == TRAVEL))
        if bad:
            print("[geomseq_core] pieces %s are missing or not polylines -- nothing was computed."
                  % bad[:20])
        else:
            try:
                pts = resample_on_lattice(
                    runs, float(spacing), float(step), angle=sweep, phase=grid_phase,  # type: ignore
                    stagger=rows_per_repeat, free_step=None if free == 0.0 else free,
                    min_step=None if shortest == 0.0 else max(shortest, 0.0))
            except (ValueError, NameError, TypeError) as e:
                print("[geomseq_core] %s" % e)
                pts = []
            points = [src_plane.PointAt(x, y, 0.0) for x, y in pts]
            if len(points) > 1:
                polyline = rg.PolylineCurve(points)
                lengths = [points[i].DistanceTo(points[i + 1]) for i in range(len(points) - 1)]
                print("[geomseq_core] %d points, steps %.3g to %.3g long"
                      % (len(points), min(lengths), max(lengths)))
