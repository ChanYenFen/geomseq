"""
Curves as 2D point lists in a working plane, for the coverage components (Hatch Region,
Plan Coverage, Resample On Lattice). RhinoCommon-dependent; GH input handling lives in gh/.
"""

import Rhino


def resolve_chord_tolerance(value):
    """Positive is kept; 0 or less reads as the document tolerance."""
    if float(value) > 0.0:
        return float(value)
    try:
        return Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
    except Exception:
        return 0.01


def flatten(curve, chord_tol):
    """A polyline's vertices as they are; anything else straightened to within chord_tol."""
    ok, pl = curve.TryGetPolyline()
    if ok and pl is not None:
        return pl
    # The zeros switch off every constraint but the chord height.
    pc = curve.ToPolyline(0, 0, 0, 0, 0, chord_tol, 0, 0, True)
    return None if pc is None else pc.ToPolyline()


def plane_coords(polyline, plane):
    """A polyline's vertices as (x, y) in plane coordinates."""
    out = []
    for k in range(polyline.Count):
        ok, lp = plane.RemapToPlaneSpace(polyline[k])
        p = lp if ok else polyline[k]
        out.append((p.X, p.Y))
    return out


def to_rings(curves, plane, chord_tol):
    """(rings in plane coordinates, indices of unusable curves: missing, open, or unflattenable)."""
    rings, rejected = [], []
    for i, crv in enumerate(curves):
        pl = flatten(crv, chord_tol) if crv is not None and crv.IsClosed else None
        if pl is None or pl.Count < 4:
            rejected.append(i)
        else:
            rings.append(plane_coords(pl, plane))
    return rings, rejected
