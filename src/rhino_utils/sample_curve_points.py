"""
Evaluates 3D points on a curve at given arc-length positions (lookups), via
Curve.LengthParameter() + Curve.PointAt(). Companion to divide_curves.py --
divide_curves produces lookups from a curve, this turns lookups (typically
redistributed by redistribute_lookups) back into points on that same curve.

Depends on RhinoCommon but not on any GH-specific input handling -- that
lives in gh/sample_curve_points_component.py.
"""


def sample_curve_points(curve, lookups):
    """Evaluates one curve at each arc-length position in `lookups`. A
    position outside the curve's length is silently skipped (LengthParameter
    fails rather than raising)."""
    points = []
    for length in lookups:
        success, t = curve.LengthParameter(length)
        if success:
            points.append(curve.PointAt(t))
    return points
