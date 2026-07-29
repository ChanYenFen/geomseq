"""
Evaluates points on a curve at arc-length lookups (companion to divide_curves.py, which
produces the lookups). RhinoCommon-dependent; GH input handling lives in the gh/ component.
"""


def sample_curve_points(curve, lookups):
    """Evaluates one curve at each arc-length position in `lookups`; a position
    outside the curve's length is silently skipped (LengthParameter fails rather than raising)."""
    points = []
    for length in lookups:
        success, t = curve.LengthParameter(length)
        if success:
            points.append(curve.PointAt(t))
    return points
