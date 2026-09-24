"""
Evaluates points on a curve at arc lengths (companion to divide_curves.py, which
produces the arc_lengths). RhinoCommon-dependent; GH input handling lives in the gh/ component.
"""


def sample_curve_points(curve, arc_lengths):
    """Evaluates one curve at each arc-length position in `arc_lengths`; a position
    outside the curve's length is silently skipped (LengthParameter fails rather than raising)."""
    points = []
    for length in arc_lengths:
        success, t = curve.LengthParameter(length)
        if success:
            points.append(curve.PointAt(t))
    return points
