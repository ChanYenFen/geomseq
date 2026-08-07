"""Property tests for sort_curves_native (fixtures/sort_curves_cases.json).

Plain CPython, no Rhino: sort_curves_native only touches PointAtStart/PointAtEnd
and Duplicate/Reverse, so _Seg is enough to stand in for a curve.
"""

import json
import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import sort_curves_native

FIXTURES = os.path.join(_HERE, "fixtures", "sort_curves_cases.json")
KNN_K, USE_2OPT, MAX_PASSES = 12, True, 10
TOL = 1e-9


class _Pt:
    __slots__ = ("X", "Y", "Z")

    def __init__(self, x, y, z=0.0):
        self.X, self.Y, self.Z = x, y, z


class _Seg:
    """Stands in for a Rhino curve, backed by [x0, y0, x1, y1]."""

    __slots__ = ("x0", "y0", "x1", "y1")

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1

    @property
    def PointAtStart(self):
        return _Pt(self.x0, self.y0)

    @property
    def PointAtEnd(self):
        return _Pt(self.x1, self.y1)

    def Duplicate(self):
        return _Seg(self.x0, self.y0, self.x1, self.y1)

    def Reverse(self):
        self.x0, self.x1 = self.x1, self.x0
        self.y0, self.y1 = self.y1, self.y0


def travel_distance(curves):
    """Sum of gaps: end of one curve to start of the next."""
    return sum(math.hypot(c.PointAtStart.X - p.PointAtEnd.X,
                          c.PointAtStart.Y - p.PointAtEnd.Y)
               for p, c in zip(curves, curves[1:]))


def check_case(name, segments):
    curves = [_Seg(*s) for s in segments]
    n = len(curves)

    ordered, order = sort_curves_native(curves, use_two_opt=USE_2OPT,
                                        two_opt_max_passes=MAX_PASSES, knn_k=KNN_K)

    assert len(ordered) == n and len(order) == n, f"{name}: expected {n} results"
    assert sorted(order) == list(range(n)), f"{name}: order is not a permutation"

    sorted_dist, naive_dist = travel_distance(ordered), travel_distance(curves)
    assert sorted_dist <= naive_dist + TOL, (
        f"{name}: sorted travel {sorted_dist:.6f} exceeds naive {naive_dist:.6f}")

    print(f"{name:<28} travel={sorted_dist:12.3f}   naive={naive_dist:12.3f}")


def main():
    with open(FIXTURES, encoding="utf-8") as fh:
        cases = json.load(fh)
    for name, segments in cases.items():
        check_case(name, segments)
    print("All tests passed")


if __name__ == "__main__":
    main()
