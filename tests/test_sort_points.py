"""Property tests for sort_points_native (fixtures/sort_points_cases.json).
Plain CPython, no Rhino: it only reads .X/.Y/.Z and re-indexes the same objects,
so _Pt suffices -- no Duplicate/Reverse, points have no direction to flip."""

import json
import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import sort_points_native

FIXTURES = os.path.join(_HERE, "fixtures", "sort_points_cases.json")
KNN_K, USE_2OPT, MAX_PASSES = 12, True, 10
TOL = 1e-9

with open(FIXTURES, encoding="utf-8") as fh:
    CASES = json.load(fh)


class _Pt:
    __slots__ = ("X", "Y", "Z")

    def __init__(self, x, y, z=0.0):
        self.X, self.Y, self.Z = x, y, z


def travel_distance(points):
    """Sum of gaps between consecutive points."""
    return sum(math.hypot(c.X - p.X, c.Y - p.Y) for p, c in zip(points, points[1:]))


@pytest.mark.parametrize("name, coords", CASES.items(), ids=list(CASES.keys()))
def test_sort_points(name, coords):
    points = [_Pt(x, y) for x, y in coords]
    n = len(points)

    ordered, order = sort_points_native(points, use_two_opt=USE_2OPT,
                                        two_opt_max_passes=MAX_PASSES, knn_k=KNN_K)

    assert len(ordered) == n and len(order) == n, f"{name}: expected {n} results"
    assert sorted(order) == list(range(n)), f"{name}: order is not a permutation"

    sorted_dist, naive_dist = travel_distance(ordered), travel_distance(points)
    assert sorted_dist <= naive_dist + TOL, (
        f"{name}: sorted travel {sorted_dist:.6f} exceeds naive {naive_dist:.6f}")

    print(f"{name:<28} travel={sorted_dist:12.3f}   naive={naive_dist:12.3f}")
