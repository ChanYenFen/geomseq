"""Property tests for sort_points_native (fixtures/sort_points_cases.json).
Plain CPython, no Rhino: it only reads .X/.Y/.Z and re-indexes the same objects,
so _Pt suffices -- no Duplicate/Reverse, points have no direction to flip."""

import json
import math
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import sort_points_native

FIXTURES = os.path.join(_HERE, "fixtures", "sort_points_cases.json")
KNN_K, USE_2OPT, MAX_PASSES = 12, True, 20  # 20 is what sort_points ships
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


# --- greedy phase: every step takes a nearest unused point ------------------
# The fixtures above are too small to reach the kd-tree's interesting paths: at
# n <= 20 the tree is one or two leaves, the k-NN window never has to grow and
# no subtree is ever fully used. These inputs are big enough for all three, so
# a search that skips a subtree it should not have picks a farther point here.
#
# 2-opt is off because it rewrites the tour and would hide which point greedy
# chose. Only the distance is compared, not the index: on a lattice many points
# tie, and any of them is a correct answer.

def _uniform(n, seed, z=False):
    rng = random.Random(seed)
    return [_Pt(rng.uniform(0, 1000), rng.uniform(0, 1000),
                rng.uniform(0, 1000) if z else 0.0) for _ in range(n)]


def _lattice(side, seed):
    pts = [_Pt(float(x), float(y)) for x in range(side) for y in range(side)]
    random.Random(seed).shuffle(pts)
    return pts


def _clustered(n, seed, clusters=8):
    rng = random.Random(seed)
    centres = [(rng.uniform(0, 1000), rng.uniform(0, 1000)) for _ in range(clusters)]
    pts = []
    for _ in range(n):
        cx, cy = rng.choice(centres)
        pts.append(_Pt(rng.gauss(cx, 10.0), rng.gauss(cy, 10.0)))
    return pts


GREEDY_CASES = {
    "uniform_1000":   lambda: _uniform(1000, 1),
    "uniform_3d_800": lambda: _uniform(800, 2, z=True),
    "lattice_32x32":  lambda: _lattice(32, 3),
    "clustered_1000": lambda: _clustered(1000, 4),
}


def _d2(a, b):
    dx, dy, dz = a.X - b.X, a.Y - b.Y, a.Z - b.Z
    return dx * dx + dy * dy + dz * dz


@pytest.mark.parametrize("name", list(GREEDY_CASES), ids=list(GREEDY_CASES))
def test_greedy_step_takes_a_nearest_unused_point(name):
    points = GREEDY_CASES[name]()
    _, order = sort_points_native(points, use_two_opt=False, knn_k=KNN_K)
    assert sorted(order) == list(range(len(points)))

    unused = set(range(len(points)))
    cur = _Pt(0.0, 0.0)  # start_pt=None means the origin
    for step, idx in enumerate(order):
        best = min(_d2(cur, points[u]) for u in unused)
        got = _d2(cur, points[idx])
        assert got <= best * (1 + 1e-12), (
            f"{name}: step {step} took a point at d^2={got!r}, "
            f"but an unused one lies at d^2={best!r}")
        unused.remove(idx)
        cur = points[idx]
