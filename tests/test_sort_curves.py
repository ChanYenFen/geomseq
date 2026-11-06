"""Property tests for sort_curves_native (fixtures/sort_curves_cases.json).
Plain CPython, no Rhino: it only touches PointAtStart/PointAtEnd and
Duplicate/Reverse, so _Seg is enough to stand in for a curve."""

import json
import math
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import sort_curves_native

FIXTURES = os.path.join(_HERE, "fixtures", "sort_curves_cases.json")
KNN_K, USE_2OPT, MAX_PASSES = 12, True, 20  # 20 is what sort_curves ships
TOL = 1e-9

with open(FIXTURES, encoding="utf-8") as fh:
    CASES = json.load(fh)


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


@pytest.mark.parametrize("name, segments", CASES.items(), ids=list(CASES.keys()))
def test_sort_curves(name, segments):
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


# --- greedy phase: every step enters a curve at a nearest allowed endpoint ---
# As in test_sort_points.py: the fixtures above are too small to reach the
# kd-tree's interesting paths, so these inputs are big enough to grow the k-NN
# window and to use up whole subtrees. 2-opt is off so the tour is greedy's own,
# and only distances are compared, since lattice endpoints tie.
#
# With if_flip a curve may be entered at either end; without it, only at its
# start. Either way, every endpoint of a placed curve stops being a candidate.

def _random_segments(n, seed, spread=None):
    rng = random.Random(seed)
    centres = None
    if spread is not None:
        centres = [(rng.uniform(0, 1000), rng.uniform(0, 1000)) for _ in range(8)]
    segs = []
    for _ in range(n):
        if centres is None:
            x0, y0 = rng.uniform(0, 1000), rng.uniform(0, 1000)
        else:
            cx, cy = rng.choice(centres)
            x0, y0 = rng.gauss(cx, spread), rng.gauss(cy, spread)
        a, length = rng.uniform(0, 2 * math.pi), rng.uniform(5, 20)
        segs.append(_Seg(x0, y0, x0 + length * math.cos(a), y0 + length * math.sin(a)))
    return segs


def _hatch(side, seed):
    rng = random.Random(seed)
    segs = [_Seg(float(x), float(y), float(x + 1), float(y))
            for x in range(side) for y in range(side)]
    for s in segs:
        if rng.random() < 0.5:
            s.Reverse()
    rng.shuffle(segs)
    return segs


CURVE_GREEDY_CASES = {
    "uniform_800":   lambda: _random_segments(800, 1),
    "clustered_800": lambda: _random_segments(800, 2, spread=10.0),
    "hatch_28x28":   lambda: _hatch(28, 3),
}


def _d2(ax, ay, bx, by):
    dx, dy = ax - bx, ay - by
    return dx * dx + dy * dy


@pytest.mark.parametrize("if_flip", [True, False], ids=["flip", "fixed"])
@pytest.mark.parametrize("name", list(CURVE_GREEDY_CASES), ids=list(CURVE_GREEDY_CASES))
def test_greedy_step_enters_at_a_nearest_allowed_endpoint(name, if_flip):
    curves = CURVE_GREEDY_CASES[name]()
    ordered, order = sort_curves_native(curves, use_two_opt=False, if_flip=if_flip,
                                        knn_k=KNN_K)
    assert sorted(order) == list(range(len(curves)))

    unused = set(range(len(curves)))
    cx, cy = 0.0, 0.0  # start_pt=None means the origin
    for step, (idx, placed) in enumerate(zip(order, ordered)):
        best = min(min(_d2(cx, cy, curves[u].x0, curves[u].y0),
                       _d2(cx, cy, curves[u].x1, curves[u].y1) if if_flip else math.inf)
                   for u in unused)
        got = _d2(cx, cy, placed.x0, placed.y0)
        assert got <= best * (1 + 1e-12), (
            f"{name}: step {step} entered at d^2={got!r}, "
            f"but an allowed endpoint lies at d^2={best!r}")
        unused.remove(idx)
        cx, cy = placed.x1, placed.y1
