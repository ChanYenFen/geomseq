"""The C++ coverage path against the pure-Python one, the reference.

Each native stage must give what coverage_path.py gives for the same input:
the same cells, the same fill lines, the same four walks of each cell, to
within rounding. Shapes are the Python tests' own plus a few that stress the
fill: a bent side the one-way join has to go round, a sawtooth, sweep angles
that put no vertex on a grid line.
"""

import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))
sys.path.insert(0, _HERE)

from geomseq_core.coverage_path import ONE_WAY, ZIGZAG, hatch_region
from geomseq_core.geometry_utils import hatch_region_native
from test_coverage_path import BENT, COMB, DIAMOND, HOLE, HOURGLASS, SQUARE, U_SHAPE, _holes_row

SAWTOOTH = [(0.0, 0.0), (6.0, 0.0), (6.0, 3.0), (5.0, 2.0), (4.0, 3.0), (3.0, 2.0),
            (2.0, 3.0), (1.0, 2.0), (0.0, 3.0)]

SHAPES = {
    "square": [SQUARE],
    "donut": [SQUARE, HOLE],
    "u": [U_SHAPE],
    "comb": [COMB],
    "hourglass": [HOURGLASS],
    "diamond": [DIAMOND],
    "bent": [BENT],
    "sawtooth": [SAWTOOTH],
    "holes3": _holes_row(3),
    "closed-ring": [SQUARE + [SQUARE[0]], HOLE],
}


def _close(a, b, tol=1e-9):
    return math.dist(a, b) <= tol


def _same_points(ps, qs):
    assert len(ps) == len(qs)
    for p, q in zip(ps, qs):
        assert _close(p, q), (p, q)


@pytest.mark.parametrize("link", [ZIGZAG, ONE_WAY], ids=["zigzag", "one_way"])
@pytest.mark.parametrize("angle", [0.0, 0.5, math.pi / 2, 2.0])
@pytest.mark.parametrize("name", sorted(SHAPES))
def test_hatch_region_matches_python(name, angle, link):
    rings = SHAPES[name]
    for spacing, phase in ((0.3, 0.1), (1.0, 0.0), (0.25, 0.05)):
        dec, fills = hatch_region(rings, spacing, angle=angle, phase=phase, link=link)
        outlines, nfills = hatch_region_native(rings, spacing, angle=angle, phase=phase, link=link)
        assert len(outlines) == len(dec.cells)
        for cell, outline in zip(dec.cells, outlines):
            _same_points(cell.outline, outline)
        assert len(nfills) == len(fills)
        for f, g in zip(fills, nfills):
            assert len(f.lines) == len(g.lines)
            for (a0, a1), (b0, b1) in zip(f.lines, g.lines):
                assert _close(a0, b0) and _close(a1, b1)
            for d in range(4):
                _same_points(f.walks[d], g.walks[d])


@pytest.mark.parametrize("rings,kwargs,message", [
    ([SQUARE], {"spacing": 0.0}, "spacing"),
    ([SQUARE], {"spacing": 1.0, "link": 2}, "link"),
    ([SQUARE], {"spacing": 1.0, "tol": -1.0}, "tol"),
    ([[(0.0, 0.0), (1.0, 0.0)]], {"spacing": 1.0}, "three distinct"),
    ([[(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]], {"spacing": 1.0}, "no height"),
], ids=["spacing", "link", "tol", "short-ring", "flat"])
def test_native_refuses_what_python_refuses(rings, kwargs, message):
    with pytest.raises(ValueError, match=message):
        hatch_region(rings, **kwargs)
    with pytest.raises(ValueError, match=message):
        hatch_region_native(rings, **kwargs)


# --- the plan ----------------------------------------------------------------

from geomseq_core.coverage_path import _Router, decompose, fill_cells, plan
from geomseq_core.geometry_utils import plan_coverage_native
from test_coverage_path import _check_plan, _cost_close, _order_cost

PLAN_SHAPES = ["square", "donut", "u", "comb", "hourglass", "bent", "sawtooth", "holes3"]


@pytest.mark.parametrize("link", [ZIGZAG, ONE_WAY], ids=["zigzag", "one_way"])
@pytest.mark.parametrize("angle", [0.0, 0.5])
@pytest.mark.parametrize("name", PLAN_SHAPES)
def test_plan_matches_python(name, angle, link):
    """Same order, same walks, same pieces: the port replays the reference
    step for step, ties included."""
    rings = SHAPES[name]
    for start in (None, (0.1, 0.1)):
        dec = decompose(rings, angle=angle)
        fills = fill_cells(dec, 0.3, phase=0.1, link=link)
        if start is not None and not any(_inside(c.outline, start) for c in dec.cells):
            continue
        ref = plan(dec, fills, start=start)
        _, _, got = plan_coverage_native(rings, 0.3, angle=angle, phase=0.1, link=link, start=start)
        _check_plan(dec, fills, got)
        assert got.order == ref.order
        assert len(got.pieces) == len(ref.pieces)
        for a, b in zip(got.pieces, ref.pieces):
            assert a.kind == b.kind
            _same_points(a.points, b.points)
            assert a.visible == pytest.approx(b.visible, abs=1e-9)


@pytest.mark.parametrize("name", ["comb", "holes3", "sawtooth"])
def test_kicks_match_python(name):
    """The kicks draw the same random cuts as Python's random.Random(0)."""
    rings = SHAPES[name]
    dec = decompose(rings, angle=0.5)
    fills = fill_cells(dec, 0.3, phase=0.1)
    ref = plan(dec, fills, kicks=5)
    _, _, got = plan_coverage_native(rings, 0.3, angle=0.5, phase=0.1, kicks=5)
    assert got.order == ref.order
    router = _Router(dec, fills, 1e-9)
    assert _cost_close(_order_cost(router, fills, [c for c, _ in got.order], [d for _, d in got.order]),
                       _order_cost(router, fills, [c for c, _ in ref.order], [d for _, d in ref.order]))


def test_native_plan_refuses_negative_kicks():
    with pytest.raises(ValueError, match="kicks"):
        plan_coverage_native([SQUARE], 1.0, kicks=-1)


def _inside(ring, p):
    from geomseq_core.coverage_path import _in_closed_polygon
    return _in_closed_polygon(p, ring, 1e-9)
