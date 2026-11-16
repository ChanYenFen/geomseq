"""Tests for point_in_curves, the plural twin of point_in_curve.

What is tested here is only what the plural adds: the index output, the rule
for a point several curves contain, and that several curves are accepted at
all. The classification core is shared with the singular, and
test_point_in_curve.py covers it; nothing there is repeated.
"""

import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import (
    PIC_COINCIDENT,
    PIC_INSIDE,
    PIC_OK,
    PIC_OUTSIDE,
    point_in_curve_native,
    point_in_curves_native,
)

TOL = 1e-9


def square(x, y, size):
    """Closed, counter-clockwise, first vertex repeated as last."""
    return [(x, y), (x + size, y), (x + size, y + size), (x, y + size), (x, y)]


UNIT   = square(0.0, 0.0, 1.0)
OUTER  = square(0.0, 0.0, 4.0)       # a donut with HOLE
HOLE   = square(1.0, 1.0, 2.0)
APART  = square(10.0, 10.0, 1.0)     # disjoint from everything above


# --- what the plural accepts ---------------------------------------------

def test_several_polylines_are_accepted():
    rel, idx, status = point_in_curves_native([UNIT, APART], [(0.5, 0.5)], tol=TOL)
    assert status == PIC_OK
    assert rel == [PIC_INSIDE]
    assert idx == [0]


def test_no_polylines_is_refused():
    with pytest.raises(ValueError) as e:
        point_in_curves_native([], [(0.5, 0.5)], tol=TOL)
    assert "one or more" in str(e.value)


def test_a_single_polyline_matches_the_singular():
    """One curve through either export: same relationship, and index 0 or -1."""
    query = [(0.5, 0.5), (9.0, 9.0), (0.0, 0.5), (-1.0, 0.5)]
    single, _          = point_in_curve_native([UNIT], query, tol=TOL)
    rel, idx, _        = point_in_curves_native([UNIT], query, tol=TOL)
    assert rel == single
    assert idx == [0 if r != PIC_OUTSIDE else -1 for r in single]


def test_no_query_points_returns_empty():
    rel, idx, _ = point_in_curves_native([UNIT, APART], [], tol=TOL)
    assert rel == [] and idx == []


# --- the index -----------------------------------------------------------

def test_index_names_the_containing_curve():
    query = [(0.5, 0.5), (10.5, 10.5), (5.0, 5.0)]
    rel, idx, _ = point_in_curves_native([UNIT, APART], query, tol=TOL)
    assert rel == [PIC_INSIDE, PIC_INSIDE, PIC_OUTSIDE]
    assert idx == [0, 1, -1]


def test_outside_every_curve_is_minus_one():
    rel, idx, _ = point_in_curves_native([UNIT, APART], [(5.0, 5.0)], tol=TOL)
    assert rel == [PIC_OUTSIDE]
    assert idx == [-1]


def test_index_follows_input_order():
    """Swap the list and the index swaps with it."""
    _, a, _ = point_in_curves_native([UNIT, APART], [(10.5, 10.5)], tol=TOL)
    _, b, _ = point_in_curves_native([APART, UNIT], [(10.5, 10.5)], tol=TOL)
    assert a == [1] and b == [0]


def test_coincident_reports_that_curve():
    rel, idx, _ = point_in_curves_native([UNIT, APART], [(10.0, 10.5)], tol=TOL)
    assert rel == [PIC_COINCIDENT]
    assert idx == [1]


# --- nesting: the first containing curve in input order wins --------------

def test_nested_point_in_the_hole_answers_for_the_first_listed():
    """(2,2) is inside OUTER and inside HOLE. Listed outer-first it answers
    for the outer; listed hole-first, for the hole. The rule is the order
    given, not depth."""
    _, outer_first, _ = point_in_curves_native([OUTER, HOLE], [(2.0, 2.0)], tol=TOL)
    _, hole_first,  _ = point_in_curves_native([HOLE, OUTER], [(2.0, 2.0)], tol=TOL)
    assert outer_first == [0]
    assert hole_first  == [0]


def test_nested_point_in_the_ring_answers_for_the_outer_only():
    """(0.5, 0.5) is inside OUTER but not HOLE, whichever is listed first."""
    rel, a, _ = point_in_curves_native([OUTER, HOLE], [(0.5, 0.5)], tol=TOL)
    _,   b, _ = point_in_curves_native([HOLE, OUTER], [(0.5, 0.5)], tol=TOL)
    assert rel == [PIC_INSIDE]
    assert a == [0] and b == [1]


def test_on_the_hole_boundary_listed_second_is_still_inside_the_outer():
    """(1,2) lies on HOLE's edge and inside OUTER. Outer-first, the outer
    claims it as inside before the hole is reached: the stop-on-first rule
    does not promote a later coincidence over an earlier containment."""
    rel, idx, _ = point_in_curves_native([OUTER, HOLE], [(1.0, 2.0)], tol=TOL)
    assert rel == [PIC_INSIDE]
    assert idx == [0]

    rel, idx, _ = point_in_curves_native([HOLE, OUTER], [(1.0, 2.0)], tol=TOL)
    assert rel == [PIC_COINCIDENT]
    assert idx == [0]


def test_shared_edge_goes_to_the_first_listed():
    """Two squares sharing the edge x=1. A point on it is coincident with
    both; the first listed answers."""
    left, right = square(0.0, 0.0, 1.0), square(1.0, 0.0, 1.0)
    rel, idx, _ = point_in_curves_native([left, right], [(1.0, 0.5)], tol=TOL)
    assert rel == [PIC_COINCIDENT]
    assert idx == [0]
    _, idx, _ = point_in_curves_native([right, left], [(1.0, 0.5)], tol=TOL)
    assert idx == [0]
