"""Tests for point_in_curve; cases are inline, plain floats.

Unlike the other test modules, these have no Python mirror of the C++ to fall
back on: every assertion goes through ctypes into the committed binary, so a
stale binary fails them rather than passing against a mirror.

The file is in two halves. Above CLASSIFICATION is the plumbing around the
core -- input validation, output shapes, the bounding-box rejection. Below it
is what the core itself has to get right: the boundary within tol, a ray
through a vertex, and a concave shape.
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
    PIC_WARN_FLAT_AREA,
    point_in_curve_native,
)

# A closed unit square, counter-clockwise, first vertex repeated as last.
SQUARE = [(0.0, 0.0), (1.0, 0.0), (1.0, 1.0), (0.0, 1.0), (0.0, 0.0)]

# Concave, so a crossing count has to survive a ray that re-enters:
#
#   (0,2) +---+ (1,2)
#         |   |
#   (0,1) +   +---+ (2,1)
#         |       |
#   (0,0) +-------+ (2,0)
L_SHAPE = [(0.0, 0.0), (2.0, 0.0), (2.0, 1.0), (1.0, 1.0),
           (1.0, 2.0), (0.0, 2.0), (0.0, 0.0)]

TOL = 1e-9


def _status(polyline=None, query=None, tol=TOL):
    _, status = point_in_curve_native([polyline or SQUARE],
                                      query or [(0.5, 0.5)], tol=tol)
    return status


# --- input validation --------------------------------------------------------

def test_negative_tol_is_refused():
    with pytest.raises(ValueError):
        point_in_curve_native([SQUARE], [(0.5, 0.5)], tol=-1.0)


def test_two_polylines_are_refused_in_this_stage():
    """The ABI already carries a count; the implementation does not yet."""
    with pytest.raises(ValueError) as e:
        point_in_curve_native([SQUARE, SQUARE], [(0.5, 0.5)], tol=TOL)
    assert "one polyline" in str(e.value)


def test_too_few_vertices_is_refused():
    with pytest.raises(ValueError) as e:
        point_in_curve_native([[(0.0, 0.0), (1.0, 0.0), (0.0, 0.0)]],
                              [(0.5, 0.5)], tol=TOL)
    assert "4 vertices" in str(e.value)


def test_open_polyline_is_refused():
    """Rhino answers Unset for an open curve; we answer with a status."""
    open_square = SQUARE[:-1]
    with pytest.raises(ValueError) as e:
        point_in_curve_native([open_square], [(0.5, 0.5)], tol=TOL)
    assert "not closed" in str(e.value)


def test_closure_is_judged_within_tol():
    """A gap under tol closes the polyline; the same gap over it does not."""
    nearly = SQUARE[:-1] + [(0.0, 0.01)]

    point_in_curve_native([nearly], [(0.5, 0.5)], tol=0.1)  # closes

    with pytest.raises(ValueError):
        point_in_curve_native([nearly], [(0.5, 0.5)], tol=1e-6)


# --- output shape ------------------------------------------------------------

def test_no_query_points_returns_empty():
    relationship, _ = point_in_curve_native([SQUARE], [], tol=TOL)
    assert relationship == []


def test_relationship_has_one_entry_per_query_point():
    query = [(0.5, 0.5), (9.0, 9.0), (0.0, 0.0), (-1.0, 0.5)]
    relationship, _ = point_in_curve_native([SQUARE], query, tol=TOL)
    assert len(relationship) == len(query)


def test_vertices_may_carry_a_z_that_is_ignored():
    """The caller projects; only x and y are read."""
    square_3d = [(x, y, 7.0) for (x, y) in SQUARE]
    a, _ = point_in_curve_native([square_3d], [(0.5, 0.5, 7.0)], tol=TOL)
    b, _ = point_in_curve_native([SQUARE], [(0.5, 0.5)], tol=TOL)
    assert a == b


# --- bounding-box rejection --------------------------------------------------
#
# The box test is provable, not a heuristic: dist_to_box is a lower bound on
# the distance to every point of the polyline, so a point further than tol
# from the box is neither on the boundary nor inside it. These check that the
# bound is applied with the right sign -- a point just outside the box is
# rejected, one just inside it is not decided by the box alone.

def test_far_point_is_outside():
    relationship, _ = point_in_curve_native([SQUARE], [(100.0, 100.0)], tol=TOL)
    assert relationship == [PIC_OUTSIDE]


def test_point_just_outside_the_box_is_outside():
    relationship, _ = point_in_curve_native([SQUARE], [(1.0 + 1e-3, 0.5)], tol=1e-9)
    assert relationship == [PIC_OUTSIDE]


def test_tol_widens_the_box():
    """Just outside the box, but within tol of the boundary: coincident."""
    relationship, _ = point_in_curve_native([SQUARE], [(1.0 + 1e-3, 0.5)], tol=1e-2)
    assert relationship == [PIC_COINCIDENT]


# --- CLASSIFICATION ----------------------------------------------------------
#
# What crossing_number_parity and distance_to_polyline have to get right.

def test_centre_is_inside():
    relationship, _ = point_in_curve_native([SQUARE], [(0.5, 0.5)], tol=TOL)
    assert relationship == [PIC_INSIDE]


def test_status_is_ok_for_a_healthy_square():
    assert _status() == PIC_OK


@pytest.mark.parametrize("q", [
    (0.0, 0.0),    # a vertex
    (0.5, 0.0),    # mid-edge
    (1.0, 0.5),    # mid-edge, other side
])
def test_boundary_points_are_coincident(q):
    relationship, _ = point_in_curve_native([SQUARE], [q], tol=1e-6)
    assert relationship == [PIC_COINCIDENT]


def test_ray_through_a_vertex_is_not_double_counted():
    """y = 1.0 passes exactly through two of the square's corners. A naive
    crossing count sees two crossings there and calls the point outside."""
    relationship, _ = point_in_curve_native([SQUARE], [(0.5, 1.0 - 1e-7)], tol=1e-12)
    assert relationship == [PIC_INSIDE]


@pytest.mark.parametrize("q,expected", [
    ((0.5, 0.5),  PIC_INSIDE),    # in the thick part
    ((1.5, 0.5),  PIC_INSIDE),    # in the arm
    ((1.5, 1.5),  PIC_OUTSIDE),   # in the notch -- the case a convex test fails
    ((0.5, 1.5),  PIC_INSIDE),    # above the notch
])
def test_concave_shape(q, expected):
    relationship, _ = point_in_curve_native([L_SHAPE], [q], tol=1e-9)
    assert relationship == [expected]


def test_collapsed_projection_warns():
    """A curve whose plane is near perpendicular to the working plane projects
    to something with no area, and containment stops meaning anything."""
    sliver = [(0.0, 0.0), (1.0, 0.0), (1.0, 1e-12), (0.0, 1e-12), (0.0, 0.0)]
    _, status = point_in_curve_native([sliver], [(0.5, 0.0)], tol=1e-15)
    assert status == PIC_WARN_FLAT_AREA


def test_winding_direction_does_not_matter():
    clockwise = list(reversed(SQUARE))
    a, _ = point_in_curve_native([SQUARE], [(0.5, 0.5)], tol=TOL)
    b, _ = point_in_curve_native([clockwise], [(0.5, 0.5)], tol=TOL)
    assert a == b == [PIC_INSIDE]
