"""Property tests for shatter_at_crossings; cases are inline, plain floats.

What these exercise is the ALGORITHM, not the compiled code. reference_shatter below
mirrors native/geometry2d_staging.cpp in Python and is what the assertions run
against, so they still pass when the committed DLL is stale -- which it is whenever
Rhino has it open and it could not be rebuilt. Point `shatter` at
geometry_utils.shatter_at_crossings_native to run the same expectations through
ctypes instead; they are written to hold either way.
"""

import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import (
    SHATTER_FIRST_GUESS,
    shatter_at_crossings_native,
)

EPS = 1e-9
TOL = 1e-9


# --- Python mirror of native/geometry2d_staging.cpp -------------------------

def _sgn_tol(v, tol):
    t = tol if tol > EPS else EPS
    if v > t:
        return 1
    if v < -t:
        return -1
    return 0


def _cross(p, q, r):
    return (q[0] - p[0]) * (r[1] - p[1]) - (q[1] - p[1]) * (r[0] - p[0])


def _pt_dist(p, q):
    return math.hypot(q[0] - p[0], q[1] - p[1])


def _on_segment(p, q, r, tol):
    return (min(p[0], q[0]) - tol <= r[0] <= max(p[0], q[0]) + tol and
            min(p[1], q[1]) - tol <= r[1] <= max(p[1], q[1]) + tol)


def _project_t(p, q, r):
    dx, dy = q[0] - p[0], q[1] - p[1]
    return ((r[0] - p[0]) * dx + (r[1] - p[1]) * dy) / (dx * dx + dy * dy)


def _contact_param(A, B, C, D, touch_tol):
    """Parameter along A->B where AB meets CD, or None. Covers a transversal
    crossing and a touch (an endpoint of one landing on the other). Collinear
    overlap returns None: no single point to centre a gap on."""
    ab_len, cd_len = _pt_dist(A, B), _pt_dist(C, D)
    if ab_len < EPS or cd_len < EPS:
        return None

    g1 = _sgn_tol(_cross(A, B, C) / ab_len, touch_tol)
    g2 = _sgn_tol(_cross(A, B, D) / ab_len, touch_tol)
    g3 = _sgn_tol(_cross(C, D, A) / cd_len, touch_tol)
    g4 = _sgn_tol(_cross(C, D, B) / cd_len, touch_tol)

    if g1 == 0 and g2 == 0:
        return None
    if g3 == 0 and g4 == 0:
        return None

    if g1 * g2 < 0 and g3 * g4 < 0:
        d3, d4 = _cross(C, D, A), _cross(C, D, B)
        denom = d3 - d4
        if abs(denom) < EPS:
            return None
        return d3 / denom

    span = touch_tol if touch_tol > EPS else EPS
    if g3 == 0 and _on_segment(C, D, A, span):
        return 0.0
    if g4 == 0 and _on_segment(C, D, B, span):
        return 1.0
    if g1 == 0 and _on_segment(A, B, C, span):
        return _project_t(A, B, C)
    if g2 == 0 and _on_segment(A, B, D, span):
        return _project_t(A, B, D)

    return None


def reference_shatter(segments, gap_d, touch_tol=0.0):
    """Mirror of shatter_at_crossings with segment_owner=None. Segments are
    ((x,y,z), (x,y,z)) pairs; returns one sub-list of pieces per input."""
    result = []

    for i, (a, b) in enumerate(segments):
        dx, dy = b[0] - a[0], b[1] - a[1]
        length = math.hypot(dx, dy)
        pieces = []

        def emit(t0, t1):
            if length >= EPS and (t1 - t0) * length < EPS:
                return
            pieces.append((
                (a[0] + t0 * dx, a[1] + t0 * dy, a[2] + t0 * (b[2] - a[2])),
                (a[0] + t1 * dx, a[1] + t1 * dy, a[2] + t1 * (b[2] - a[2])),
            ))

        if length < EPS:
            emit(0.0, 1.0)
            result.append(pieces)
            continue

        dt = (gap_d * 0.5) / length

        removed = []
        for j in range(i):  # input order decides who yields: only j < i cuts i
            t = _contact_param(a, b, segments[j][0], segments[j][1], touch_tol)
            if t is not None:
                removed.append((t - dt, t + dt))

        removed.sort()
        cursor = 0.0
        for lo, hi in removed:
            lo = min(lo, 1.0)
            if lo > cursor:
                emit(cursor, lo)
            if hi > cursor:
                cursor = hi
        if cursor < 1.0:
            emit(cursor, 1.0)

        result.append(pieces)

    return result


shatter = reference_shatter


# --- helpers ----------------------------------------------------------------

def dist(p, q):
    return math.hypot(q[0] - p[0], q[1] - p[1])


def seg(x0, y0, x1, y1, z0=0.0, z1=0.0):
    return ((x0, y0, z0), (x1, y1, z1))


# The hand-verified case from the feature spec: a vertical line and a diagonal
# crossing it at (63, 69.236), gap 6 (so 3 comes off either side).
L1 = seg(63.0, 110.0, 63.0, 3.0)
L2 = seg(25.0, 94.0, 114.0, 36.0)
CROSSING = (63.0, 69.23595505617978)


# --- the three rules --------------------------------------------------------

def test_known_example_matches_hand_computed_result():
    out = shatter([L1, L2], 6.0)

    assert len(out) == 2
    assert len(out[0]) == 1, "the lower index is left whole"
    assert len(out[1]) == 2, "the higher index yields, in two pieces"

    assert out[0][0][0] == pytest.approx(L1[0], abs=TOL)
    assert out[0][0][1] == pytest.approx(L1[1], abs=TOL)

    (p0_s, p0_e), (p1_s, p1_e) = out[1]
    assert p0_s[:2] == pytest.approx((25.0, 94.0), abs=TOL)
    assert p0_e[:2] == pytest.approx((60.48, 70.87), abs=0.01)
    assert p1_s[:2] == pytest.approx((65.51, 67.59), abs=0.01)
    assert p1_e[:2] == pytest.approx((114.0, 36.0), abs=TOL)


def test_gap_is_exactly_d_and_centred_on_the_crossing():
    gap_d = 6.0
    (_, p0_e), (p1_s, _) = shatter([L1, L2], gap_d)[1]

    assert dist(p0_e, CROSSING) == pytest.approx(gap_d / 2, abs=1e-6)
    assert dist(CROSSING, p1_s) == pytest.approx(gap_d / 2, abs=1e-6)
    assert dist(p0_e, p1_s) == pytest.approx(gap_d, abs=1e-6)


def test_rule_1_input_order_decides_who_yields():
    """Same pair, swapped order -- now the other one is the one that is cut."""
    out = shatter([L2, L1], 6.0)

    assert len(out[0]) == 1, "L2 is now the lower index, so it stays whole"
    assert len(out[1]) == 2, "L1 now yields"
    assert out[0][0][0][:2] == pytest.approx((25.0, 94.0), abs=TOL)


def test_rule_2_a_segment_crossed_several_times_is_cut_at_all_of_them():
    spine = seg(0.0, 0.0, 100.0, 0.0)
    ribs = [seg(x, -10.0, x, 10.0) for x in (20.0, 50.0, 80.0)]

    out = shatter(ribs + [spine], 6.0)

    assert [len(p) for p in out[:3]] == [1, 1, 1], "the ribs come first, so none is cut"
    assert len(out[3]) == 4, "three crossings leave four pieces"

    for piece in out[3]:
        assert piece[0][0] < piece[1][0], "pieces run along the direction of their own segment"


def test_rule_3_a_segment_the_gap_swallows_whole_disappears():
    """A 4-long segment crossed at its midpoint, with a gap of 6 -- the
    gap is longer than the segment, so nothing of it survives."""
    out = shatter([seg(-50.0, 0.0, 50.0, 0.0), seg(0.0, -2.0, 0.0, 2.0)], 6.0)

    assert out[1] == [], "swallowed whole"
    assert len(out) == 2, "and still occupies its slot"


# --- contact, not just crossing ---------------------------------------------

SPINE = seg(0.0, 0.0, 100.0, 0.0)


@pytest.mark.parametrize("name, branch", [
    ("endpoint exactly on the spine", seg(50.0, 0.0, 50.0, 20.0)),
    ("endpoint a hair short",         seg(50.0, -1e-6, 50.0, 20.0)),
    ("endpoint a hair past",          seg(50.0, 1e-6, 50.0, 20.0)),
    ("two curves sharing an end",     seg(100.0, 0.0, 150.0, 50.0)),
])
def test_a_t_junction_is_a_contact_and_gets_a_gap(name, branch):
    """The branch ends at the junction, so it yields by retracting its tip --
    one piece, not two, and the same gap_d/2 of clearance a crossing leaves."""
    out = shatter([SPINE, branch], 6.0, 0.01)

    assert len(out[0]) == 1, "the spine is index 0 and keeps its whole length"
    assert len(out[1]) == 1, "the branch ends at the contact, so it loses its tip"
    assert dist(out[1][0][0], branch[0][:2]) == pytest.approx(3.0, abs=1e-6), name


def test_a_clear_miss_is_still_a_miss():
    out = shatter([SPINE, seg(50.0, 5.0, 50.0, 20.0)], 6.0, 0.01)
    assert [len(p) for p in out] == [1, 1]
    assert out[1][0][0][:2] == pytest.approx((50.0, 5.0), abs=TOL)


def test_touch_tolerance_is_a_distance_not_a_cross_product():
    """Piece counts cannot tell a touch from a near crossing here -- both cut,
    and the stub is swallowed either way. Where the survivor STARTS can: a touch
    centres the gap on the endpoint, a crossing centres it on the spine."""
    # 0.005 out, inside a 0.01 tolerance -> contact is the endpoint itself, so
    # the survivor starts gap_d/2 along the branch from there.
    near = seg(50.0, -0.005, 50.0, 20.0)
    y_touch = shatter([SPINE, near], 6.0, 0.01)[1][0][0][1]
    assert y_touch == pytest.approx(-0.005 + 3.0, abs=1e-9)

    # Same physical offset, spine 100x longer. A raw cross-product epsilon grows
    # with segment length and would flip this verdict; a distance one does not.
    long_spine = seg(0.0, 0.0, 10000.0, 0.0)
    y_long = shatter([long_spine, seg(5000.0, -0.005, 5000.0, 20.0)], 6.0, 0.01)[1][0][0][1]
    assert y_long == pytest.approx(y_touch, abs=1e-9)

    # 0.05 out, beyond tolerance -> read as a crossing, so the gap centres on
    # the spine and the survivor starts a clean gap_d/2 above it.
    far = seg(50.0, -0.05, 50.0, 20.0)
    y_cross = shatter([SPINE, far], 6.0, 0.01)[1][0][0][1]
    assert y_cross == pytest.approx(3.0, abs=1e-9)


# --- index preservation -----------------------------------------------------

@pytest.mark.parametrize("segments, gap_d", [
    ([L1, L2], 6.0),
    ([L1, L2], 0.0),
    ([seg(-50.0, 0.0, 50.0, 0.0), seg(0.0, -2.0, 0.0, 2.0)], 6.0),
    ([seg(0.0, 0.0, 1.0, 1.0)], 6.0),
    ([], 6.0),
])
def test_output_length_always_equals_input_length(segments, gap_d):
    assert len(shatter(segments, gap_d)) == len(segments)


# --- edge cases -------------------------------------------------------------

def test_segments_that_do_not_cross_come_back_untouched():
    a = seg(0.0, 0.0, 10.0, 0.0)
    b = seg(0.0, 5.0, 10.0, 5.0)
    out = shatter([a, b], 6.0)

    assert [len(p) for p in out] == [1, 1]
    assert out[0][0][0][:2] == pytest.approx(a[0][:2], abs=TOL)
    assert out[1][0][1][:2] == pytest.approx(b[1][:2], abs=TOL)


def test_overlapping_gaps_merge_instead_of_leaving_a_crumb_between_them():
    """Two crossings 2 apart with a gap of 6 each: the removals overlap, so the
    result is one wide gap and two pieces -- not three, and no sliver."""
    out = shatter([seg(50.0, -10.0, 50.0, 10.0),
                   seg(52.0, -10.0, 52.0, 10.0),
                   seg(0.0, 0.0, 100.0, 0.0)], 6.0)

    assert len(out[2]) == 2
    (_, left_end), (right_start, _) = out[2]
    assert left_end[0] == pytest.approx(47.0, abs=1e-6)
    assert right_start[0] == pytest.approx(55.0, abs=1e-6)


def test_collinear_overlap_is_left_alone():
    """No single crossing point, so there is nothing to open a gap around.
    Deliberately out of scope for this stage."""
    out = shatter([seg(0.0, 0.0, 10.0, 0.0), seg(5.0, 0.0, 15.0, 0.0)], 6.0)

    assert [len(p) for p in out] == [1, 1]


def test_zero_length_segment_passes_through_rather_than_vanishing():
    out = shatter([seg(-50.0, 0.0, 50.0, 0.0), seg(5.0, 5.0, 5.0, 5.0)], 6.0)

    assert len(out[1]) == 1
    assert out[1][0][0][:2] == pytest.approx((5.0, 5.0), abs=TOL)


def test_z_is_carried_along_the_segment():
    """The geometry is 2D, but a cut point keeps the height it would have had."""
    out = shatter([seg(63.0, 110.0, 63.0, 3.0), seg(25.0, 94.0, 114.0, 36.0, 0.0, 10.0)], 6.0)

    (_, p0_e), (p1_s, _) = out[1]
    assert 0.0 < p0_e[2] < p1_s[2] < 10.0


def test_gap_d_zero_cuts_without_removing_anything():
    out = shatter([L1, L2], 0.0)

    assert len(out[1]) == 2
    (_, p0_e), (p1_s, _) = out[1]
    assert dist(p0_e, p1_s) == pytest.approx(0.0, abs=1e-9)


# --- the wrapper ------------------------------------------------------------
# Only what runs before load_dll(); the rest needs the DLL (see module docstring).

def test_wrapper_rejects_a_negative_gap():
    with pytest.raises(ValueError):
        shatter_at_crossings_native([object()], -1.0)


def test_wrapper_short_circuits_on_empty_input():
    assert shatter_at_crossings_native([], 6.0) == []


# --- the retry path, through the real DLL -----------------------------------
# Everything above runs on the mirror, which is why it stayed green while the
# wrapper was passing 7 arguments to a 9-argument function and corrupting the
# heap. This one goes through ctypes on purpose, and only trips when the output
# exceeds the wrapper's first guess of 2n so the regrow-and-call-again branch
# actually runs.

class _Pt3:
    __slots__ = ("X", "Y", "Z")

    def __init__(self, x, y, z=0.0):
        self.X, self.Y, self.Z = x, y, z


class _Line:
    __slots__ = ("PointAtStart", "PointAtEnd")

    def __init__(self, x0, y0, x1, y1):
        self.PointAtStart, self.PointAtEnd = _Pt3(x0, y0), _Pt3(x1, y1)


def test_native_retry_path_when_the_first_buffer_is_too_small():
    import random

    # gap=0 is what makes the output explode: nothing is removed, so every
    # contact adds a whole piece instead of merging into a neighbouring gap.
    # That is the only reliable way to clear the wrapper's first guess -- see
    # the sizing note in geometry_utils.
    rnd = random.Random(7)
    n = 40
    lines = [_Line(rnd.uniform(0, 60), rnd.uniform(0, 60),
                   rnd.uniform(0, 60), rnd.uniform(0, 60)) for _ in range(n)]

    out = shatter_at_crossings_native(lines, 0.0,
                                      segment_owner=list(range(n)), test_self=True)

    total = sum(len(g) for g in out)
    assert len(out) == n
    assert total > SHATTER_FIRST_GUESS * n, "this case must overflow the first guess or it tests nothing"
    for group in out:
        for s, e in group:
            assert len(s) == 3 and len(e) == 3
            assert s != e


def test_declared_argtypes_reject_a_wrong_arity_call():
    """What the missing declaration used to cost: without argtypes ctypes passes
    whatever it is given and the native side writes through the wrong pointer."""
    import ctypes as ct

    from geomseq_core import native_bridge

    lib = native_bridge.load_dll()

    # Too few arguments is a TypeError; ArgumentError is for the wrong type.
    # It is the arity check that matters here -- that is the one that was missing.
    with pytest.raises(TypeError):
        lib.shatter_at_crossings(None, 0, ct.c_double(0.0))
