"""resample_on_lattice: lattice points on rows, joins in the columns of the row
they arrive on, free runs spread evenly along their length, short steps
dropped."""

import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))
sys.path.insert(0, _HERE)

from geomseq_core.coverage_path import FILL, ONE_WAY_SEAMS, TRAVEL, ZIGZAG, decompose, fill_cells, plan
from geomseq_core.resample_on_lattice import resample_on_lattice


def _close(a, b, tol=1e-9):
    return math.dist(a, b) <= tol


def _on_columns(x, offset, step):
    return abs((x - offset) / step - round((x - offset) / step)) < 1e-9


def _lat(points, **kw):
    return resample_on_lattice([(points, False)], **kw)


def test_row_points_sit_on_the_staggered_lattice():
    for r in range(5):
        y = 0.5 * r
        pts = _lat([(0.3, y), (17.9, y)], spacing=0.5, step=4.0, min_step=0)
        inner = [p[0] for p in pts[1:-1]]
        assert inner and all(_on_columns(x, (r % 4) * 1.0, 4.0) for x in inner)
        assert _close(pts[0], (0.3, y)) and _close(pts[-1], (17.9, y))


def test_rows_walked_backwards_take_the_same_points():
    a = _lat([(0.3, 1.0), (17.9, 1.0)], spacing=0.5, step=4.0, min_step=0)
    b = _lat([(17.9, 1.0), (0.3, 1.0)], spacing=0.5, step=4.0, min_step=0)
    assert a == list(reversed(b))


def test_a_line_halfway_between_rows_takes_the_half_step():
    pts = _lat([(0.0, 1.25), (12.0, 1.25)], spacing=0.5, step=4.0, min_step=0)
    assert [round(p[0], 9) for p in pts[1:-1]] == [2.5, 6.5, 10.5]


def test_a_return_join_stands_in_the_columns_of_the_row_it_arrives_on():
    path = [(0.3, 0.0), (17.9, 0.0), (0.4, 0.5), (17.6, 0.5)]
    pts = _lat(path, spacing=0.5, step=4.0, min_step=0)
    k = pts.index((17.9, 0.0))
    j = next(i for i in range(k + 1, len(pts)) if abs(pts[i][1] - 0.5) < 1e-12)
    on_return = pts[k + 1:j]
    assert len(on_return) >= 3
    for x, y in on_return:
        assert 0.0 < y < 0.5 and _on_columns(x, 1.0, 4.0)


def test_a_steep_segment_in_a_lattice_run_is_split_evenly():
    pts = _lat([(0.0, 0.0), (1.0, 9.0)], spacing=0.5, step=4.0)
    steps = [math.dist(p, q) for p, q in zip(pts, pts[1:])]
    assert len(steps) == 3 and max(steps) - min(steps) < 1e-9


def test_a_free_run_ignores_its_vertices_and_spreads_points_by_length():
    # A midline-like travel: a vertex every 0.5, 6 long in all.
    travel = [(0.0, 0.0)] + [(0.0, 0.5 * k) for k in range(1, 13)]
    pts = resample_on_lattice([(travel, True)], 0.5, 4.0, free_step=2.0)
    assert [round(p[1], 9) for p in pts] == [0.0, 2.0, 4.0, 6.0]


def test_free_points_lie_on_the_run():
    travel = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0)]
    pts = resample_on_lattice([(travel, True)], 0.5, 4.0, free_step=1.0, min_step=0)
    assert len(pts) == 7
    for x, y in pts:
        assert (abs(y) < 1e-9 and -1e-9 <= x <= 3.0 + 1e-9) or (abs(x - 3.0) < 1e-9 and -1e-9 <= y <= 3.0 + 1e-9)


def test_a_half_spacing_climb_is_dropped_and_a_full_one_kept():
    seam = [(10.0, 0.75), (0.2, 0.75), (0.2, 1.0), (9.0, 1.0)]
    pts = _lat(seam, spacing=0.5, step=4.0)
    assert not any(_close(p, (0.2, 1.0)) for p in pts)
    zig = [(10.0, 0.5), (0.2, 0.5), (0.2, 1.0), (9.0, 1.0)]
    pts = _lat(zig, spacing=0.5, step=4.0)
    assert any(_close(p, (0.2, 1.0)) for p in pts)


def test_skip_row_ends_drops_row_ends_but_not_the_path_end():
    path = [(0.3, 0.0), (17.9, 0.0), (17.5, 0.5), (0.5, 0.5)]
    pts = _lat(path, spacing=0.5, step=4.0, skip_row_ends=True, min_step=0)
    assert not any(_close(p, (17.9, 0.0)) for p in pts)
    assert _close(pts[-1], (0.5, 0.5)) and _close(pts[0], (0.3, 0.0))


def test_angle_turns_the_lattice_with_the_path():
    path = [(0.3, 0.0), (17.9, 0.0), (17.5, 0.5), (0.5, 0.5)]
    flat = _lat(path, spacing=0.5, step=4.0, phase=0.1)
    t = 0.7
    c, s = math.cos(t), math.sin(t)
    turn = lambda p: (c * p[0] - s * p[1], s * p[0] + c * p[1])
    turned = _lat([turn(p) for p in path], spacing=0.5, step=4.0, angle=t, phase=0.1)
    assert len(flat) == len(turned)
    for p, q in zip(flat, turned):
        assert _close(turn(p), q)


@pytest.mark.parametrize("link", [ZIGZAG, ONE_WAY_SEAMS])
def test_a_plan_resamples_end_to_end(link):
    rings = [[(0.0, 0.0), (12.0, 0.0), (12.0, 8.0), (0.0, 8.0)], [(4.0, 3.0), (4.0, 5.0), (8.0, 5.0), (8.0, 3.0)]]
    dec = decompose(rings, angle=0.3)
    p = plan(dec, fill_cells(dec, 0.5, phase=0.1, link=link))
    runs = [(pc.points, pc.kind == TRAVEL) for pc in p.pieces]
    pts = resample_on_lattice(runs, 0.5, 3.0, angle=0.3, phase=0.1, free_step=2.0)
    assert _close(pts[0], p.path[0]) and _close(pts[-1], p.path[-1])
    steps = [math.dist(a, b) for a, b in zip(pts, pts[1:])]
    assert min(steps) >= 0.5 - 1e-9
    assert max(steps) <= 3.0 + 0.5 + 1e-9


@pytest.mark.parametrize("kwargs,message", [
    ({"spacing": 0.0, "step": 1.0}, "spacing"),
    ({"spacing": 1.0, "step": 0.0}, "step"),
    ({"spacing": 1.0, "step": 1.0, "stagger": 0.0}, "stagger"),
    ({"spacing": 1.0, "step": 1.0, "free_step": 0.0}, "free_step"),
    ({"spacing": 1.0, "step": 1.0, "min_step": -1.0}, "min_step"),
])
def test_bad_arguments_are_refused(kwargs, message):
    with pytest.raises(ValueError, match=message):
        resample_on_lattice([([(0.0, 0.0), (1.0, 0.0)], False)], **kwargs)


# --- the C++ port --------------------------------------------------------------

import random

from geomseq_core.coverage_path import ONE_WAY
from geomseq_core.geometry_utils import resample_on_lattice_native


def _plan_runs(link, angle):
    rings = [[(0.0, 0.0), (12.0, 0.0), (12.0, 8.0), (6.0, 5.5), (0.0, 8.0)],
             [(3.0, 2.0), (3.0, 3.5), (5.0, 3.5), (5.0, 2.0)]]
    dec = decompose(rings, angle=angle)
    p = plan(dec, fill_cells(dec, 0.35, phase=0.1, link=link))
    return [(pc.points, pc.kind == TRAVEL) for pc in p.pieces]


@pytest.mark.parametrize("link", [ZIGZAG, ONE_WAY, ONE_WAY_SEAMS])
@pytest.mark.parametrize("angle", [0.0, 0.6])
@pytest.mark.parametrize("kwargs", [
    {},
    {"free_step": 2.0, "min_step": 0.6},
    {"stagger": 2.5, "skip_row_ends": True, "min_step": 0.0},
    {"step": 1.3, "free_step": 0.7},
], ids=["defaults", "free-and-min", "stagger-skip", "short-steps"])
def test_native_matches_python_point_for_point(link, angle, kwargs):
    runs = _plan_runs(link, angle)
    args = dict(spacing=0.35, step=3.0, angle=angle, phase=0.1)
    args.update(kwargs)
    assert resample_on_lattice_native(runs, **args) == resample_on_lattice(runs, **args)


def test_native_matches_python_on_random_runs():
    rng = random.Random(7)
    for _ in range(200):
        runs, at = [], (rng.uniform(-5, 5), rng.uniform(-5, 5))
        for _ in range(rng.randint(1, 4)):
            pts = [at]
            for _ in range(rng.randint(1, 6)):
                horizontal = rng.random() < 0.5
                pts.append((pts[-1][0] + rng.uniform(-6, 6),
                            pts[-1][1] + (0.0 if horizontal else rng.uniform(-1, 1))))
            runs.append((pts, rng.random() < 0.4))
            at = pts[-1]
        args = dict(spacing=rng.uniform(0.1, 1.0), step=rng.uniform(0.5, 4.0),
                    angle=rng.uniform(-3, 3), phase=rng.uniform(-1, 1),
                    stagger=rng.choice([1.0, 3.0, 4.0, 2.5]), free_step=rng.uniform(0.3, 3.0),
                    skip_row_ends=rng.random() < 0.3, min_step=rng.choice([None, 0.0, 0.4]))
        assert resample_on_lattice_native(runs, **args) == resample_on_lattice(runs, **args)


@pytest.mark.parametrize("kwargs,message", [
    ({"spacing": 0.0, "step": 1.0}, "spacing"),
    ({"spacing": 1.0, "step": 0.0}, "step"),
    ({"spacing": 1.0, "step": 1.0, "stagger": 0.0}, "stagger"),
    ({"spacing": 1.0, "step": 1.0, "free_step": 0.0}, "free_step"),
    ({"spacing": 1.0, "step": 1.0, "min_step": -1.0}, "min_step"),
])
def test_native_refuses_what_python_refuses(kwargs, message):
    with pytest.raises(ValueError, match=message):
        resample_on_lattice_native([([(0.0, 0.0), (1.0, 0.0)], False)], **kwargs)


def test_native_handles_no_runs_and_a_point():
    assert resample_on_lattice_native([], 0.5, 4.0) == resample_on_lattice([], 0.5, 4.0) == []
    one = [([(1.0, 2.0)], False)]
    assert resample_on_lattice_native(one, 0.5, 4.0) == resample_on_lattice(one, 0.5, 4.0)
