"""Tests for coverage_path.decompose; cases are inline, plain floats.

The invariants below are what the later stages rely on: the cells tile the
region (areas add up, nothing overlaps), every cell is monotone along the
sweep, and portals join exactly the cells that share an open interval of
scanline. Shapes are chosen to hit the slab machinery where it can go wrong:
a vertex exactly on a slab boundary, a horizontal edge, a hole, a pinch.
"""

import itertools
import math
import os
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.coverage_path import (
    FORWARD,
    MIRRORED,
    MIRRORED_REVERSE,
    REVERSE,
    FILL,
    TRAVEL,
    _Router,
    _segment_inside_union,
    _walks_for,
    decompose,
    fill_cells,
    plan,
    polygon_area,
    region_area,
    shortest_travel,
    travel,
    visible_length,
)

SQUARE = [(0.0, 0.0), (4.0, 0.0), (4.0, 4.0), (0.0, 4.0)]
HOLE = [(1.0, 1.0), (3.0, 1.0), (3.0, 3.0), (1.0, 3.0)]

# A U, open at the top: one bar and two arms.
#
#   (0,3) +--+     +--+ (3,3)
#         |  |     |  |
#   (0,1) |  +-----+  |
#         |  (1,1) (2,1)
#   (0,0) +-----------+ (3,0)
U_SHAPE = [(0.0, 0.0), (3.0, 0.0), (3.0, 3.0), (2.0, 3.0),
           (2.0, 1.0), (1.0, 1.0), (1.0, 3.0), (0.0, 3.0)]

# Two triangles meeting at one point: a pinch, not a portal.
HOURGLASS = [(0.0, 0.0), (2.0, 0.0), (1.0, 1.0), (2.0, 2.0), (0.0, 2.0), (1.0, 1.0)]

DIAMOND = [(1.0, 0.0), (2.0, 1.0), (1.0, 2.0), (0.0, 1.0)]


def _rotated(ring, angle):
    c, s = math.cos(angle), math.sin(angle)
    return [(c * x - s * y, s * x + c * y) for x, y in ring]


def _cells_tile_region(dec, rings):
    """Cell areas add up to the region's, and every cell is CCW."""
    total = 0.0
    for cell in dec.cells:
        a = polygon_area(cell.outline)
        assert a > 0.0, "cell outline is not counter-clockwise"
        total += a
    assert total == pytest.approx(region_area(rings))


def _is_monotone(cell):
    """Any horizontal line in the sweep frame meets the cell in one interval:
    rows ascend in y and keep x_left < x_right."""
    rows = cell.rows
    for k in range(1, len(rows)):
        assert rows[k][0] >= rows[k - 1][0]
    for _, xl, xr in rows:
        assert xl <= xr


def _portals_are_symmetric(dec):
    for i in range(len(dec.cells)):
        for j in dec.neighbours(i):
            assert i in dec.neighbours(j)


# --- plumbing ----------------------------------------------------------------

def test_negative_tol_is_refused():
    with pytest.raises(ValueError):
        decompose([SQUARE], tol=-1.0)


def test_degenerate_ring_is_refused():
    with pytest.raises(ValueError):
        decompose([[(0.0, 0.0), (1.0, 0.0)]])


def test_closing_vertex_may_be_repeated():
    closed = SQUARE + [SQUARE[0]]
    assert len(decompose([closed]).cells) == 1


def test_ring_orientation_does_not_matter():
    cw = list(reversed(SQUARE))
    dec = decompose([cw, HOLE])
    _cells_tile_region(dec, [cw, HOLE])
    assert len(dec.cells) == 4


# --- the cut -----------------------------------------------------------------

def test_convex_region_is_one_cell():
    dec = decompose([SQUARE])
    assert len(dec.cells) == 1
    assert dec.portals == []
    _cells_tile_region(dec, [SQUARE])


def test_vertex_on_every_slab_boundary():
    """A diamond puts a vertex on every level, and the top and bottom slabs
    taper to a point. Still one cell."""
    dec = decompose([DIAMOND])
    assert len(dec.cells) == 1
    _cells_tile_region(dec, [DIAMOND])


def test_u_shape_is_bar_plus_two_arms():
    dec = decompose([U_SHAPE])
    assert len(dec.cells) == 3
    assert len(dec.portals) == 2
    _cells_tile_region(dec, [U_SHAPE])
    _portals_are_symmetric(dec)
    for cell in dec.cells:
        _is_monotone(cell)
    # The bar is the cell both arms open onto.
    degrees = sorted(len(dec.neighbours(i)) for i in range(3))
    assert degrees == [1, 1, 2]


def test_hole_makes_four_cells_in_a_ring():
    dec = decompose([SQUARE, HOLE])
    assert len(dec.cells) == 4
    assert len(dec.portals) == 4
    _cells_tile_region(dec, [SQUARE, HOLE])
    _portals_are_symmetric(dec)
    # Every cell has exactly two neighbours: the ring is a cycle.
    assert all(len(dec.neighbours(i)) == 2 for i in range(4))


def test_pinch_is_not_a_portal():
    dec = decompose([HOURGLASS])
    assert len(dec.cells) == 2
    assert dec.portals == []
    _cells_tile_region(dec, [HOURGLASS])


def test_portal_is_the_shared_interval():
    """The U's left arm meets the bar across y = 1, x in (0, 1)."""
    dec = decompose([U_SHAPE])
    spans = sorted((min(p.p0[0], p.p1[0]), max(p.p0[0], p.p1[0]), p.p0[1])
                   for p in dec.portals)
    assert spans[0] == pytest.approx((0.0, 1.0, 1.0))
    assert spans[1] == pytest.approx((2.0, 3.0, 1.0))


def test_sweep_angle_follows_the_shape():
    """The U rotated by 90 degrees, swept at 90 degrees, cuts as the upright
    U does: three cells. Swept at 0 it is a C lying on its side, which every
    horizontal line meets once, so it is a single cell."""
    angle = math.pi / 2
    sideways = _rotated(U_SHAPE, angle)

    along = decompose([sideways], angle=angle)
    assert len(along.cells) == 3
    _cells_tile_region(along, [sideways])
    for cell in along.cells:
        _is_monotone(cell)
    # Outlines come back in the input frame: the bar is now at x in [0, 1]
    # after rotation, i.e. the cell whose outline has the smallest x extent.
    xs = [max(x for x, _ in c.outline) - min(x for x, _ in c.outline) for c in along.cells]
    assert min(xs) == pytest.approx(1.0)

    across = decompose([sideways], angle=0.0)
    _cells_tile_region(across, [sideways])
    assert len(across.cells) == 1
    _is_monotone(across.cells[0])


def test_horizontal_edge_inside_a_cell_keeps_it_one_cell():
    """A step on the right side: a horizontal edge the sweep crosses, with
    one span before and after. The cell steps sideways, it does not split.

      (0,2) +-----+ (2,2)
            |     |
      (0,1) |     +---+ (3,1)
            |         |
      (0,0) +---------+ (3,0)
    """
    step = [(0.0, 0.0), (3.0, 0.0), (3.0, 1.0), (2.0, 1.0), (2.0, 2.0), (0.0, 2.0)]
    dec = decompose([step])
    assert len(dec.cells) == 1
    _cells_tile_region(dec, [step])
    _is_monotone(dec.cells[0])
    # The outline must carry the step, so the horizontal edge is traced.
    assert (3.0, 1.0) in [(round(x, 9), round(y, 9)) for x, y in dec.cells[0].outline]
    assert (2.0, 1.0) in [(round(x, 9), round(y, 9)) for x, y in dec.cells[0].outline]


def test_rings_sharing_an_edge_are_one_region():
    """Two squares sharing an edge: the shared edge cancels under even-odd
    and the two spans on every scanline touch, so they are one 8x4 cell.
    Two cells side by side would have no portal between them, and a travel
    could never cross."""
    left = SQUARE
    right = [(4.0, 0.0), (8.0, 0.0), (8.0, 4.0), (4.0, 4.0)]
    dec = decompose([left, right])
    _cells_tile_region(dec, [left, right])
    assert len(dec.cells) == 1


# --- the fill ----------------------------------------------------------------

def _inside_cell(cell, pt, angle=0.0, tol=1e-9):
    """pt (input frame) lies in the cell: on a scanline through its rows,
    within that scanline's interval."""
    c, s = math.cos(-angle), math.sin(-angle)
    x, y = c * pt[0] - s * pt[1], s * pt[0] + c * pt[1]
    rows = cell.rows
    if not rows[0][0] - tol <= y <= rows[-1][0] + tol:
        return False
    for k in range(len(rows) - 1):
        y0, l0, r0 = rows[k]
        y1, l1, r1 = rows[k + 1]
        if y0 - tol <= y <= y1 + tol:
            if y1 == y0:
                lo, hi = min(l0, l1), max(r0, r1)
            else:
                t = min(max((y - y0) / (y1 - y0), 0.0), 1.0)
                lo, hi = l0 + t * (l1 - l0), r0 + t * (r1 - r0)
            if lo - tol <= x <= hi + tol:
                return True
    return False


def test_zero_spacing_is_refused():
    with pytest.raises(ValueError):
        fill_cells(decompose([SQUARE]), 0.0)


def test_square_fill_is_a_zigzag_on_the_grid():
    fills = fill_cells(decompose([SQUARE]), 1.0)
    f = fills[0]
    assert f.line_count == 5
    # Bottom-left in, alternating ends, top out.
    assert f.forward[0] == pytest.approx((0.0, 0.0))
    assert f.forward[1] == pytest.approx((4.0, 0.0))
    assert f.forward[2] == pytest.approx((4.0, 1.0))
    assert f.forward[3] == pytest.approx((0.0, 1.0))
    assert f.forward[-1][1] == pytest.approx(4.0)
    assert f.reverse == f.forward[::-1]


@pytest.mark.parametrize("spacing", [1.0, 4.0 / 3.0])
def test_the_four_walks_enter_at_the_four_corners(spacing):
    """Odd (5) and even (4) line counts alike: between them the four walks
    enter the square at all four corners, and each is a zig-zag over the
    same lines."""
    f = fill_cells(decompose([SQUARE]), spacing)[0]
    entries = sorted((round(w[0][0], 9), round(w[0][1], 9)) for w in f.walks)
    assert entries == [(0.0, 0.0), (0.0, 4.0), (4.0, 0.0), (4.0, 4.0)]
    assert f.walks[MIRRORED][0] == pytest.approx((4.0, 0.0))
    assert f.walks[MIRRORED][1] == pytest.approx((0.0, 0.0))
    assert f.walks[REVERSE] == f.walks[FORWARD][::-1]
    assert f.walks[MIRRORED_REVERSE] == f.walks[MIRRORED][::-1]
    for w in f.walks:
        assert len(w) == 2 * f.line_count


def test_fill_stays_inside_its_cell():
    rings = [U_SHAPE]
    dec = decompose(rings)
    for f in fill_cells(dec, 0.3, phase=0.1):
        cell = dec.cells[f.cell]
        for w in f.walks:
            for p in w:
                assert _inside_cell(cell, p)


def test_lines_of_neighbouring_cells_share_the_grid():
    """Across the U's portals, the arms' lines sit on the same y values as
    the bar's would: every line is on the global grid."""
    dec = decompose([U_SHAPE])
    for f in fill_cells(dec, 0.5):
        ys = sorted(set(round(p[1], 9) for p in f.forward))
        for y in ys:
            assert (y / 0.5) == pytest.approx(round(y / 0.5))


def test_a_line_on_a_portal_is_stitched_once():
    """With spacing 0.5 the grid lands on y = 1, the U's portals: the arms
    take that row, the bar stops below it."""
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5)
    bar = _cell_at(dec, (1.5, 0.5))
    rows = [(round(a[1], 9), round(a[0], 9), round(b[0], 9))
            for f in fills for a, b in f.lines]
    assert len(rows) == len(set(rows))
    assert max(a[1] for a, _ in fills[bar].lines) == pytest.approx(0.5)
    for f in fills:
        if f.cell != bar:
            assert min(a[1] for a, _ in f.lines) == pytest.approx(1.0)


def test_connection_follows_a_bent_side():
    """A notch on the right between two lines: the join must go through the
    notch vertices, not cut across the gap.

      (0,2) +-------+ (3,2)
            |       |
      (0,1.5)|   +---+ (3,1.5)
            |   | (2,1.5)
      (0,.5)|   +---+ (3,0.5)
            |       |
      (0,0) +-------+ (3,0)
    """
    notch = [(0.0, 0.0), (3.0, 0.0), (3.0, 0.5), (2.0, 0.5), (2.0, 1.5),
             (3.0, 1.5), (3.0, 2.0), (0.0, 2.0)]
    dec = decompose([notch])
    assert len(dec.cells) == 1
    f = fill_cells(dec, 2.0)[0]
    assert f.line_count == 2
    rounded = [(round(x, 9), round(y, 9)) for x, y in f.forward]
    # (3,0) -> down the notch and back -> (3,2)
    assert rounded == [(0.0, 0.0), (3.0, 0.0), (3.0, 0.5), (2.0, 0.5),
                       (2.0, 1.5), (3.0, 1.5), (3.0, 2.0), (0.0, 2.0)]
    for p in f.forward:
        assert _inside_cell(dec.cells[0], p)


def test_mirrored_walk_follows_a_bent_side_too():
    """The notch reflected to the left side: the mirrored walk climbs that
    side, so it must go through the notch exactly as the forward walk does
    on the unreflected shape."""
    notch = [(0.0, 0.0), (3.0, 0.0), (3.0, 0.5), (2.0, 0.5), (2.0, 1.5),
             (3.0, 1.5), (3.0, 2.0), (0.0, 2.0)]
    reflected = [(3.0 - x, y) for x, y in notch]
    dec = decompose([reflected])
    f = fill_cells(dec, 2.0)[0]
    rounded = [(round(x, 9), round(y, 9)) for x, y in f.walks[MIRRORED]]
    assert rounded == [(3.0, 0.0), (0.0, 0.0), (0.0, 0.5), (1.0, 0.5),
                       (1.0, 1.5), (0.0, 1.5), (0.0, 2.0), (3.0, 2.0)]


def test_thin_cell_gets_one_line_through_its_middle():
    sliver = [(0.0, 0.0), (4.0, 0.0), (4.0, 0.2), (0.0, 0.2)]
    f = fill_cells(decompose([sliver]), 1.0, phase=0.5)[0]
    assert f.line_count == 1
    assert f.forward[0][1] == pytest.approx(0.1)


def _path_length(path):
    return sum(math.dist(a, b) for a, b in zip(path, path[1:]))


def _cell_at(dec, pt):
    """Index of the cell whose interior holds pt; the test shapes keep their
    probe points off every boundary."""
    hits = [i for i, c in enumerate(dec.cells) if _inside_cell(c, pt)]
    assert len(hits) == 1
    return hits[0]


# --- the travel --------------------------------------------------------------

def _fill_ends(dec, fills, from_cell, to_cell):
    return fills[from_cell].forward[-1], fills[to_cell].forward[0]


def _check_path(dec, unstitched, path, start, goal):
    """Ends in place, every segment inside the unstitched cells."""
    assert path[0] == start and path[-1] == goal
    rings = [dec.cells[i].outline for i in unstitched]
    for a, b in zip(path, path[1:]):
        assert _segment_inside_union(a, b, rings, 1e-9)


def test_travel_to_self_is_one_point():
    dec = decompose([SQUARE])
    fills = fill_cells(dec, 1.0)
    """Two points for the caller's polyline, zero length."""
    assert travel(dec, fills, [0], (1.0, 1.0), (1.0, 1.0)) == [(1.0, 1.0), (1.0, 1.0)]


def test_travel_runs_down_the_middle_of_the_arm():
    """Left arm to right arm of the U with the bar unstitched: down the left
    arm's midline (x = 0.5), through the portal, along the bar's midline,
    up the right arm's (x = 2.5). Nothing on a corner, nothing on an edge."""
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    left, right = _cell_at(dec, (0.5, 2.0)), _cell_at(dec, (2.5, 2.0))
    start, goal = _fill_ends(dec, fills, left, right)
    every = list(range(len(dec.cells)))

    path = travel(dec, fills, every, start, goal)
    assert path is not None
    _check_path(dec, every, path, start, goal)
    xs = sorted(set(round(x, 9) for x, _ in path[1:-1]))
    assert xs == [0.5, 1.5, 2.5]
    for p in path[1:-1]:
        assert p not in [(1.0, 1.0), (2.0, 1.0)]


def test_travel_over_a_stitched_bar_shows_only_the_crossing():
    """The bar is already stitched and the arms are joined only through it:
    no trim, the travel crosses the bar, and only that crossing shows."""
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    left, right = _cell_at(dec, (0.5, 2.0)), _cell_at(dec, (2.5, 2.0))
    start, goal = _fill_ends(dec, fills, left, right)
    path = travel(dec, fills, [left, right], start, goal)
    every = list(range(len(dec.cells)))
    _check_path(dec, every, path, start, goal)
    shown = visible_length(path, dec, fills, [left, right])
    assert 0.0 < shown < _path_length(path)
    # Everything above the bar is in the arms, hidden.
    assert shown == pytest.approx(visible_length(
        [p for p in path if p[1] <= 1.0 + 1e-9], dec, fills, [left, right]))


def test_travel_is_hidden_whenever_unstitched_cells_join_the_ends():
    """With the bar unstitched the same travel shows nothing."""
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    left, right = _cell_at(dec, (0.5, 2.0)), _cell_at(dec, (2.5, 2.0))
    start, goal = _fill_ends(dec, fills, left, right)
    every = list(range(len(dec.cells)))
    path = travel(dec, fills, every, start, goal)
    assert visible_length(path, dec, fills, every) == 0.0


def test_hidden_detour_beats_a_shorter_visible_way():
    """Donut, the bottom cell stitched: from the left cell to the right the
    way under the bottom is shorter but shows; the way over the top is
    longer and hidden, and is the one taken."""
    rings = [SQUARE, HOLE]
    dec = decompose(rings)
    fills = fill_cells(dec, 0.3, phase=0.1)
    bottom = _cell_at(dec, (2.0, 0.5))
    left, right = _cell_at(dec, (0.5, 2.0)), _cell_at(dec, (3.5, 2.0))
    start = fills[left].lines[0][0]
    goal = fills[right].lines[0][1]
    unstitched = [i for i in range(len(dec.cells)) if i != bottom]
    path = travel(dec, fills, unstitched, start, goal)
    assert visible_length(path, dec, fills, unstitched) == 0.0
    assert max(y for _, y in path) > 3.0


def test_goal_in_a_stitched_cell_is_still_reached():
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    bar = _cell_at(dec, (1.5, 0.5))
    left = _cell_at(dec, (0.5, 2.0))
    start, goal = _fill_ends(dec, fills, left, bar)
    unstitched = [i for i in range(len(dec.cells)) if i != bar]
    path = travel(dec, fills, unstitched, start, goal)
    _check_path(dec, list(range(len(dec.cells))), path, start, goal)
    assert visible_length(path, dec, fills, unstitched) > 0.0


def test_cells_meeting_at_a_point_are_joined_through_it():
    """The hourglass's two cells share no portal, only the pinch: the
    midline cannot cross, the corner fallback does, through (1, 1)."""
    dec = decompose([HOURGLASS])
    fills = fill_cells(dec, 0.2, phase=0.1)
    lower, upper = _cell_at(dec, (1.0, 0.3)), _cell_at(dec, (1.0, 1.7))
    start, goal = _fill_ends(dec, fills, lower, upper)
    every = list(range(len(dec.cells)))
    path = travel(dec, fills, every, start, goal)
    _check_path(dec, every, path, start, goal)
    assert any(_on_segment_pt((1.0, 1.0), a, b) for a, b in zip(path, path[1:]))


def test_separate_pieces_cannot_be_joined():
    far = [(10.0, 0.0), (11.0, 0.0), (11.0, 1.0), (10.0, 1.0)]
    dec = decompose([SQUARE, far])
    fills = fill_cells(dec, 0.5, phase=0.25)
    every = list(range(len(dec.cells)))
    with pytest.raises(ValueError):
        travel(dec, fills, every, (2.0, 2.0), (10.5, 0.5))


def test_every_pair_of_donut_cells_is_joined_from_fill_end_to_fill_start():
    """What the component asks for: from the end of one cell's fill to the
    start of another's, the cell just stitched still open to leave through.
    No point of any travel sits on the hole's boundary."""
    rings = [SQUARE, HOLE]
    dec = decompose(rings)
    fills = fill_cells(dec, 0.3, phase=0.1)
    n = len(dec.cells)
    for f in range(n):
        for t in range(n):
            if f == t:
                continue
            for fd in range(4):
                for td in range(4):
                    start, goal = fills[f].walks[fd][-1], fills[t].walks[td][0]
                    path = travel(dec, fills, list(range(n)), start, goal)
                    _check_path(dec, list(range(n)), path, start, goal)
                    for p in path[1:-1]:
                        assert not any(_on_segment_pt(p, HOLE[i], HOLE[(i + 1) % 4])
                                       for i in range(4)), (f, fd, t, td)


def _on_segment_pt(p, a, b, tol=1e-9):
    dx, dy = b[0] - a[0], b[1] - a[1]
    if dx == 0.0 and dy == 0.0:
        return math.dist(p, a) <= tol
    t = min(max(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / (dx * dx + dy * dy), 0.0), 1.0)
    return math.hypot(p[0] - a[0] - t * dx, p[1] - a[1] - t * dy) <= tol


def test_travel_is_longer_than_the_shortest_path_but_off_the_corners():
    """The midline travel pays for staying central: longer than the exact
    shortest path, which hugs the hole's corner."""
    dec = decompose([SQUARE, HOLE])
    fills = fill_cells(dec, 0.3, phase=0.1)
    every = list(range(len(dec.cells)))
    start, goal = (0.5, 0.5), (3.5, 3.5)
    tight = shortest_travel(dec, every, start, goal)
    assert tight[1] in [(1.0, 3.0), (3.0, 1.0)]
    mid = travel(dec, fills, every, start, goal)
    assert mid is not None
    assert _path_length(mid) > _path_length(tight)
    assert (1.0, 3.0) not in mid and (3.0, 1.0) not in mid


def test_an_end_off_every_line_attaches_to_the_nearest_midpoint_in_sight():
    """A start that lies on no fill line (here between two lines) still
    gets a path, through the nearest midpoint it can see."""
    dec = decompose([SQUARE])
    fills = fill_cells(dec, 1.0)
    path = travel(dec, fills, [0], (0.3, 0.5), (3.7, 3.5))
    assert path is not None
    _check_path(dec, [0], path, (0.3, 0.5), (3.7, 3.5))


def test_no_doubling_back_to_a_midpoint_when_the_goal_is_nearer():
    """Two fill ends one line apart on the same side: out to the middle and
    back would be 5 long for a 1 long hop. Straight there instead."""
    dec = decompose([SQUARE])
    fills = fill_cells(dec, 1.0)
    path = travel(dec, fills, [0], (0.0, 0.0), (0.0, 1.0))
    assert path == [(0.0, 0.0), (0.0, 1.0)]


def test_shortcut_never_leaves_the_unstitched_cells():
    """Across the U's notch the two arms' fill ends are 1 apart, nearer than
    any midpoint, but the straight hop crosses the notch; the path still
    goes down through the bar."""
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    left, right = _cell_at(dec, (0.5, 2.0)), _cell_at(dec, (2.5, 2.0))
    start = fills[left].lines[-1][1]   # right end of the left arm's top line
    goal = fills[right].lines[-1][0]   # left end of the right arm's top line
    every = list(range(len(dec.cells)))
    path = travel(dec, fills, every, start, goal)
    assert path is not None
    _check_path(dec, every, path, start, goal)
    assert min(y for _, y in path) < 1.0


def test_segment_inside_union_refuses_a_line_through_the_hole():
    """Across the hole the cells have nothing, so a line through its middle
    leaves the union; a line along the ring's band stays inside."""
    dec = decompose([SQUARE, HOLE])
    rings = [c.outline for c in dec.cells]
    assert not _segment_inside_union((0.0, 2.0), (4.0, 2.0), rings, 1e-9)
    assert _segment_inside_union((0.5, 0.5), (3.5, 0.5), rings, 1e-9)


def test_fill_lines_run_along_the_angle():
    angle = math.radians(30.0)
    dec = decompose([SQUARE], angle=angle)
    f = fill_cells(dec, 0.5)[0]
    # The segments along +-angle are the fill lines; the rest follow the
    # cell's sides between them (the tilted square has a corner there).
    along = 0
    for k in range(len(f.forward) - 1):
        (x0, y0), (x1, y1) = f.forward[k], f.forward[k + 1]
        heading = math.atan2(y1 - y0, x1 - x0) % math.pi
        if abs(heading - angle) < 1e-9:
            along += 1
    assert along == f.line_count
    for p in f.forward:
        assert _inside_cell(dec.cells[0], p, angle)


# --- the order ---------------------------------------------------------------

# A comb: a bar with three prongs up. Stitching the bar between two prongs
# leaves the last prong reachable only over it, so the order matters.
COMB = [(0.0, 0.0), (5.0, 0.0), (5.0, 3.0), (4.0, 3.0), (4.0, 1.0), (3.0, 1.0),
        (3.0, 3.0), (2.0, 3.0), (2.0, 1.0), (1.0, 1.0), (1.0, 3.0), (0.0, 3.0)]


def _check_plan(dec, fills, p):
    """Every cell filled once by one of its walks; each piece starts where
    the last ended; no two travels in a row; every travel inside the
    region; path is the pieces joined."""
    filled = [piece.cell for piece in p.pieces if piece.kind == FILL]
    assert sorted(filled) == list(range(len(dec.cells)))
    assert [(piece.cell, piece.direction) for piece in p.pieces if piece.kind == FILL] == p.order
    for piece in p.pieces:
        if piece.kind == FILL:
            assert piece.points == list(fills[piece.cell].walks[piece.direction])
    for a, b in zip(p.pieces, p.pieces[1:]):
        assert a.points[-1] == pytest.approx(b.points[0])
        assert not (a.kind == TRAVEL and b.kind == TRAVEL)
    every = list(range(len(dec.cells)))
    corners = [v for r in dec.rings for v in r]
    for piece in p.pieces:
        if piece.kind == TRAVEL:
            _check_path(dec, every, piece.points, piece.points[0], piece.points[-1])
            # No travel runs through a corner of the outline.
            for a, b in zip(piece.points, piece.points[1:]):
                for v in corners:
                    if math.dist(v, a) > 1e-9 and math.dist(v, b) > 1e-9:
                        assert not _on_segment_pt(v, a, b), (v, a, b)
    assert _path_length(p.path) == pytest.approx(p.fill_length + p.travel_length)


def _order_cost(router, fills, cells, walks, start=None):
    """The cost of one order and its walks, straight from the definition:
    each travel's (visible, length), hidden by the cells still to come."""
    total = (0.0, 0.0)
    pos = start
    for i, (c, d) in enumerate(zip(cells, walks)):
        if pos is not None:
            step = router.costs(pos, (fills[c].walks[d][0],), frozenset(cells[i:]))[0]
            total = (total[0] + step[0], total[1] + step[1])
        pos = fills[c].walks[d][-1]
    return total


def _exhaustive(router, fills, n, start=None):
    """The best cost over every order and every walk of every cell."""
    return min(_order_cost(router, fills, list(cells), list(walks), start)
               for cells in itertools.permutations(range(n))
               for walks in itertools.product(range(4), repeat=n))


def _cost_close(a, b):
    return a[0] == pytest.approx(b[0], abs=1e-9) and a[1] == pytest.approx(b[1], abs=1e-9)


def test_best_walks_for_an_order_are_exact():
    """For every order of the comb's cells, the walks chosen are as good as
    the best of all 4^n."""
    dec = decompose([COMB])
    fills = fill_cells(dec, 0.3, phase=0.1)
    router = _Router(dec, fills, 1e-9)
    n = len(dec.cells)
    for cells in itertools.permutations(range(n)):
        cells = list(cells)
        cost, walks = _walks_for(router, fills, cells, None)
        best = min(_order_cost(router, fills, cells, list(w))
                   for w in itertools.product(range(4), repeat=n))
        assert _cost_close(cost, best)
        assert _cost_close(_order_cost(router, fills, cells, walks), best)


@pytest.mark.parametrize("rings", [[U_SHAPE], [SQUARE, HOLE], [COMB]],
                         ids=["u", "donut", "comb"])
def test_plan_reaches_the_exhaustive_optimum(rings):
    """Small enough to try every order and every walk: the plan's order and
    walks cost exactly the best of them."""
    dec = decompose(rings)
    fills = fill_cells(dec, 0.3, phase=0.1)
    router = _Router(dec, fills, 1e-9)
    p = plan(dec, fills)
    _check_plan(dec, fills, p)
    cells = [c for c, _ in p.order]
    walks = [d for _, d in p.order]
    assert _cost_close(_order_cost(router, fills, cells, walks),
                       _exhaustive(router, fills, len(dec.cells)))


@pytest.mark.parametrize("rings", [[SQUARE], [U_SHAPE], [COMB]],
                         ids=["square", "u", "comb"])
def test_plan_hides_every_travel_when_an_order_can(rings):
    """Each of these has an order whose every travel runs under cells still
    to come, the cell just left counted as stitched; the plan finds one."""
    dec = decompose(rings)
    fills = fill_cells(dec, 0.3, phase=0.1)
    p = plan(dec, fills)
    _check_plan(dec, fills, p)
    assert p.visible_length == 0.0


def test_travel_back_across_the_cell_just_finished_shows():
    """The left arm stitched bottom to top ends at its top, and the way to
    the bar is back down the whole arm, over fresh rows: visible. Walked
    top to bottom, it ends by the portal and only the step across shows."""
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    bar = _cell_at(dec, (1.5, 0.5))
    left, right = _cell_at(dec, (0.5, 2.0)), _cell_at(dec, (2.5, 2.0))
    goal = fills[bar].walks[FORWARD][0]
    unstitched = [bar, right]
    up = travel(dec, fills, unstitched, fills[left].walks[FORWARD][-1], goal)
    down = travel(dec, fills, unstitched, fills[left].walks[REVERSE][-1], goal)
    assert visible_length(up, dec, fills, unstitched) > 1.5
    assert visible_length(down, dec, fills, unstitched) < 0.6


def test_plan_visible_is_what_its_travels_show():
    """Each travel's visible length, recomputed from scratch against the
    cells not yet stitched."""
    dec = decompose([COMB])
    fills = fill_cells(dec, 0.3, phase=0.1)
    p = plan(dec, fills, start=(4.5, 2.9))
    done = set()
    for piece in p.pieces:
        if piece.kind == FILL:
            done.add(piece.cell)
        else:
            hidden = [c for c in range(len(dec.cells)) if c not in done]
            assert piece.visible == pytest.approx(visible_length(piece.points, dec, fills, hidden))


def test_plan_from_a_given_start_travels_from_it_first():
    dec = decompose([U_SHAPE])
    fills = fill_cells(dec, 0.5, phase=0.25)
    p = plan(dec, fills, start=(1.5, 0.1))
    _check_plan(dec, fills, p)
    assert p.pieces[0].kind == TRAVEL and p.pieces[0].points[0] == (1.5, 0.1)


def test_plan_crosses_a_pinch():
    dec = decompose([HOURGLASS])
    fills = fill_cells(dec, 0.2, phase=0.1)
    p = plan(dec, fills)
    _check_plan(dec, fills, p)


@pytest.mark.parametrize("rings", [[SQUARE, HOLE], [COMB], [U_SHAPE]],
                         ids=["donut", "comb", "u"])
def test_folding_the_midline_keeps_every_cost(rings):
    """Searches run over the graph with each cell's run of line midpoints
    folded into one link. Against the unfolded graph, every route between
    every pair of walk ends, under every stitched set, costs the same."""
    dec = decompose(rings)
    fills = fill_cells(dec, 0.3, phase=0.1)
    folded = _Router(dec, fills, 1e-9)
    plain = _Router(dec, fills, 1e-9)
    plain.cadj = [[(v, length, pieces, ()) for v, length, pieces in a] for a in plain.adj]
    plain.folded = {}
    assert sum(map(len, folded.cadj)) < sum(map(len, plain.cadj))
    n = len(dec.cells)
    ends = [w[i] for f in fills for w in f.walks for i in (0, -1)]
    goals = tuple(dict.fromkeys(ends))
    for hidden in itertools.chain.from_iterable(
            itertools.combinations(range(n), k) for k in range(n + 1)):
        hidden = frozenset(hidden)
        for start in goals:
            a = folded.route(start, list(goals), hidden)
            b = plain.route(start, list(goals), hidden)
            for (ca, pa), (cb, pb) in zip(a, b):
                assert _cost_close(ca, cb)
                assert _path_length(pa) == pytest.approx(_path_length(pb))
