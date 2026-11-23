"""Python wrappers around the native library: one *_native function per export.
They take and return plain values or duck-typed geometry, never Rhino types."""

__author__ = "Yen-Fen Chan"
__date__ = "2026.03.05"
__update__ = "2026.07.04"

import ctypes
import math

try:
    from . import misc
    from . import native_bridge
except ImportError:
    # GhPython runs this file as a script, with no parent package.
    import misc
    import native_bridge


def sort_curves_native(curves, start_pt=None,
                       use_two_opt=False, two_opt_max_passes=20, knn_k=12,
                       if_flip=True, return_travel_points=False):
    """Sort curves to shorten the travel between them (greedy, then optional 2-opt).

    `start_pt=None` starts at the origin. `if_flip=False` keeps every curve's
    direction and skips 2-opt. `knn_k` is unused. Returns (sorted curves,
    original indices); `return_travel_points=True` adds a list of n
    (start_xyz, end_xyz) tuples, one per travel segment."""
    if not curves:
        return ([], []) if not return_travel_points else ([], [], [])

    lib = native_bridge.load_dll()

    buf, n = misc.curves_to_endpoint_buffer(curves)
    sp     = misc.start_pt_to_buffer(start_pt)

    endpoints_ptr = (ctypes.c_double * len(buf)).from_buffer(buf)
    start_ptr     = (ctypes.c_double * 3).from_buffer(sp)

    out_order         = (ctypes.c_int * n)()
    out_reversal     = (ctypes.c_int * n)()
    out_travel_points = (ctypes.c_double * (n * 6))()

    lib.sort_curves(
        endpoints_ptr,
        n,
        start_ptr,
        1 if use_two_opt else 0,
        two_opt_max_passes,
        knn_k,
        1 if if_flip else 0,
        out_order,
        out_reversal,
        out_travel_points,
    )

    order    = list(out_order)
    reversal = list(out_reversal)

    ordered_curves = misc.apply_order(curves, order, reversal)

    if not return_travel_points:
        return ordered_curves, order

    travel_points = [
        (
            (out_travel_points[k * 6 + 0], out_travel_points[k * 6 + 1], out_travel_points[k * 6 + 2]),
            (out_travel_points[k * 6 + 3], out_travel_points[k * 6 + 4], out_travel_points[k * 6 + 5]),
        )
        for k in range(n)
    ]
    return ordered_curves, order, travel_points


def sort_points_native(points, start_pt=None,
                        use_two_opt=False, two_opt_max_passes=20, knn_k=12):
    """Sort points to shorten the travel between them (greedy, then optional 2-opt).

    `start_pt=None` starts at the origin. `knn_k` is unused.
    Returns (sorted points, original indices)."""
    if not points:
        return [], []

    lib = native_bridge.load_dll()

    buf, n = misc.points_to_buffer(points)
    sp     = misc.start_pt_to_buffer(start_pt)

    points_ptr = (ctypes.c_double * len(buf)).from_buffer(buf)
    start_ptr  = (ctypes.c_double * 3).from_buffer(sp)

    out_order = (ctypes.c_int * n)()

    lib.sort_points(
        points_ptr,
        n,
        start_ptr,
        1 if use_two_opt else 0,
        two_opt_max_passes,
        knn_k,
        out_order,
    )

    order = list(out_order)
    ordered_points = [points[i] for i in order]
    return ordered_points, order


def redistribute_arc_lengths_native(arc_lengths, low, high, mode, flat_pct, corner_indices=None):
    """Redistribute arc lengths along a curve to a density gradient.

    `low`/`high` are the smallest and largest step. `mode`: 0 = dense_center
    (sparse ends, dense middle), 1 = dense_sides. `flat_pct` is a percent
    (0-100) of the length held at constant density. `corner_indices` are
    indices into `arc_lengths` that must survive exactly (e.g. polyline
    vertices). Returns a new list of arc lengths."""
    if not arc_lengths:
        return []

    if low <= 0:
        # The native loop would never advance.
        raise ValueError(f"low must be > 0, got {low}")

    if high < low:
        print(f"[geomseq_core] redistribute_arc_lengths_native: high ({high}) < low ({low}), clamping high = low")
        high = low

    total_length = arc_lengths[-1]
    if high > total_length:
        print(f"[geomseq_core] redistribute_arc_lengths_native: high ({high}) > curve length "
              f"({total_length}) -- result will just be the two endpoints")

    lib = native_bridge.load_dll()

    # Corners are resolved to arc lengths here; the native side never sees the
    # array. An out-of-range index raises IndexError.
    if corner_indices:
        corner_lengths = [arc_lengths[i] for i in corner_indices]
        corner_ptr = (ctypes.c_double * len(corner_lengths))(*corner_lengths)
        num_corners = len(corner_lengths)
    else:
        corner_ptr = None
        num_corners = 0

    # Upper bound on the output: one step of min(low, high) at a time, plus
    # slack per corner and for rounding. Undercounting overruns the buffer.
    min_step = low if low < high else high
    max_possible_points = int(total_length / min_step) + num_corners + 10

    out_arc_lengths = (ctypes.c_double * max_possible_points)()
    out_count = ctypes.c_int(0)

    lib.redistribute_arc_lengths(
        total_length,
        low,
        high,
        mode,
        flat_pct,
        corner_ptr,
        num_corners,
        out_arc_lengths,
        ctypes.byref(out_count),
    )

    return out_arc_lengths[:out_count.value]  # slicing a ctypes array gives a list

def _unit(vx, vy):
    """Unit vector, or the input unchanged at zero length (matches native unit_vec)."""
    n = math.hypot(vx, vy)
    return (vx / n, vy / n) if n else (vx, vy)


def build_turn_waypoints_native(Ex, Ey, a_vx, a_vy, Sx, Sy, b_vx, b_vy,
                                 theta_max_deg, step_len, extend_len):
    """Smooth turn from path end E (heading a_v) to the next start S (heading b_v). 2D.

    Each end is extended `extend_len` into the gap and its corner filleted so
    no waypoint turns by more than `theta_max_deg`. `step_len` must be > 0.
    Returns (exit_pts, entry_pts) as lists of (x, y) tuples."""
    if step_len <= 0:
        raise ValueError(f"step_len must be > 0, got {step_len}")

    # The angle limit holds within each fillet, not where the two meet: warn
    # when they have too little room.
    gap = math.hypot(Sx - Ex, Sy - Ey)
    if gap < 2.0 * extend_len:
        print(f"[geomseq_core] build_turn_waypoints_native: warning: E-S distance {gap:.4f} "
              f"< 2*extend_len {2.0 * extend_len:.4f}, junction angle not guaranteed")

    a_hx, a_hy = _unit(a_vx, a_vy)
    b_hx, b_hy = _unit(b_vx, b_vy)
    bridge = math.hypot((Sx - b_hx * extend_len) - (Ex + a_hx * extend_len),
                        (Sy - b_hy * extend_len) - (Ey + a_hy * extend_len))
    if bridge < step_len:
        print(f"[geomseq_core] build_turn_waypoints_native: warning: E_extend-S_extend distance "
              f"{bridge:.4f} < step_len {step_len:.4f}, junction angle not guaranteed")

    lib = native_bridge.load_dll()

    # Buffer bound from the .cpp header, with the native side's own fallback
    # to 1 degree when theta_max_deg <= 1e-6.
    effective_theta = theta_max_deg if theta_max_deg > 1e-6 else 1.0
    max_points = math.ceil(180 / effective_theta) + 2

    out_exit_pts    = (ctypes.c_double * (max_points * 2))()
    out_exit_count  = ctypes.c_int()
    out_entry_pts   = (ctypes.c_double * (max_points * 2))()
    out_entry_count = ctypes.c_int()

    lib.build_turn_waypoints(
        Ex, Ey, a_vx, a_vy, Sx, Sy, b_vx, b_vy,
        theta_max_deg, step_len, extend_len,
        out_exit_pts, ctypes.byref(out_exit_count),
        out_entry_pts, ctypes.byref(out_entry_count),
    )

    exit_pts = [
        (out_exit_pts[i * 2], out_exit_pts[i * 2 + 1])
        for i in range(out_exit_count.value)
    ]
    entry_pts = [
        (out_entry_pts[i * 2], out_entry_pts[i * 2 + 1])
        for i in range(out_entry_count.value)
    ]

    return exit_pts, entry_pts


# Multiple of n the shatter output buffer is first sized at. A test and a
# benchmark read it to know they are exercising the regrow path.
SHATTER_FIRST_GUESS = 3


def shatter_at_crossings_native(segments, gap_d, touch_tol=0.0, segment_owner=None, test_self=True):
    """Cut segments where they meet and open a gap of `gap_d` there. 2D; z is interpolated.

    `gap_d` is the whole gap: half comes off either side of a contact. Input
    order decides who yields: of a meeting pair the lower index stays whole and
    the higher one is cut. `touch_tol` is how close, in model units, counts as
    touching (a T-junction); 0 means exact only. `segment_owner` is one int per
    segment naming its source curve; with `test_self=False`, pairs sharing an
    owner are skipped. `segment_owner=None` means every segment owns itself.

    Returns one sub-list of ((x,y,z), (x,y,z)) pieces per input segment, in
    input order; a segment the gaps swallow whole gives an empty sub-list."""
    if not segments:
        return []

    if gap_d < 0:
        raise ValueError(f"gap_d must be >= 0, got {gap_d}")

    if touch_tol < 0:
        raise ValueError(f"touch_tol must be >= 0, got {touch_tol}")

    lib = native_bridge.load_dll()

    buf, n = misc.curves_to_endpoint_buffer(segments)
    seg_ptr = (ctypes.c_double * len(buf)).from_buffer(buf)

    if segment_owner is None:
        owner_ptr = None
    else:
        if len(segment_owner) != n:
            raise ValueError(f"segment_owner must have one entry per segment: got {len(segment_owner)}, expected {n}")
        owner_ptr = (ctypes.c_int * n)(*segment_owner)

    out_piece_counts = (ctypes.c_int * n)()
    out_total        = ctypes.c_int(0)

    # First guess; if it falls short the native side reports the size needed
    # and the call is repeated once.
    capacity    = SHATTER_FIRST_GUESS * n
    out_segments = (ctypes.c_double * (capacity * 6))()

    lib.shatter_at_crossings(seg_ptr, n, ctypes.c_double(gap_d), ctypes.c_double(touch_tol),
                             owner_ptr, 1 if test_self else 0,
                             out_segments, capacity,
                             out_piece_counts, ctypes.byref(out_total))

    if out_total.value > capacity:
        capacity     = out_total.value
        out_segments = (ctypes.c_double * (capacity * 6))()
        lib.shatter_at_crossings(seg_ptr, n, ctypes.c_double(gap_d), ctypes.c_double(touch_tol),
                                 owner_ptr, 1 if test_self else 0,
                                 out_segments, capacity,
                                 out_piece_counts, ctypes.byref(out_total))

    # Pieces arrive grouped in input order.
    result = []
    cursor = 0
    for i in range(n):
        pieces = []
        for _ in range(out_piece_counts[i]):
            o = cursor * 6
            pieces.append((
                (out_segments[o + 0], out_segments[o + 1], out_segments[o + 2]),
                (out_segments[o + 3], out_segments[o + 4], out_segments[o + 5]),
            ))
            cursor += 1
        result.append(pieces)

    return result


# point_in_curve's status codes, as native/point_in_curve.cpp defines them.
# Below zero nothing was written, and the wrapper raises instead of returning.
PIC_OK                 = 0
PIC_WARN_FLAT_AREA     = 1
PIC_ERR_POLYLINE_COUNT = -1
PIC_ERR_TOO_FEW_VERTS  = -2
PIC_ERR_NOT_CLOSED     = -3
PIC_ERR_BAD_ARGUMENT   = -4
PIC_ERR_NEGATIVE_TOL   = -5

# What point_in_curve writes per query point: the integers Grasshopper's own
# Point In Curve reports in R. NOT Rhino's PointContainment enum, which numbers
# the same three states differently (Unset 0, Inside 1, Outside 2, Coincident 3).
PIC_OUTSIDE    = 0
PIC_COINCIDENT = 1
PIC_INSIDE     = 2

_PIC_ERRORS = {
    PIC_ERR_POLYLINE_COUNT: "point_in_curve takes exactly one polyline; point_in_curves takes one or more",
    PIC_ERR_TOO_FEW_VERTS:  "a polyline needs at least 4 vertices (3 corners plus the repeated first)",
    PIC_ERR_NOT_CLOSED:     "a polyline is not closed: its last vertex is not its first within tol",
    PIC_ERR_BAD_ARGUMENT:   "point_in_curve got a null pointer or a negative count",
    PIC_ERR_NEGATIVE_TOL:   "tol must be >= 0",
}


def _pic_marshal(polylines, query_pts, tol):
    """The ctypes arrays both point-in-curve wrappers hand to the native side."""
    if tol < 0:
        raise ValueError(f"tol must be >= 0, got {tol}")

    counts = [len(p) for p in polylines]
    num_polylines = len(counts)

    flat = []
    for p in polylines:
        for v in p:
            flat.append(float(v[0]))
            flat.append(float(v[1]))

    n_query = len(query_pts)
    flat_q = []
    for q in query_pts:
        flat_q.append(float(q[0]))
        flat_q.append(float(q[1]))

    poly_ptr   = (ctypes.c_double * len(flat))(*flat) if flat else None
    counts_ptr = (ctypes.c_int * num_polylines)(*counts) if counts else None
    query_ptr  = (ctypes.c_double * len(flat_q))(*flat_q) if flat_q else None

    return poly_ptr, counts_ptr, num_polylines, query_ptr, n_query


def _pic_raise(name, status):
    raise ValueError("%s (%s status %d)"
                     % (_PIC_ERRORS.get(status, name + " refused the input"), name, status))


def point_in_curve_native(polylines, query_pts, tol=0.0):
    """Classify 2D query points against one closed 2D polyline.

    Takes geometry already flattened into the plane the containment is
    measured in: only x and y are read from each vertex and query point, and
    the caller is responsible for the projection. `polylines` is a sequence of
    polylines, each a sequence of vertices with its first repeated as its
    last, and must hold exactly one -- several is `point_in_curves_native`.
    `tol` is how close to the boundary counts as PIC_COINCIDENT, in model
    units, and 0 means exact only.

    Returns `(relationship, status)`. `relationship` is one PIC_OUTSIDE /
    PIC_COINCIDENT / PIC_INSIDE per query point. `status` is PIC_OK, or
    PIC_WARN_FLAT_AREA when the projection collapsed and the caller should
    say so.

    Raises ValueError for the negative statuses, where nothing was written."""
    poly_ptr, counts_ptr, num_polylines, query_ptr, n_query = \
        _pic_marshal(polylines, query_pts, tol)

    lib = native_bridge.load_dll()
    out_relationship = (ctypes.c_int * max(n_query, 1))()

    status = lib.point_in_curve(poly_ptr, counts_ptr, num_polylines,
                                query_ptr, n_query, ctypes.c_double(tol),
                                out_relationship)
    if status < 0:
        _pic_raise("point_in_curve", status)

    return [out_relationship[i] for i in range(n_query)], status


def point_in_curves_native(polylines, query_pts, tol=0.0):
    """Classify 2D query points against several closed 2D polylines, and say
    which one contains each.

    Inputs are as for `point_in_curve_native`, with any number of polylines
    from one up. Each point is tested against them in order and stops at the
    first that contains it, on the boundary or inside; nested curves therefore
    answer for whichever containing curve is listed first.

    Returns `(relationship, index, status)`. `index` is the containing
    polyline's position in `polylines`, or -1 for a point none contains, which
    is always PIC_OUTSIDE.

    Raises ValueError for the negative statuses, where nothing was written."""
    poly_ptr, counts_ptr, num_polylines, query_ptr, n_query = \
        _pic_marshal(polylines, query_pts, tol)

    lib = native_bridge.load_dll()
    out_relationship = (ctypes.c_int * max(n_query, 1))()
    out_index        = (ctypes.c_int * max(n_query, 1))()

    status = lib.point_in_curves(poly_ptr, counts_ptr, num_polylines,
                                 query_ptr, n_query, ctypes.c_double(tol),
                                 out_relationship, out_index)
    if status < 0:
        _pic_raise("point_in_curves", status)

    return ([out_relationship[i] for i in range(n_query)],
            [out_index[i] for i in range(n_query)],
            status)

# --- coverage path -------------------------------------------------------------
# The C++ port of coverage_path.py. Status codes and field numbers mirror
# native/coverage_path.cpp; the messages are the pure-Python module's own, so
# either raises the same ValueError for the same input.

_COVERAGE_ERRORS = {
    -1: "bad argument to the native coverage path",
    -2: "tol cannot be negative",
    -3: "a ring needs at least three distinct vertices",
    -4: "the region has no height along the sweep direction",
    -5: "a scanline crossed the boundary an odd number of times; the rings "
        "are not closed or they cross each other",
    -6: "spacing must be greater than 0",
    -7: "link must be ZIGZAG (0), ONE_WAY (1) or ONE_WAY_SEAMS (2)",
    -8: "kicks cannot be negative",
    -9: "the cells left lie in another piece of the region",
}

(_F_CELL_COUNTS, _F_CELL_POINTS, _F_LINE_COUNTS, _F_LINE_POINTS,
 _F_WALK_COUNTS, _F_WALK_POINTS, _F_ORDER, _F_PIECE_KINDS, _F_PIECE_COUNTS,
 _F_PIECE_POINTS, _F_PIECE_VISIBLE) = range(11)


def _coverage_check(status):
    if status != 0:
        raise ValueError(_COVERAGE_ERRORS.get(status, "native coverage path failed (%d)" % status))


def _result_ints(lib, handle, field):
    n = lib.coverage_result_size(handle, field)
    buf = (ctypes.c_int * max(n, 1))()
    lib.coverage_result_ints(handle, field, buf, n)
    return list(buf[:n])


def _result_points(lib, handle, field):
    n = lib.coverage_result_size(handle, field)
    buf = (ctypes.c_double * max(n, 1))()
    lib.coverage_result_doubles(handle, field, buf, n)
    return [(buf[i], buf[i + 1]) for i in range(0, n, 2)]


def _split(items, counts):
    out, at = [], 0
    for c in counts:
        out.append(items[at:at + c])
        at += c
    return out


def _ring_buffers(rings):
    counts = [len(r) for r in rings]
    flat = [float(v) for r in rings for p in r for v in (p[0], p[1])]
    return ((ctypes.c_double * max(len(flat), 1))(*flat),
            (ctypes.c_int * max(len(counts), 1))(*counts))


def _hatch_from(lib, handle, Fill):
    outlines = _split(_result_points(lib, handle, _F_CELL_POINTS),
                      _result_ints(lib, handle, _F_CELL_COUNTS))
    line_pts = _result_points(lib, handle, _F_LINE_POINTS)
    lines = _split([(line_pts[i], line_pts[i + 1]) for i in range(0, len(line_pts), 2)],
                   _result_ints(lib, handle, _F_LINE_COUNTS))
    walks = _split(_result_points(lib, handle, _F_WALK_POINTS),
                   _result_ints(lib, handle, _F_WALK_COUNTS))
    fills = [Fill(i, walks[4 * i], walks[4 * i + 2], lines[i]) for i in range(len(outlines))]
    return outlines, fills


def plan_coverage_native(rings, spacing, angle=0.0, phase=0.0, link=0, start=None,
                         kicks=0, tol=1e-9):
    """hatch_region followed by coverage_path.plan, in C++. Returns
    (outlines, fills, plan), plan a coverage_path.Plan as the Python gives."""
    try:
        from .coverage_path import FILL, Fill, Piece, Plan
    except ImportError:
        from coverage_path import FILL, Fill, Piece, Plan
    lib = native_bridge.load_dll()
    xy, ring_counts = _ring_buffers(rings)
    start_buf = None if start is None else (ctypes.c_double * 2)(float(start[0]), float(start[1]))
    handle = ctypes.c_void_p()
    _coverage_check(lib.coverage_plan(xy, ring_counts, len(rings), spacing, angle, phase, link,
                                      start_buf, kicks, tol, ctypes.byref(handle)))
    try:
        outlines, fills = _hatch_from(lib, handle, Fill)
        flat_order = _result_ints(lib, handle, _F_ORDER)
        kinds = _result_ints(lib, handle, _F_PIECE_KINDS)
        points = _split(_result_points(lib, handle, _F_PIECE_POINTS),
                        _result_ints(lib, handle, _F_PIECE_COUNTS))
        n = lib.coverage_result_size(handle, _F_PIECE_VISIBLE)
        buf = (ctypes.c_double * max(n, 1))()
        lib.coverage_result_doubles(handle, _F_PIECE_VISIBLE, buf, n)
        visible = list(buf[:n])
    finally:
        lib.coverage_result_free(handle)
    order = [(flat_order[i], flat_order[i + 1]) for i in range(0, len(flat_order), 2)]
    pieces, k = [], 0
    for kind, pts, vis in zip(kinds, points, visible):
        if kind == FILL:
            c, d = order[k]
            k += 1
            pieces.append(Piece(kind, pts, cell=c, direction=d))
        else:
            pieces.append(Piece(kind, pts, visible=vis))
    return outlines, fills, Plan(order, pieces)


def hatch_region_native(rings, spacing, angle=0.0, phase=0.0, link=0, tol=1e-9):
    """coverage_path.hatch_region in C++. Returns (outlines, fills):
    outlines[i] the outline of cell i, fills[i] a coverage_path.Fill with
    its walks and lines, as the Python function gives them."""
    try:
        from .coverage_path import Fill
    except ImportError:
        from coverage_path import Fill
    lib = native_bridge.load_dll()
    xy, ring_counts = _ring_buffers(rings)
    handle = ctypes.c_void_p()
    _coverage_check(lib.coverage_hatch_region(xy, ring_counts, len(rings), spacing, angle,
                                              phase, link, tol, ctypes.byref(handle)))
    try:
        return _hatch_from(lib, handle, Fill)
    finally:
        lib.coverage_result_free(handle)


_RESAMPLE_ERRORS = {
    -1: "bad argument to the native resample_on_lattice",
    -2: "spacing must be greater than 0",
    -3: "step must be greater than 0",
    -4: "stagger must be greater than 0",
    -5: "free_step must be greater than 0",
    -6: "min_step cannot be negative",
}


def resample_on_lattice_native(runs, spacing, step, angle=0.0, phase=0.0, stagger=4.0,
                               free_step=None, skip_row_ends=False, min_step=None, tol=1e-9):
    """resample_on_lattice.resample_on_lattice in C++, same arguments and
    result: the points as (x, y), in order."""
    lib = native_bridge.load_dll()
    free_step = step if free_step is None else free_step
    min_step = spacing if min_step is None else min_step
    counts = [len(points) for points, _ in runs]
    flat = [float(v) for points, _ in runs for p in points for v in (p[0], p[1])]
    xy = (ctypes.c_double * max(len(flat), 1))(*flat)
    run_counts = (ctypes.c_int * max(len(runs), 1))(*counts)
    run_free = (ctypes.c_int * max(len(runs), 1))(*[1 if free else 0 for _, free in runs])
    capacity = 2 * sum(counts) + 16
    while True:
        out = (ctypes.c_double * (2 * capacity))()
        need = ctypes.c_int()
        status = lib.resample_on_lattice(xy, run_counts, run_free, len(runs), spacing, step, angle,
                                         phase, stagger, free_step, 1 if skip_row_ends else 0,
                                         min_step, tol, out, capacity, ctypes.byref(need))
        if status != 0:
            raise ValueError(_RESAMPLE_ERRORS.get(status, "native resample_on_lattice failed (%d)" % status))
        if need.value <= capacity:
            return [(out[2 * i], out[2 * i + 1]) for i in range(need.value)]
        capacity = need.value
