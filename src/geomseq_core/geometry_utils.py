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
    PIC_ERR_POLYLINE_COUNT: "point_in_curve takes exactly one polyline in this stage",
    PIC_ERR_TOO_FEW_VERTS:  "a polyline needs at least 4 vertices (3 corners plus the repeated first)",
    PIC_ERR_NOT_CLOSED:     "a polyline is not closed: its last vertex is not its first within tol",
    PIC_ERR_BAD_ARGUMENT:   "point_in_curve got a null pointer or a negative count",
    PIC_ERR_NEGATIVE_TOL:   "tol must be >= 0",
}


def point_in_curve_native(polylines, query_pts, tol=0.0):
    """Classify 2D query points against closed 2D polylines.

    Takes geometry already flattened into the plane the containment is
    measured in: only x and y are read from each vertex and query point, and
    the caller is responsible for the projection. `polylines` is a sequence of
    polylines, each a sequence of vertices with its first repeated as its
    last; this stage accepts exactly one. `tol` is how close to the boundary
    counts as PIC_COINCIDENT, in model units, and 0 means exact only.

    Returns `(relationship, status)`. `relationship` is one PIC_OUTSIDE /
    PIC_COINCIDENT / PIC_INSIDE per query point. `status` is PIC_OK, or
    PIC_WARN_FLAT_AREA when the projection collapsed and the caller should
    say so.

    Raises ValueError for the negative statuses, where nothing was written."""
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

    lib = native_bridge.load_dll()

    poly_ptr   = (ctypes.c_double * len(flat))(*flat) if flat else None
    counts_ptr = (ctypes.c_int * num_polylines)(*counts) if counts else None
    query_ptr  = (ctypes.c_double * len(flat_q))(*flat_q) if flat_q else None

    out_relationship = (ctypes.c_int * n_query)() if n_query else (ctypes.c_int * 1)()

    status = lib.point_in_curve(poly_ptr, counts_ptr, num_polylines,
                                query_ptr, n_query, ctypes.c_double(tol),
                                out_relationship)

    if status < 0:
        raise ValueError("%s (point_in_curve status %d)"
                         % (_PIC_ERRORS.get(status, "point_in_curve refused the input"), status))

    return [out_relationship[i] for i in range(n_query)], status