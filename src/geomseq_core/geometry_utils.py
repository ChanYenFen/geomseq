"""
Path Optimization Module for GeomSeq -- embroidery/CNC toolpath sorting. Wrapper functions
only (sort_curves_native, sort_points_native, redistribute_arc_lengths_native); GH entry points and hot-reload live in gh/*_component.py, so this module just needs to stay importable.
"""

__author__ = "Yen-Fen Chan"
__date__ = "2026.03.05"
__update__ = "2026.07.04"

import ctypes
import math

try:
    # Package import (pip-installed geomseq_core, or `from geomseq_core import geometry_utils`).
    from . import misc
    from . import native_bridge
except ImportError:
    # GH runs this file directly as a script (no parent package) -- GhPython
    # adds the script's own directory to sys.path, so plain imports resolve.
    import misc
    import native_bridge


# Native (C++) backend (geomseq_core.dll): same greedy + 2-opt algorithm as
# sort_curves_by_rtree, plus a knn_k parameter; DLL binding lives in native_bridge.py.


def sort_curves_native(curves, start_pt=None,
                       use_two_opt=False, two_opt_max_passes=20, knn_k=12,
                       if_flip=True, return_travel_points=False):
    """C++-backed drop-in replacement for sort_curves_by_rtree: `knn_k` sets neighbors queried per greedy hop, `if_flip=False` fixes curve direction (head->tail only, skipping reversal/2-opt), and `start_pt=None` defaults to the origin.
    `return_travel_points=True` adds a 3rd return item: a list of n plain (start_xyz, end_xyz) tuples, one per travel segment -- not Rhino types, since this module doesn't depend on Rhino.
    2-opt tests only the pairs that could shorten the tour, found through the
    kd-tree; it discards no improving move, so it still finishes at a true 2-opt
    local optimum (see CLAUDE.md).
    The cap is 20 rather than `sort_points_native`'s 10 because every shape
    measured converges by 20 and the pruned search is only ever worse than the
    exhaustive one while it is still unconverged -- reaching convergence is what
    makes that weakness go away, and it costs 140 ms at n=50,000."""
    if not curves:
        return ([], []) if not return_travel_points else ([], [], [])

    lib = native_bridge.load_dll()

    # Marshal geometry -> flat double buffers (zero-copy views for ctypes).
    buf, n = misc.curves_to_endpoint_buffer(curves)
    sp     = misc.start_pt_to_buffer(start_pt)

    endpoints_ptr = (ctypes.c_double * len(buf)).from_buffer(buf)
    start_ptr     = (ctypes.c_double * 3).from_buffer(sp)

    # Output buffers the DLL fills in.
    out_order         = (ctypes.c_int * n)()
    out_reversal      = (ctypes.c_int * n)()
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
    """C++-backed greedy + 2-opt sort for plain points (no direction/reversal, unlike sort_curves_native); same inputs minus `if_flip`, `start_pt=None` defaults to the origin.
    Returns (sorted points, original indices).
    2-opt tests only the pairs that could shorten the tour, found through the
    kd-tree; it discards no improving move, so it still finishes at a true 2-opt
    local optimum (see CLAUDE.md).
    The cap is 20 because that is where every shape measured converges, and a
    pruned search is only ever the worse of the two implementations while it is
    still unconverged -- so the cap guarantees convergence rather than rationing
    time. It costs milliseconds at n=64,000."""
    if not points:
        return [], []

    lib = native_bridge.load_dll()

    # Marshal geometry -> flat double buffers (zero-copy views for ctypes).
    buf, n = misc.points_to_buffer(points)
    sp     = misc.start_pt_to_buffer(start_pt)

    points_ptr = (ctypes.c_double * len(buf)).from_buffer(buf)
    start_ptr  = (ctypes.c_double * 3).from_buffer(sp)

    # Output buffer the DLL fills in.
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
    """C++-backed density redistribution of arc lengths (native/redistribute_arc_lengths.cpp); `arc_lengths`/return are flat arc-length floats, not Rhino geometry. `mode`: 0=dense_center (sparse ends, dense middle), 1=dense_sides (dense ends, sparse middle); `corner_indices` are arc_length indices that must survive exactly (e.g. polyline vertices).
    Returns a new list of arc lengths."""
    if not arc_lengths:
        return []

    if low <= 0:
        # low <= 0 never advances the native marching loop -- would hang.
        raise ValueError(f"low must be > 0, got {low}")

    if high < low:
        print(f"[geomseq_core] redistribute_arc_lengths_native: high ({high}) < low ({low}), clamping high = low")
        high = low

    total_length = arc_lengths[-1]
    if high > total_length:
        # Not unsafe (native side clamps), but degenerates to 2 points.
        print(f"[geomseq_core] redistribute_arc_lengths_native: high ({high}) > curve length "
              f"({total_length}) -- result will just be the two endpoints")

    lib = native_bridge.load_dll()

    # The native side only ever needed total_length and the corner arc lengths,
    # so resolve the corners here (O(num_corners) list indexing) instead of
    # marshaling the whole arc length array across the boundary for it to ignore.
    # An out-of-range corner index now raises IndexError here rather than
    # reading out of bounds inside the DLL.
    if corner_indices:
        corner_lengths = [arc_lengths[i] for i in corner_indices]
        corner_ptr = (ctypes.c_double * len(corner_lengths))(*corner_lengths)
        num_corners = len(corner_lengths)
    else:
        corner_ptr = None
        num_corners = 0

    # Upper bound: total_length/min_step steps (native's edge_step/mid_step bottom out at
    # min(high, low), so using `low` alone would undercount + overflow when high < low), plus slack per corner and for rounding.
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

    # Slicing a ctypes array already builds a list; wrapping it in list() again
    # would just copy it a second time.
    return out_arc_lengths[:out_count.value]

def _unit(vx, vy):
    """Unit vector, or the input unchanged at zero length (matches native unit_vec)."""
    n = math.hypot(vx, vy)
    return (vx / n, vy / n) if n else (vx, vy)


def build_turn_waypoints_native(Ex, Ey, a_vx, a_vy, Sx, Sy, b_vx, b_vy,
                                 theta_max_deg, step_len, extend_len):
    """C++-backed smooth turn from path end E (heading a_v) to next start S (heading b_v); each end is extended `extend_len` into the gap and its corner filleted under `theta_max_deg`, `step_len` must be > 0.
    Returns (exit_pts, entry_pts) as plain (x, y) tuples, not Rhino types."""
    if step_len <= 0:
        raise ValueError(f"step_len must be > 0, got {step_len}")

    # The native side caps the turn per waypoint within each fillet, but the
    # exit->entry junction is only smooth when both fillets have room to open
    # up. Warn rather than raise -- the output is still usable, just kinkier.
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

    # Buffer size follows the .cpp header comment's own suggested
    # ceil(180/theta_max_deg) + 2, using the same <= 1e-6 -> 1-degree
    # fallback the native side applies internally -- so this is sized for
    # what the DLL actually runs, not a division by a ~0 or negative value.
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

    # Flat buffer -> (x, y) tuples, only the actually-written prefix.
    exit_pts = [
        (out_exit_pts[i * 2], out_exit_pts[i * 2 + 1])
        for i in range(out_exit_count.value)
    ]
    entry_pts = [
        (out_entry_pts[i * 2], out_entry_pts[i * 2 + 1])
        for i in range(out_entry_count.value)
    ]

    return exit_pts, entry_pts


# Multiple of n the shatter output buffer is first sized at. Named rather than
# inlined because a test and a benchmark both have to agree with it: each checks
# it is looking at the regrow path, and both went quietly meaningless the once
# this moved and they did not.
SHATTER_FIRST_GUESS = 3


def shatter_at_crossings_native(segments, gap_d, touch_tol=0.0, segment_owner=None, test_self=True):
    """C++-backed shatter (native/geometry2d_staging.cpp): cuts segments where they cross each other and removes a gap of `gap_d` centred on each crossing, so the two paths no longer meet there. `gap_d` is the whole gap -- how far apart the two cut ends end up -- and the native side takes half of it off either side.
    Input order decides who yields: for a crossing pair the lower index is left whole and the higher one is cut. Sort beforehand to impose any other priority.
    `touch_tol` is how close, in model units, counts as touching rather than missing -- a T-junction (one endpoint landing on another segment) is a contact, not a crossing, and real ones are rarely drawn exact. 0 means exact only.
    `segment_owner` is one int per segment naming the source curve it came from -- exploding a polyline gives many segments with one owner. With `test_self=False`, pairs sharing an owner are skipped, so a polyline is not cut where it crosses itself. `segment_owner=None` means every segment owns itself, and then `test_self` changes nothing.
    Returns a nested list of plain ((x,y,z), (x,y,z)) tuples, not Rhino types -- one sub-list per input segment, in input order. Its length always equals the input, and a segment the gaps swallow whole comes back as an empty sub-list, so input index i is always result[i].

    The DLL must be current: geometry2d_staging.cpp is one of the sources in the
    README build command, and native_bridge declares this signature, so a stale
    binary fails at load_dll() with a clear error rather than silently."""
    if not segments:
        return []

    if gap_d < 0:
        raise ValueError(f"gap_d must be >= 0, got {gap_d}")

    if touch_tol < 0:
        raise ValueError(f"touch_tol must be >= 0, got {touch_tol}")

    lib = native_bridge.load_dll()

    # Marshal geometry -> flat double buffer (zero-copy view for ctypes).
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

    # First guess. Falling short costs a second full O(n^2) pass, so this is
    # sized from measurement rather than optimism: across n, density and gap,
    # `out_n / n` stays under 2.4 at any gap wide enough to be worth asking for,
    # and over-allocating is nearly free -- 3n instead of 2n adds 0.38 ms at
    # n=16,000, against a 3.45 s run.
    #
    # 3 and not more, because more buys nothing. The distribution is bimodal:
    # either the output is about n, or it explodes past any sane multiple --
    # gap=0 reaches 26x, since nothing is removed so every contact adds a whole
    # piece. 4n and 6n covered exactly the same configurations as 3n. The tail
    # is what out_total is for.
    capacity     = SHATTER_FIRST_GUESS * n
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

    # Pieces arrive grouped in input order, so one cursor walks them in step
    # with the per-segment counts.
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