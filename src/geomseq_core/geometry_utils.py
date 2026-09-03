"""
Path Optimization Module for GeomSeq -- embroidery/CNC toolpath sorting. Wrapper functions
only (sort_curves_native, sort_points_native, redistribute_lookups_native); GH entry points and hot-reload live in gh/*_component.py, so this module just needs to stay importable.
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
                       use_two_opt=False, two_opt_max_passes=10, knn_k=12,
                       if_flip=True, return_travel_points=False, two_opt_mode=0):
    """C++-backed drop-in replacement for sort_curves_by_rtree: `knn_k` sets neighbors queried per greedy hop, `if_flip=False` fixes curve direction (head->tail only, skipping reversal/2-opt), and `start_pt=None` defaults to the origin.
    `return_travel_points=True` adds a 3rd return item: a list of n plain (start_xyz, end_xyz) tuples, one per travel segment -- not Rhino types, since this module doesn't depend on Rhino.
    `two_opt_mode` forces the 2-opt implementation: 0 = auto (pick by the native
    size threshold -- what every normal caller wants), 1 = exhaustive, 2 = windowed.
    Only benchmarks should pass anything but 0; under auto the two paths never run
    at the same n, so their crossover cannot otherwise be measured."""
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
        two_opt_mode,
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
                        use_two_opt=False, two_opt_max_passes=10, knn_k=12):
    """C++-backed greedy + 2-opt sort for plain points (no direction/reversal, unlike sort_curves_native); same inputs minus `if_flip`, `start_pt=None` defaults to the origin.
    Returns (sorted points, original indices)."""
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


def redistribute_lookups_native(lookups, low, high, mode, flat_pct, corner_indices=None):
    """C++-backed density redistribution of arc-length lookups (native/redistribute_lookups.cpp); `lookups`/return are flat arc-length floats, not Rhino geometry. `mode`: 0=dense_center (sparse ends, dense middle), 1=dense_sides (dense ends, sparse middle); `corner_indices` are lookup indices that must survive exactly (e.g. polyline vertices).
    Returns a new list of arc-length lookups."""
    if not lookups:
        return []

    if low <= 0:
        # low <= 0 never advances the native marching loop -- would hang.
        raise ValueError(f"low must be > 0, got {low}")

    if high < low:
        print(f"[geomseq_core] redistribute_lookups_native: high ({high}) < low ({low}), clamping high = low")
        high = low

    total_length = lookups[-1]
    if high > total_length:
        # Not unsafe (native side clamps), but degenerates to 2 points.
        print(f"[geomseq_core] redistribute_lookups_native: high ({high}) > curve length "
              f"({total_length}) -- result will just be the two endpoints")

    lib = native_bridge.load_dll()

    # The native side only ever needed total_length and the corner arc lengths,
    # so resolve the corners here (O(num_corners) list indexing) instead of
    # marshaling the whole lookup array across the boundary for it to ignore.
    # An out-of-range corner index now raises IndexError here rather than
    # reading out of bounds inside the DLL.
    if corner_indices:
        corner_lengths = [lookups[i] for i in corner_indices]
        corner_ptr = (ctypes.c_double * len(corner_lengths))(*corner_lengths)
        num_corners = len(corner_lengths)
    else:
        corner_ptr = None
        num_corners = 0

    # Upper bound: total_length/min_step steps (native's edge_step/mid_step bottom out at
    # min(high, low), so using `low` alone would undercount + overflow when high < low), plus slack per corner and for rounding.
    min_step = low if low < high else high
    max_possible_points = int(total_length / min_step) + num_corners + 10

    out_lookups = (ctypes.c_double * max_possible_points)()
    out_count = ctypes.c_int(0)

    lib.redistribute_lookups(
        total_length,
        low,
        high,
        mode,
        flat_pct,
        corner_ptr,
        num_corners,
        out_lookups,
        ctypes.byref(out_count),
    )

    # Slicing a ctypes array already builds a list; wrapping it in list() again
    # would just copy it a second time.
    return out_lookups[:out_count.value]

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