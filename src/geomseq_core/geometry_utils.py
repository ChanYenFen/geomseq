"""
Path Optimization Module for GeomSeq -- embroidery/CNC toolpath sorting. Wrapper functions
only (sort_curves_native, sort_points_native, redistribute_lookups_native); GH entry points and hot-reload live in gh/*_component.py, so this module just needs to stay importable.
"""

__author__ = "Yen-Fen Chan"
__date__ = "2026.03.05"
__update__ = "2026.07.04"

import ctypes

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
                       if_flip=True, return_travel_points=False):
    """C++-backed drop-in replacement for sort_curves_by_rtree: `knn_k` sets neighbors queried per greedy hop, `if_flip=False` fixes curve direction (head->tail only, skipping reversal/2-opt), and `start_pt=None` defaults to the origin.
    `return_travel_points=True` adds a 3rd return item: a list of n plain (start_xyz, end_xyz) tuples, one per travel segment -- not Rhino types, since this module doesn't depend on Rhino."""
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

    buf, n = misc.floats_to_buffer(lookups)
    lookups_ptr = (ctypes.c_double * n).from_buffer(buf)

    if corner_indices:
        corner_buf = (ctypes.c_int * len(corner_indices))(*corner_indices)
        corner_ptr = corner_buf
        num_corners = len(corner_indices)
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
        lookups_ptr,
        n,
        low,
        high,
        mode,
        flat_pct,
        corner_ptr,
        num_corners,
        out_lookups,
        ctypes.byref(out_count),
    )

    return list(out_lookups[:out_count.value])