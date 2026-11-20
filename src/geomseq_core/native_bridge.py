import os
import platform
import ctypes

_DLL = None  # cached handle so the DLL loads only once per session

_LIB_NAMES = {
    "Windows": "geomseq_core.dll",
    "Darwin":  "geomseq_core.dylib",
    "Linux":   "geomseq_core.so",
}


# Must equal geomseq_abi_version() in native/abi_version.cpp, which says when to bump it.
ABI_VERSION = 5


def _check_abi_version(lib, dll_path):
    """Raise if the binary's ABI version is not the one this module declares."""
    try:
        get_version = lib.geomseq_abi_version
    except AttributeError:
        raise RuntimeError(
            "%s has no geomseq_abi_version, so it is older than this Python "
            "wrapper (which expects ABI version %d). Rebuild it from "
            "src/geomseq_core/native/ -- see Build in README.md."
            % (dll_path, ABI_VERSION))
    get_version.argtypes = []
    get_version.restype = ctypes.c_int
    found = get_version()
    if found != ABI_VERSION:
        raise RuntimeError(
            "%s is ABI version %d, but this Python wrapper expects %d. The "
            "binary and the wrapper are from different revisions; rebuild the "
            "binary from src/geomseq_core/native/ -- see Build in README.md."
            % (dll_path, found, ABI_VERSION))


def load_dll():
    """Load the native library (once) and declare every export's signature.
    Always declare argtypes: it is what rejects a call with the wrong argument count."""
    global _DLL
    if _DLL is not None:
        return _DLL

    here = os.path.dirname(os.path.abspath(__file__))
    lib_name = _LIB_NAMES[platform.system()]
    dll_path = os.path.join(here, "native", lib_name)
    lib = ctypes.CDLL(dll_path)
    _check_abi_version(lib, dll_path)

    # void sort_curves(const double*, int, const double*, int, int, int, int, int*, int*, double*)
    lib.sort_curves.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # endpoints (6*n)
        ctypes.c_int,                     # n
        ctypes.POINTER(ctypes.c_double),  # start_pt (3)
        ctypes.c_int,                     # use_two_opt
        ctypes.c_int,                     # two_opt_max_passes
        ctypes.c_int,                     # knn_k
        ctypes.c_int,                     # if_flip
        ctypes.POINTER(ctypes.c_int),     # out_order (n)
        ctypes.POINTER(ctypes.c_int),     # out_reversal (n)
        ctypes.POINTER(ctypes.c_double),  # out_travel_points (n*6)
    ]
    lib.sort_curves.restype = None

    # void sort_points(const double*, int, const double*, int, int, int, int*)
    lib.sort_points.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # points (3*n)
        ctypes.c_int,                     # n
        ctypes.POINTER(ctypes.c_double),  # start_pt (3)
        ctypes.c_int,                     # use_two_opt
        ctypes.c_int,                     # two_opt_max_passes
        ctypes.c_int,                     # knn_k
        ctypes.POINTER(ctypes.c_int),     # out_order (n)
    ]
    lib.sort_points.restype = None

    # void redistribute_arc_lengths(double, double, double, int, double, const double*, int, double*, int*)
    lib.redistribute_arc_lengths.argtypes = [
        ctypes.c_double,                  # total_length
        ctypes.c_double,                  # low
        ctypes.c_double,                  # high
        ctypes.c_int,                     # mode
        ctypes.c_double,                  # flat_pct
        ctypes.POINTER(ctypes.c_double),  # corner_lengths (num_corners), nullable
        ctypes.c_int,                     # num_corners
        ctypes.POINTER(ctypes.c_double),  # out_arc_lengths (caller-allocated)
        ctypes.POINTER(ctypes.c_int),     # out_count
    ]
    lib.redistribute_arc_lengths.restype = None

    # void build_turn_waypoints(....)
    lib.build_turn_waypoints.argtypes = [
        ctypes.c_double, ctypes.c_double,   # Ex, Ey
        ctypes.c_double, ctypes.c_double,   # a_vx, a_vy
        ctypes.c_double, ctypes.c_double,   # Sx, Sy
        ctypes.c_double, ctypes.c_double,   # b_vx, b_vy
        ctypes.c_double,                     # theta_max_deg
        ctypes.c_double,                     # step_len
        ctypes.c_double,                     # extend_len
        ctypes.POINTER(ctypes.c_double),    # out_exit_pts
        ctypes.POINTER(ctypes.c_int),       # out_exit_count
        ctypes.POINTER(ctypes.c_double),    # out_entry_pts
        ctypes.POINTER(ctypes.c_int),       # out_entry_count
    ]
    lib.build_turn_waypoints.restype = None

    # void shatter_at_crossings(const double*, int, double, double, const int*,
    #                           int, double*, int, int*, int*)
    lib.shatter_at_crossings.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # segments (6*n)
        ctypes.c_int,                     # n
        ctypes.c_double,                  # gap_d
        ctypes.c_double,                  # touch_tol
        ctypes.POINTER(ctypes.c_int),     # segment_owner (n), nullable
        ctypes.c_int,                     # test_self
        ctypes.POINTER(ctypes.c_double),  # out_segments (caller-allocated)
        ctypes.c_int,                     # out_capacity, in pieces
        ctypes.POINTER(ctypes.c_int),     # out_piece_counts (n)
        ctypes.POINTER(ctypes.c_int),     # out_total
    ]
    lib.shatter_at_crossings.restype = None

    # int point_in_curve(const double*, const int*, int, const double*, int, double, int*)
    # The only export with a return value: a PIC_* status, negative meaning
    # nothing was written. See native/point_in_curve.cpp for the codes.
    lib.point_in_curve.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # polylines (2*sum(vertex_counts))
        ctypes.POINTER(ctypes.c_int),     # vertex_counts (num_polylines)
        ctypes.c_int,                     # num_polylines, exactly 1
        ctypes.POINTER(ctypes.c_double),  # query_pts (2*num_query)
        ctypes.c_int,                     # num_query
        ctypes.c_double,                  # tol
        ctypes.POINTER(ctypes.c_int),     # out_relationship (num_query)
    ]
    lib.point_in_curve.restype = ctypes.c_int

    # int point_in_curves(const double*, const int*, int, const double*, int, double, int*, int*)
    # As above for any number of polylines, plus which one contains each point.
    lib.point_in_curves.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # polylines (2*sum(vertex_counts))
        ctypes.POINTER(ctypes.c_int),     # vertex_counts (num_polylines)
        ctypes.c_int,                     # num_polylines, 1 or more
        ctypes.POINTER(ctypes.c_double),  # query_pts (2*num_query)
        ctypes.c_int,                     # num_query
        ctypes.c_double,                  # tol
        ctypes.POINTER(ctypes.c_int),     # out_relationship (num_query)
        ctypes.POINTER(ctypes.c_int),     # out_index (num_query), -1 when outside all
    ]
    lib.point_in_curves.restype = ctypes.c_int

    # int coverage_hatch_region(const double*, const int*, int, double, double,
    #                           double, int, double, void**)
    # Results come back as a handle: read with coverage_result_*, then free.
    # See native/coverage_path.cpp for the status codes and field numbers.
    lib.coverage_hatch_region.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # ring_xy (2*sum(ring_counts))
        ctypes.POINTER(ctypes.c_int),     # ring_counts (num_rings)
        ctypes.c_int,                     # num_rings
        ctypes.c_double,                  # spacing
        ctypes.c_double,                  # angle, radians
        ctypes.c_double,                  # phase
        ctypes.c_int,                     # link
        ctypes.c_double,                  # tol
        ctypes.POINTER(ctypes.c_void_p),  # out_handle
    ]
    lib.coverage_hatch_region.restype = ctypes.c_int
    # int coverage_plan(<as coverage_hatch_region, without out_handle>,
    #                   const double* start, int kicks, double tol, void**)
    lib.coverage_plan.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # ring_xy
        ctypes.POINTER(ctypes.c_int),     # ring_counts
        ctypes.c_int,                     # num_rings
        ctypes.c_double,                  # spacing
        ctypes.c_double,                  # angle, radians
        ctypes.c_double,                  # phase
        ctypes.c_int,                     # link
        ctypes.POINTER(ctypes.c_double),  # start (2), nullable
        ctypes.c_int,                     # kicks
        ctypes.c_double,                  # tol
        ctypes.POINTER(ctypes.c_void_p),  # out_handle
    ]
    lib.coverage_plan.restype = ctypes.c_int
    lib.coverage_result_size.argtypes = [ctypes.c_void_p, ctypes.c_int]
    lib.coverage_result_size.restype = ctypes.c_int
    lib.coverage_result_ints.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_int), ctypes.c_int]
    lib.coverage_result_ints.restype = ctypes.c_int
    lib.coverage_result_doubles.argtypes = [
        ctypes.c_void_p, ctypes.c_int, ctypes.POINTER(ctypes.c_double), ctypes.c_int]
    lib.coverage_result_doubles.restype = ctypes.c_int
    lib.coverage_result_free.argtypes = [ctypes.c_void_p]
    lib.coverage_result_free.restype = None

    _DLL = lib
    return lib
