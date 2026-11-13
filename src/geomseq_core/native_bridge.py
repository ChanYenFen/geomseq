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
ABI_VERSION = 1


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

    _DLL = lib
    return lib
