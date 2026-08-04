import os
import platform
import ctypes

_DLL = None  # cached handle so the DLL loads only once per session

_LIB_NAMES = {
    "Windows": "geomseq_core.dll",
    "Darwin":  "geomseq_core.dylib",
    "Linux":   "geomseq_core.so",
}


def load_dll():
    """Load the native library (once) and declare the sort_curves/sort_points signatures."""
    global _DLL
    if _DLL is not None:
        return _DLL

    here = os.path.dirname(os.path.abspath(__file__))
    lib_name = _LIB_NAMES[platform.system()]
    dll_path = os.path.join(here, "native", lib_name)
    lib = ctypes.CDLL(dll_path)

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

    # void redistribute_lookups(const double*, int, double, double, int, double, const int*, int, double*, int*)
    lib.redistribute_lookups.argtypes = [
        ctypes.POINTER(ctypes.c_double),  # lookups (n)
        ctypes.c_int,                     # n
        ctypes.c_double,                  # low
        ctypes.c_double,                  # high
        ctypes.c_int,                     # mode
        ctypes.c_double,                  # flat_pct
        ctypes.POINTER(ctypes.c_int),     # corner_indices (num_corners), nullable
        ctypes.c_int,                     # num_corners
        ctypes.POINTER(ctypes.c_double),  # out_lookups (caller-allocated)
        ctypes.POINTER(ctypes.c_int),     # out_count
    ]
    lib.redistribute_lookups.restype = None

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


    _DLL = lib
    return lib
