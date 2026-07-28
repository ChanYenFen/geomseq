"""
Redistribute Lookups Component for GeomSeq
GH entry point: arc-length density redistribution
(geometry_utils.redistribute_lookups_native). Pure 1D arithmetic -- no
Rhino.Geometry needed here; lookups in and out are flat float lists.
"""

import os
import sys

import ghpythonlib.treehelpers as th

try:
    import geomseq_core
except ImportError:
    # Dev mode: geomseq_core isn't installed / not on sys.path yet -- add its
    # parent src/ dir (this file lives in src/gh/, geomseq_core lives in src/geomseq_core/).
    _SRC = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    if _SRC not in sys.path:
        sys.path.insert(0, _SRC)

from geomseq_core import _reload

# Force-reload geometry_utils/misc so edits show up without restarting Rhino.
# Skip native_bridge -- reloading it would reload geomseq_core.dll every recompute (slow).
_reload.unload_modules("geomseq_core.misc")
_reload.unload_modules("geomseq_core.geometry_utils")

from geomseq_core.geometry_utils import redistribute_lookups_native


if __name__ == "__main__":
    nested_lookups = th.tree_to_list(lookups_tree)  # type: ignore

    # corner_indices_tree is optional. When unconnected this environment hands
    # back an empty DataTree (not None), and th.tree_to_list() raises IndexError
    # on that -- so "is not None" alone isn't enough of a guard. Fall back to
    # "no corners" on any failure to read it.
    try:
        nested_corners = th.tree_to_list(corner_indices_tree)
    except Exception:
        nested_corners = None

    out_nested_lookups = []

    for i, lookups in enumerate(nested_lookups):
        corners = nested_corners[i] if nested_corners is not None else None
        new_lookups = redistribute_lookups_native(lookups, low, high, mode, flat_pct,
                                                   corner_indices=corners)
        out_nested_lookups.append(new_lookups)

    # --- Output to Grasshopper ---
    # DataTree output (preserves per-curve grouping)
    out_lookups_tree = th.list_to_tree(out_nested_lookups)
