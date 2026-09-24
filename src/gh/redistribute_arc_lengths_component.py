"""
Redistribute Arc Lengths Component for GeomSeq
GH entry point: arc-length density redistribution (geometry_utils.redistribute_arc_lengths_native); pure 1D, no Rhino.Geometry needed.
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

from geomseq_core.geometry_utils import redistribute_arc_lengths_native


if __name__ == "__main__":
    nested_arc_lengths = th.tree_to_list(arc_lengths_tree)  # type: ignore

    # corner_indices_tree is optional.
    try:
        nested_corners = th.tree_to_list(corner_indices_tree)
    except Exception:
        nested_corners = None

    out_nested_arc_lengths = []

    for i, arc_lengths in enumerate(nested_arc_lengths):
        corners = nested_corners[i] if nested_corners is not None else None
        new_arc_lengths = redistribute_arc_lengths_native(arc_lengths, low, high, mode, flat_pct,
                                                   corner_indices=corners)
        out_nested_arc_lengths.append(new_arc_lengths)

    # --- Output to Grasshopper ---
    # DataTree output (preserves per-curve grouping)
    out_arc_lengths_tree = th.list_to_tree(out_nested_arc_lengths)
