# geomseq

A 2D computational geometry library in C++, with thin Python bindings.

`geomseq_core` provides fast, dependency-light primitives for planar geometry —
point/curve ordering, orientation tests, segment intersection, and related
operations. The core is pure C++ operating on flat coordinate arrays and knows
nothing about any CAD environment; a thin Python layer bridges it into
Rhino/Grasshopper for input and visualization.

## Design

Three clean layers, so the geometry core stays portable and independently
testable:

```
Visualization (Grasshopper)   draw results, interactive debugging
        │  2D coords + result data
Bridge (Python)               array marshaling, CAD <-> coordinate mapping
        │  flat (x, y) arrays
Core (C++)                    pure numerical geometry — no CAD dependency
```

The C++ core only ever receives and returns numbers (coordinates, indices,
flags). It never returns rendering instructions — how to draw a result is the
caller's decision. This keeps the core usable from Grasshopper, a plain script,
or any other front end.

## Modules

| Module | Purpose | Status |
|--------|---------|--------|
| `sort_curves` | greedy k-NN + 2-opt ordering of curves/points to minimize travel | ✅ |
| `redistribute_lookups` | redistribute arc-length lookups to a density gradient (dense_center / dense_sides), preserving named corner positions | ✅ |
| _(planned)_ orientation | signed area / clockwise test for polygons | — |
| _(planned)_ segment intersection | self-intersections of a polyline (2D) | — |

## Layout

```
src/
└── geomseq_core/            # Python package
    ├── native/                 # C++ core
    │   ├── sort_curves.cpp
    │   ├── sort_points.cpp     # single-point sibling of sort_curves (no direction/reversal)
    │   ├── redistribute_lookups.cpp # arc-length density redistribution (pure 1D, no kd-tree)
    │   ├── nanoflann.hpp       # vendored kd-tree (BSD 2-Clause)
    │   └── geomseq_core.dll    # all official .cpp compiled into one binary (also .dylib / .so per platform)
    ├── native_bridge.py        # ctypes loading + signatures (platform-aware)
    ├── geometry_utils.py       # Python-facing API
    └── misc.py                 # coordinate <-> flat-buffer marshaling
```

## Build

Rebuild the native library after editing any `.cpp`. All official sources
compile into one shared library (`native_bridge.py` loads a single DLL and
expects `sort_curves`, `sort_points`, and `redistribute_lookups` all exported
from it), per platform (same code, different compiler):

```
# Windows (x64 Native Tools Command Prompt)
cl /std:c++17 /O2 /LD /EHsc /MT sort_curves.cpp sort_points.cpp redistribute_lookups.cpp /Fe:geomseq_core.dll

# macOS / Linux
clang++ -std=c++17 -O2 -shared -fPIC -o geomseq_core.dylib sort_curves.cpp sort_points.cpp redistribute_lookups.cpp
```

`sort_points_no_crossing.cpp` + `geometry2d.cpp` are shelved (kept in
`native/` but deliberately left out of the build -- see native_bridge.py's
comment above the shelved-feature note). Don't add them to the command above
unless that feature gets picked back up.

The Python bridge auto-selects the right binary (`.dll` / `.dylib` / `.so`) by
platform, so the same Python code runs everywhere.

## Notes

- The C++ core is CAD-independent and can be unit-tested on plain coordinate
  arrays without Rhino.
- Correctness is verified by cross-checking against a reference implementation
  (e.g. total travel distance for ordering).
- `sort_curves`'s 2-opt pass dispatches on `n`: exhaustive O(n²) at or below
  ~10,000 curves, a windowed kd-tree version (K=500 nearest candidate edges,
  ~O(n log n)) above that. Cut a 50k-curve case from ~3 min to ~43s; below
  the threshold the exhaustive path is still faster in practice (kd-tree
  overhead isn't worth it at small n). See `archive/sort_curves_v1_
  windowed2opt_backup.cpp` for the pre-windowing reference version. The
  greedy k-NN phase still has its own theoretical O(n²) worst case
  (unaddressed) from filtering already-used points out of a static kd-tree.

## License

See [LICENSE](LICENSE). Vendors [nanoflann](https://github.com/jlblancoc/nanoflann)
(BSD 2-Clause) — see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).