# geomseq

A C++ library for 2D spatial sequencing and geometric primitives,
with thin Python bindings.

## Background

This C++ library grew out of a real bottleneck in a computational
design pipeline: sorting thousands of geometries into an efficient
traversal path. A pure-Python implementation worked, but performance
dropped sharply at scale — minutes, not seconds, past a few thousand
geometries.

The core algorithms — spatial sequencing and its supporting primitives
 — were reimplemented in C++ for raw speed and scalability at large n.

## Design

Three clean layers, so the geometry core stays portable and independently
testable:

```
Visualization (Grasshopper)   draw results, interactive debugging
        │
        │  2D coords + result data
        │
Bridge (Python)               array marshaling, CAD <-> coordinate mapping
        │
        │  flat (x, y) arrays
        │
Core (C++)                    pure numerical geometry — no CAD dependency
```

The core never returns rendering instructions — how to draw a
result is always the caller's decision. This keeps it usable from
Grasshopper, a plain script, or any other front end.

## Modules

| Module | Purpose | Status |
|--------|---------|--------|
| `sort_curves` | greedy k-NN + 2-opt ordering of curves to minimize travel (direction-aware: reversal flags + optional per-segment travel points) | ✅ |
| `sort_points` | single-point sibling of `sort_curves` (no direction/reversal concept) | ✅ |
| `redistribute_lookups` | redistribute arc-length lookups to a density gradient (dense_center / dense_sides), preserving named corner positions | ✅ |
| `build_turn_waypoints` | travel path between two segments: extends each end into the gap, then fillets both corners so no waypoint turns by more than `theta_max_deg` | ✅ |

## Layout

```
src/
├── geomseq_core/                       # pure-numeric core, no Rhino dependency
│   ├── native/                         # C++ source + compiled binaries
│   │   ├── sort_curves.cpp
│   │   ├── sort_points.cpp             # single-point sibling of sort_curves (no direction/reversal)
│   │   ├── redistribute_lookups.cpp    # arc-length density redistribution (pure 1D, no kd-tree)
│   │   ├── build_turn_waypoints.cpp    # smooth polygonal turn between two path segments' headings
│   │   ├── nanoflann.hpp               # vendored kd-tree (BSD 2-Clause)
│   │   ├── archive/                    # superseded reference implementations (e.g. pre-windowing 2-opt)
│   │   └── geomseq_core.dll            # official .cpp files compiled into one binary (also .dylib / .so per platform)
│   ├── native_bridge.py                # ctypes loading + signatures (platform-aware)
│   ├── geometry_utils.py               # Python-facing wrappers (sort_curves_native, sort_points_native, ...)
│   ├── misc.py                         # coordinate <-> flat-buffer marshaling
│   └── _reload.py                      # dev-mode module unloading for GH hot-reload
├── rhino_utils/                        # depends on RhinoCommon; logic complex/reusable enough not to be a thin GH shell
│   ├── divide_curves.py                # curve -> division points + arc-length lookups
│   └── sample_curve_points.py          # arc-length lookups -> points on a curve
└── gh/                                 # thin Grasshopper component shells (GH I/O only, calls into the layers above)
    ├── definitions/                    # .gh example files
    └── *_component.py

tests/                                  # property tests, plain CPython (no Rhino)
├── fixtures/                           # JSON inputs for the sort tests
└── test_*.py
```

## Build

Rebuild the native library after editing any `.cpp`. All official sources
compile into one shared library (`native_bridge.py` loads a single DLL and
expects `sort_curves`, `sort_points`, `redistribute_lookups`, and
`build_turn_waypoints` all exported from it), per platform (same code,
different compiler):

```
# Windows (x64 Native Tools Command Prompt)
cl /std:c++17 /O2 /LD /EHsc /MT sort_curves.cpp sort_points.cpp redistribute_lookups.cpp build_turn_waypoints.cpp /Fe:geomseq_core.dll

# macOS / Linux
clang++ -std=c++17 -O2 -shared -fPIC -o geomseq_core.dylib sort_curves.cpp sort_points.cpp redistribute_lookups.cpp build_turn_waypoints.cpp
```

The Python bridge auto-selects the right binary (`.dll` / `.dylib` / `.so`) by
platform, so the same Python code runs everywhere.

## Tests

Property tests, run on plain CPython — no Rhino, since the core only ever sees
coordinate arrays. Needs a compiled binary for the current platform (see Build):

```
python tests/test_sort_curves.py
python tests/test_sort_points.py
python tests/test_redistribute_lookups.py
```

Each prints one line per case, then `All tests passed`. The sort tests assert the
result is a permutation of the input and that travel distance never exceeds the
input order's. The redistribute test asserts the output spans the full arc
length, increases strictly, reproduces every corner exactly, and keeps step sizes
within `[low, high]` — except the final step onto `total_length` and steps
rescaled to land on a corner, which the native side breaks the band for by
design.

## Notes

- The C++ core is CAD-independent: it takes and returns plain coordinate arrays,
  so it is testable without Rhino (see Tests) and reusable from any front end.
- `redistribute_lookups`'s `flat_pct` is a **percent (0–100), not a 0–1
  fraction** — `100.0` holds one density for the whole curve, `0.0` fades across
  its entire length. Nothing validates the range, so `1.0` silently gives an
  almost fully graded curve rather than a uniform one.
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