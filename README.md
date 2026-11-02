# geomseq

A C++ library for spatial sequencing and geometric primitives, with thin
Python bindings and a Grasshopper plug-in built on the same core
(see [src/gha/README.md](src/gha/README.md)).

## Background

This C++ library grew out of a real bottleneck in a computational
design pipeline: sorting thousands of geometries into an efficient
traversal path. A pure-Python implementation worked, but performance
dropped sharply at scale — minutes, not seconds, past a few thousand
geometries.

The core algorithms — spatial sequencing and its supporting primitives
 — were reimplemented in C++ for raw speed and scalability at large n.

## Design

The C++ core is the engine; everything else is a caller. It takes and returns
plain coordinate arrays and never returns rendering instructions — how to draw
a result is always the caller's decision. That is what keeps it testable
without Rhino, and reusable from any front end.

```
  GeomSeq.gha (C#)              geomseq_core/ (Python)
  the shipped plug-in           the development interface
  Grasshopper users             tests · benchmarks · CAD scripting
          │                              │
          │  P/Invoke                    │  ctypes
          └──────────────┬───────────────┘
                         │  flat (x, y, z) arrays
                         │
                   Core (C++)
                   pure numerical geometry — no CAD dependency
```

Two callers, developed and released on different schedules:

- **`geomseq_core/` (Python)** is how the library is built and verified. It is
  the only way to exercise the core *outside* Rhino — `tests/` and
  `benchmarks/` both run on plain CPython — and it doubles as the scripting
  interface inside a CAD session. `src/gh/` holds GHPython component shells
  that hot-reload on edit, which is how results get checked *inside* Rhino.
- **`GeomSeq.gha` (C#)** is the packaged product: two files dropped into
  Grasshopper's `Libraries` folder, for users who never touch Python. It
  P/Invokes the same binary rather than going through the Python bridge, and
  carries its own version number and compatibility promises — see
  [src/gha/README.md](src/gha/README.md).

## Modules

| Module | Purpose | Status |
|--------|---------|--------|
| `sort_curves` | greedy k-NN + 2-opt ordering of curves to minimize travel (direction-aware: reversal flags + optional per-segment travel points) | ✅ |
| `sort_points` | single-point sibling of `sort_curves` (no direction/reversal concept) | ✅ |
| `redistribute_arc_lengths` | redistribute arc lengths to a density gradient (dense_center / dense_sides), preserving named corner positions | ✅ |
| `build_turn_waypoints` | travel path between two segments: extends each end into the gap, then fillets both corners so no waypoint turns by more than `theta_max_deg` (**2D only** — the ABI takes x/y, no Z) | ✅ |
| `shatter_at_crossings` | cuts segments where they meet and opens a gap of `gap_d` there, so paths no longer touch; covers both a crossing and a T-junction, within a `touch_tol` given in model units. Input order decides which of a meeting pair yields; segments overlapping along a line are left uncut (**2D only**, z interpolated along the segment) | ✅ |

## Layout

```
src/
├── geomseq_core/                       # pure-numeric core, no Rhino dependency
│   ├── native/                         # C++ source + compiled binaries
│   │   ├── sort_curves.cpp
│   │   ├── sort_points.cpp             # single-point sibling of sort_curves (no direction/reversal)
│   │   ├── redistribute_arc_lengths.cpp    # arc-length density redistribution (pure 1D, no kd-tree)
│   │   ├── build_turn_waypoints.cpp    # smooth polygonal turn between two path segments' headings
│   │   ├── shatter_at_crossings.cpp    # contact predicates + broad phase + shatter_at_crossings
│   │   ├── nanoflann.hpp               # vendored kd-tree (BSD 2-Clause)
│   │   ├── archive/                    # superseded reference implementations (the windowed 2-opt, measured and removed)
│   │   └── geomseq_core.dll            # official .cpp files compiled into one binary (also .dylib / .so per platform)
│   ├── native_bridge.py                # ctypes loading + signatures (platform-aware)
│   ├── geometry_utils.py               # Python-facing wrappers (sort_curves_native, sort_points_native, ...)
│   ├── misc.py                         # coordinate <-> flat-buffer marshaling
│   └── _reload.py                      # dev-mode module unloading for GH hot-reload
├── rhino_utils/                        # depends on RhinoCommon; logic complex/reusable enough not to be a thin GH shell
│   ├── divide_curves.py                # curve -> division points + arc lengths
│   └── sample_curve_points.py          # arc lengths -> points on a curve
├── gh/                                 # Rhino-side verification: GHPython shells, hot-reloaded on edit (GH I/O only)
│   ├── definitions/                    # .gh example files
│   └── *_component.py
└── gha/                                # the shipped plug-in: C# GeomSeq.gha, P/Invoke into the same native library -- see src/gha/README.md

tests/                                  # property tests, plain CPython (no Rhino)
├── fixtures/                           # JSON inputs for the sort tests
└── test_*.py
```

## Build

Rebuild the native library after editing any `.cpp`. Every source compiles into
one shared library, and `native_bridge.py` checks that every function it expects
is there — a binary missing one fails at `load_dll()` with a clear error.

Run these from `src/geomseq_core/native/`:

```
# Windows (x64 Native Tools Command Prompt)
cl /std:c++17 /O2 /LD /EHsc /MT sort_curves.cpp sort_points.cpp redistribute_arc_lengths.cpp build_turn_waypoints.cpp shatter_at_crossings.cpp /Fe:geomseq_core.dll

# macOS
clang++ -std=c++17 -O2 -shared -fPIC -pthread -arch x86_64 -arch arm64 -o geomseq_core.dylib *.cpp
lipo -archs geomseq_core.dylib      # expect: x86_64 arm64

# Linux
g++ -std=c++17 -O2 -shared -fPIC -pthread -o geomseq_core.so *.cpp
```

On macOS both `-arch` flags are required: Rhino 8 runs natively on Apple
Silicon and on Intel, so a single-architecture build fails to load on the other.

The Python bridge picks the right binary for the platform. Only `.dll` and
`.dylib` are committed; Rhino does not run on Linux, so the `.so` is built only
in CI.

## Tests

Run on plain CPython — no Rhino needed. Needs a compiled binary for the current
platform (see Build):

```
pip install -r requirements-dev.txt
python -m pytest tests/
```

## CI

Every push rebuilds the native library from source and runs the tests on
Windows (MSVC), macOS (clang++, arm64) and Linux (g++). Linux is not a
deployment target; it is there as a third compiler, to catch code that only
happens to compile on the other two.

What CI does **not** check:

- **The committed `.dll` / `.dylib`.** CI builds its own, so a green run says
  nothing about whether the binaries in this repo are current. Rebuild and
  commit them whenever the C++ changes.
- **`rhino_utils/`, `gh/` and `gha/`.** They need Rhino; they are checked by
  hand.
- **Intel macOS.** GitHub no longer provides those runners; it is tested
  locally.

The reasons behind these choices are in
[docs/design-notes.md](docs/design-notes.md) (*CI and builds*).

## Watch out for

- **`redistribute_arc_lengths`'s `flat_pct` is a percent (0–100), not a
  fraction (0–1).** `100.0` keeps one density along the whole curve, `0.0`
  fades across all of it. Nothing checks the range, so `1.0` silently gives an
  almost fully graded curve instead of a uniform one.
- **`build_turn_waypoints` keeps each turn under `theta_max_deg` within each
  fillet, but not where the two fillets meet.** They are built independently,
  so with `step_len` large compared with the gap between the segments the
  junction can kink sharply (for example 62° at `theta_max_deg=30`). If you
  need a hard limit, check the gap before calling, or keep `step_len` well
  under it.

How fast each function is, and how it scales: [docs/benchmarks.md](docs/benchmarks.md).

## License

See [LICENSE](LICENSE). Vendors [nanoflann](https://github.com/jlblancoc/nanoflann)
(BSD 2-Clause) — see [THIRD_PARTY_LICENSES.md](THIRD_PARTY_LICENSES.md).