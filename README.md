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

Run these from `src/geomseq_core/native/`:

```
# Windows (x64 Native Tools Command Prompt)
cl /std:c++17 /O2 /LD /EHsc /MT sort_curves.cpp sort_points.cpp redistribute_lookups.cpp build_turn_waypoints.cpp /Fe:geomseq_core.dll

# macOS
clang++ -std=c++17 -O2 -shared -fPIC -pthread -arch x86_64 -arch arm64 -o geomseq_core.dylib *.cpp
lipo -archs geomseq_core.dylib      # expect: x86_64 arm64

# Linux
g++ -std=c++17 -O2 -shared -fPIC -pthread -o geomseq_core.so *.cpp
```

Both `-arch` flags are required on macOS. Without them clang builds only the
host architecture, and the result fails to load in Rhino 8 on the other one —
it runs natively on Apple Silicon, not under Rosetta. Either Mac can build the
universal binary; only the host architecture can be tested locally, and CI's
arm64 runner covers the other half.

The Python bridge auto-selects the right binary (`.dll` / `.dylib` / `.so`) by
platform. Only `.dll` and `.dylib` are committed — Rhino does not run on Linux,
so the `.so` exists solely as an ephemeral CI build.

## Tests

Property tests, run on plain CPython — no Rhino, since the core only ever sees
coordinate arrays. Needs a compiled binary for the current platform (see Build):

```
pip install -r requirements-dev.txt
python -m pytest tests/
```

Every fixture case is its own parametrized test (17 in total). The sort tests assert the
result is a permutation of the input and that travel distance never exceeds the
input order's. The redistribute test asserts the output spans the full arc
length, increases strictly, reproduces every corner exactly, and keeps step sizes
within `[low, high]` — except the final step onto `total_length` and steps
rescaled to land on a corner, which the native side breaks the band for by
design. The turn test asserts both ends stay pinned to their extend points and
that no waypoint turns by more than `theta_max_deg` (see Notes).

## CI

Every push rebuilds the native library from source and runs the whole suite under
`pytest` on three runners:

| Runner | Toolchain | Role |
|--------|-----------|------|
| `windows-latest` | MSVC | deployment target |
| `macos-latest` | clang++ (arm64) | deployment target |
| `ubuntu-latest` | g++ | portability guard |

`fail-fast` is off, so one platform breaking still reports the other two.

### Why Linux, when Rhino cannot run there

The `.so` it produces is thrown away — nobody ships it. The job earns its place
as a *third toolchain*: a different standard library catches code that merely
happens to compile under the other two. It already has. Vendored nanoflann uses
`std::thread` and `std::mutex` without including `<mutex>` or `<thread>`, leaning
on transitive includes that MSVC and libc++ provide but other implementations do
not. Two platforms would never have surfaced that.

It is also the cheapest guard available: ~10s per run, against ~28s for Windows.

### What CI does not cover

- **`rhino_utils/` and `gh/`** — they need RhinoCommon, which no runner has. Verified by hand in Rhino.
- **Intel macOS** — GitHub no longer allocates those runners (jobs queue until the 24h limit, then cancel). Built and tested locally on an Intel Mac instead.
- **The committed `.dll` / `.dylib`** — CI compiles its own, so a green badge says nothing about whether the binaries in this repo are current, or built for the right architecture. Rebuild and re-commit them whenever the C++ changes.
- **Undefined behaviour all three toolchains happen to tolerate** — passing on three is evidence of portability, not proof of it.

CI also builds macOS for the host architecture only; the universal binary in
Build is for distribution, and only its native half could be executed anywhere.

## Notes

- The C++ core is CAD-independent: it takes and returns plain coordinate arrays,
  so it is testable without Rhino (see Tests) and reusable from any front end.
- `redistribute_lookups`'s `flat_pct` is a **percent (0–100), not a 0–1
  fraction** — `100.0` holds one density for the whole curve, `0.0` fades across
  its entire length. Nothing validates the range, so `1.0` silently gives an
  almost fully graded curve rather than a uniform one.
- `build_turn_waypoints` caps the per-waypoint turn at `theta_max_deg` **within
  each fillet, but not across the exit→entry junction** — the two fillets are
  built independently and their endpoints need not meet along the chord. With
  `step_len` large relative to the E–S gap the junction can kink sharply (e.g.
  62° at `theta_max_deg=30`). Callers needing a hard cap should check the gap
  before calling, or keep `step_len` well under it.
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