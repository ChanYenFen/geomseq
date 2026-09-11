# CLAUDE.md

Notes for whoever maintains this next, human or otherwise. This file holds the
*why* behind decisions that the code and the docs do not explain on their own.

- Results and what they mean: [docs/benchmarks.md](docs/benchmarks.md)
- How to run anything: [benchmarks/README.md](benchmarks/README.md)

## Benchmark design rationale

### Why benchmarks are not tests

Tests answer "is it correct"; benchmarks answer "what does it cost, and where".
Timings are machine- and load-dependent, so they make bad assertions — nothing
under `benchmarks/` runs in `pytest` and nothing there can fail a build. The
correctness suite stays fast (0.06 s) precisely because no large dataset is
allowed into `tests/fixtures/`.

### Why two harnesses

The Python harness measures what a caller waits for; the native harness
measures the algorithm. Their difference is the bridge, and for the
microsecond-scale functions that difference *is* the finding — `redistribute`
and `build_turn_waypoints` both turned out to be 97–98% bridge. Without the C++
harness the Python one is largely measuring itself, and real scaling behaviour
(`redistribute`'s clean 4.6 ns per output point) stays invisible under ctypes
overhead.

The native harness adds no timing entry points to the shipped DLL. It compiles
the same sources a second time into its own executable, so the production API is
never shaped by measurement.

### Why each function gets a different axis

Not one table copied four times — the parameter that drives cost differs:

| Function | Axis swept | Why |
|---|---|---|
| `sort_points` | n × 2-opt on/off × dataset | Sizes and `knn_k`/`max_passes` kept fixed across runs so tables stay comparable |
| `sort_curves` | n straddling 10,000, plus `if_flip` | 10,000 is `TWO_OPT_WINDOW_THRESHOLD`; `if_flip=False` makes the native side skip 2-opt entirely |
| `sort_curves_crossover` | both 2-opt paths forced at the same n | The auto dispatch never runs both at one n |
| `redistribute_lookups` | input n, output density, corner count | Pure 1D marching, no kd-tree — which axis dominates was an open question |
| `build_turn_waypoints` | `theta_max_deg` × turn geometry | One call is microseconds, below timer resolution, so it is timed in batches of 2,000 |

The sort groups carry a further `data` column naming the input distribution.
That axis is orthogonal to n: it varies what the input *looks like* rather than
how much of it there is, which is what makes the greedy phase's kd-tree
behaviour visible at all. Uniform scatter is the friendliest case a kd-tree can
get, so a sweep over n alone cannot tell you whether the algorithm degrades on
real geometry.

Tables report the **observed** `out_n`, measured outside the timed region,
because the declared knob and the actual work can diverge: `theta_max_deg` only
*caps* the per-waypoint turn, so a straight run produces 2 waypoints whether the
cap is 30° or 1°.

### Why `redistribute_lookups` sweeps input and output separately

The design guess was that output density dominates. The first run said input
size did, by a wide margin — which is what exposed that the wrapper was
marshaling an array the native side then ignored. Two independent sweeps are
what made that separable: one holds the band fixed so `out_n` stays constant
while input grows, the other holds input constant while the band moves `out_n`.
The observed `out_n` column is how you check each control actually held.

### Why fixtures are sampled, not sliced

A slice of a design is one *region* of it, so n=1,000 and n=64,000 would have
different characters and the scaling exponent would be meaningless. Seeded
sampling keeps the distribution — clustering included — comparable across n.
Sampling also preserves the file's own order, because input order is itself
under test; `random.sample` returns picks in random order, which would silently
turn `points_zigzag` into `points_grid` at every n below the file's length.
This is implemented and commented in `sample()` in `benchmarks/python/cases.py`.

### Why `uniform` stays the control

When a measurement moves, you need to be able to tell whether the algorithm
changed or the data did. Uniform scatter is the reference every other dataset is
read against, so it is never replaced — only added to. `points_uniform.json`
exists so the control is the same bytes on every machine rather than depending
on `random.Random`'s implementation, and `uniform` is a reserved label so a
fixture and the generator can never both claim it.

### Why the synthetic set varies two axes independently

A generator that changes spatial distribution and input order at once cannot
tell you which one moved the number. `points_grid` and `points_zigzag` share one
lattice and one seed and differ *only* in output order, so the time between them
is attributable to order alone; measuring `zigzag` on its own would confound
order with lattice structure. `points_zigzag_s10` (10% of positions permuted)
sits between them, so the three give a curve rather than two endpoints.

An already-sorted input is a deliberate measurement, not a mistake: it is the
lower bound, and it asks whether existing order is detected at all. It is only
misleading when unlabelled, and the `data` column names it.

The generator parameters and the density-contrast formula live in
`benchmarks/synth/make_fixtures.py`, and every generated file repeats its own
settings in a `params` block, so the datasets stay interpretable if the script
goes away.

### Why `two_opt_mode` exists

`sort_curves.cpp` used to pick its 2-opt path solely from a hardcoded
`TWO_OPT_WINDOW_THRESHOLD = 10000` with no override, so the two paths could
never be measured at the same n — the threshold was unfalsifiable. A
`two_opt_mode` parameter (0 = auto, 1 = exhaustive, 2 = windowed) makes the
comparison possible; it defaults to 0 in the Python wrapper, so ordinary callers
are unaffected. The crossover group must report **travel distance alongside
time**, because the windowed path buys speed with tour quality — a speed-only
table would make it look strictly better than it is, and the threshold is a
choice between the two.

### Why results are only quoted from committed runs

`docs/benchmarks.md` quotes nothing that is not backed by a file in
`benchmarks/results/`. Several earlier findings were recorded only in prose from
runs that were then overwritten, and when the committed baseline finally
arrived, two of them did not survive contact with it — the numbers disagreed and
one conclusion ("corner handling is superlinear") was simply wrong. Every result
file carries an environment block and a SHA-256 prefix of the binary; CI does
not validate that the committed binary is current, so the hash is the only thing
tying a number to a build.

## Grasshopper plugin (`src/gha/`)

### Why C# and P/Invoke, not Python script components

The install target is two files in `Grasshopper/Libraries`, with no Python
environment to set up. The Python layer (`native_bridge.py`, `misc.py`,
`geometry_utils.py`) stays anyway: it is what `pytest` runs, and pytest is the
only verification that exists outside Rhino. Only `src/gh/*_component.py` is
retired, once each C# component has been checked in Rhino.

### Why the library is loaded through a resolver

A plain `[DllImport]` probes Rhino's own directory and `PATH`, never the folder
the `.gha` sits in, so a correctly installed library is not found.
`NativeLibraryLoader` loads it from the plugin's folder and hands that handle to
every `DllImport` in the assembly. The folder comes from `Assembly.Location`,
which is empty when Grasshopper's "memory load" option reads the `.gha` as bytes.
The fallback for that case, `GH_AssemblyInfo.Location`, can only be exercised
inside Rhino and has not been yet.

The loader also checks for the `sort_curves` and `sort_points` exports, so a
stale binary gets one clear message instead of `EntryPointNotFoundException`
halfway through a solve.

### Why some component behaviour is not obvious

- **`D` is summed in C#.** The native side returns the travel segments, not
  their total, and the C++ is frozen for this work.
- **Points are read as `GH_Point`, not `Point3d`.** A null item in a `Point3d`
  list arrives as the origin, so a skipped point would silently become a real one.
- **The list inputs are `Optional`.** Otherwise Grasshopper emits its own
  missing-input warning and never calls `SolveInstance`, so the empty-input Remark
  could not happen.
- **The "tested limit" warnings are placeholders.** Sort Curves uses 50,000 (the
  README's ad-hoc ~43 s) and Sort Points uses 10,000 (unmeasured: its 2-opt has
  no windowed path). Neither is backed by `benchmarks/results/`, so replace both
  once the full baseline lands.

### Contracts

Component GUIDs are permanent, and new ports are appended, never inserted. A saved
`.gh` file finds a component by GUID and its wires by port index. The library GUID
in `GeomSeqInfo` is permanent for the same reason.

## Future directions

### Commit a full baseline

The highest-value next run. `sort_points`, `sort_curves` and the crossover group
currently have **no committed results**, so `docs/benchmarks.md` covers only two
of the four functions. The harness already covers them; it needs one
`--heavy` run recorded into `benchmarks/results/`.

### Open questions waiting on that run

- **`sort_curves` greedy jump between n=8,000 and n=12,000.** An earlier ad-hoc
  run showed ~3.7× the time for 1.5× the input, then only 1.24× from 12,000 to
  16,000. That may be the known O(n²) worst case from filtering used points out
  of a static kd-tree, or an artifact of the generated point set at that size.
  Needs a seed sweep before it means anything.
- **Direction-fixed mode appeared slower than the flip-enabled greedy path**,
  despite skipping 2-opt. Unexplained.
- **The windowed 2-opt quality claim.** `sort_curves.cpp` says K=500 holds the
  loss to ~2%; an ad-hoc run at n=12,000 gave 6.4% for only a 1.2× speedup.
  Either the input distribution or the comment is optimistic. Needs a seed sweep
  and a run at 50k.
- **`redistribute` at `out_n = 102`** is slower on the Python side than the
  `out_n = 367` row, while the native row is perfectly in line. Bridge-side,
  unexplained, worth one rerun before theorising.

### Real geometry worth recording

The synthetic set covers items 2 and 3 outright and approximates 1. Its value is
that every parameter is a dial; what it cannot do is surprise you. Real data is
what finds the assumption nobody thought to encode, so these stay open, with the
synthetic rows as the reference the real ones get read against. Each one exists
to break an assumption uniform scatter quietly makes — uniform density, uniform
scale, square extent, evenly spaced arc lengths.

1. **Clustered real toolpath** (`points_*`, `curves_*`) — the headline gap.
   `make_points` scatters uniformly over a square, the friendliest possible case
   for a kd-tree; real stitch and cut data is locally dense and globally sparse.
   Directly probes the degradation the repo README flags as unaddressed.
2. **Non-uniform density** (`points_*`) — a dense region and a sparse one in the
   same file (fill plus outline). Stresses the kd-tree differently from uniform
   clustering, and is what makes window expansion pathological rather than
   merely slower.
3. **Grid / hatch structured** (`points_*`) — infill produces collinear,
   equidistant points and exact ties in nearest-neighbour distance.
   Tie-breaking is untested at scale and does not occur in float-random data.
4. **Mixed curve lengths** (`curves_*`) — `make_segments` draws lengths from
   5–20. Real toolpaths mix long travel strokes with very short stitches, often
   an order of magnitude apart, which changes how much 2-opt has to gain.
5. **Long thin extent** (`points_*` or `curves_*`) — a border or a single row of
   lettering, far wider than tall. The generator always produces a square, and a
   square is where kd-tree splits behave best.
6. **Non-uniform arc-length lookups** (`lookups_*`) — `redistribute_lookups` is
   benchmarked only against perfectly even spacing. Real
   `rhino_utils/divide_curves.py` output on a varying-curvature curve is not
   even. Needs a `lookups_` kind added to the loader plus real corner indices.
7. **Real turn geometry** (`turns_*`) — `build_turn_waypoints` uses four
   hand-written cases copied from the tests. Real E→S pairs from a toolpath
   would cover the documented failure mode: a gap small relative to `step_len`
   where the exit→entry junction kinks past `theta_max_deg`.

Before committing any of these: the repository is public, and coordinates
exported from a client job are that client's design, recoverable from the
fixture. Prefer a pattern you own or one that is already public.

### Wrapper-side optimisation, if it ever matters

Both microsecond-scale functions are bridge-bound, and in both cases the fix is
Python-side, not C++: `build_turn_waypoints_native` does more work per call than
the algorithm it wraps, and `redistribute`'s corner cost is entirely
index-to-arc-length resolution plus ctypes array construction. Neither is a
bottleneck at embroidery-scale inputs, which is why neither has been done.
