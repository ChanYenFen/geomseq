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
| `sort_curves` | n straddling 10,000, plus `if_flip` | 10,000 was where the 2-opt implementation used to switch; the sizes are kept so the rows stay comparable with baselines recorded while it did. `if_flip=False` makes the native side skip 2-opt entirely |
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

### Why the windowed 2-opt was removed

`sort_curves.cpp` used to carry two 2-opt implementations: the exhaustive O(n²)
pass, and a windowed one that tested only the `WINDOW_K = 500` spatially nearest
candidate edges per edge, via a second kd-tree over tour-edge midpoints. It
switched between them on a hardcoded `TWO_OPT_WINDOW_THRESHOLD = 10000`, with no
override, which also meant the two could never be measured at the same n — the
threshold was unfalsifiable. A `two_opt_mode` parameter (0 = auto, 1 = forced
exhaustive, 2 = forced windowed) was added to make the comparison possible, and
that comparison is what retired the whole thing.

**What was measured.** All from `benchmarks/results/`, all uniform data, one
seed. Windowed's tour against exhaustive's at the same n, both at 10 passes:
+6.4% at n=12,000, +12.9% at 25,000, +18.6% at 50,000, for speedups of 1.21×,
2.59× and 4.74×. The shipped comment claimed K=500 held the loss to ~2%; it was
wrong at every size measured, and wrong by an order of magnitude at 50,000, the
size it was written for.

**Two candidate explanations, both tested.**

The first was that the comparison was unfair: `max_passes` is a cap, not a
count, so "10 passes each" might mean different things. It did.
`sort_curves_passes` showed exhaustive converging on its own near pass 13-16
while windowed was still gaining 4.4% between passes 10 and 20 — it was being
cut off less than half way. But equalising does not rescue it. Compared by wall
time instead of by passes, exhaustive still dominates: at n=50,000 windowed
needs 20 passes and 77 s to reach a tour exhaustive reaches in **one** pass and
19 s. Twenty times the passes to match one, at five times cheaper per pass, is a
4× net loss.

The second was density, and it is the more interesting one. Every sweep packed
more segments into the same 1000×1000 square, so n and density rose together —
and windowed's window is the K *nearest* edges, whose physical reach shrinks as
density rises. `sort_curves_density` grew the extent as √n instead, holding
segments per unit area fixed (also the realistic case: more stitches usually
means a bigger design, not a tighter one). Density explained about a third of
the degradation at 50,000 — 18.6% became 13.3% — and no more. The gap still
roughly doubles from 12,000 to 50,000 with density held constant.

**The structural reason, which is why no tuning was attempted.** K is fixed
while n is not, so K's *coverage* falls as the problem grows: 500 candidates out
of 2,000 edges is 25%, out of 50,000 it is 1%. Tour quality tracks that fraction
almost monotonically across all six sizes measured. Degrading with scale is
therefore the design, not a mis-set constant.

**And an inference, not a measurement:** widening K would buy quality back only
by giving up the speed that was the entire point. Each pass does one k-NN query
per edge, so cost grows with K — at n=50,000 windowed already tests 50× fewer
pairs than exhaustive while running only 5× faster, meaning query overhead
already dominates. A K large enough to matter would plausibly make it slower
than exhaustive *and* worse. That was not measured, because the outcome it
predicts is a path that is dominated either way.

**What was kept.** The removal cost nothing in capability: the fast end belongs
to `use_two_opt=False`, which answers in 1.37 s at n=50,000 where windowed's
first pass takes 4.95 s, and the quality end belongs to exhaustive. Windowed
owned only a narrow band between them, reachable only by a caller who already
knows both their tour size and their time budget — which a shipped plug-in
cannot ask of anyone. The code is in
`native/archive/sort_curves_v2_with_windowed_2opt.cpp`; the groups that judged
it (`sort_curves_crossover`, `_density`, `_passes`) have been retired from
`cases.py`, and their results remain in `benchmarks/results/`.

**The defect underneath all of this**, worth naming separately from the numbers:
a constant in the source was choosing speed over quality on the caller's behalf,
in a range where the caller could neither see the choice nor decline it. That
would have been wrong even if windowed had been good.

### Why the 2-opt search is pruned, and why that is not windowing again

Having just removed one scheme that skipped candidate pairs, the obvious
question about the one that replaced the exhaustive scan is what makes it
different. The answer is that this one skips only pairs it can *prove* are not
improving.

A 2-opt move removes the gaps after positions i and j and reconnects them, so it
pays only when the two new edges are together shorter than the two old ones. If
both new edges were longer than the old edge each is measured against, their sum
would be too. So every improving move satisfies

    d(exit_i, exit_j) < gap_i    OR    d(entry_i+1, entry_j+1) < gap_j

and each half is a ball query the endpoint kd-tree — already built for the
greedy phase — can answer. Two scans are needed rather than one, because the two
radii belong to opposite ends of the move and neither alone is complete.

**The difference from windowing is where the number comes from.** `WINDOW_K =
500` was a constant, so its coverage fell from 25% of candidates to 1% as n
grew, and the moves it dropped were real ones. Here there is no constant: the
radius is the tour's own gap length. Nothing is dropped that could have helped,
and the search narrows only as the gaps shorten — which is the objective, not a
sacrifice.

**Verified, not argued.** A tour the pass has *converged* on must contain no
improving move at all, which an exhaustive O(n²) scan can check directly. Across
uniform, clustered and zigzag inputs at n = 500, 4,000 and 16,000, the scan
finds zero remaining in all nine tours. The check must raise the pass cap high
enough that the loop ends on convergence — a tour cut off at `max_passes`
legitimately still has improving moves and would fail for the wrong reason.

**What it costs and buys.** At n=16,000, 10 passes: 17.15 s to 380 ms on uniform
(45×), 17.06 s to 400 ms on clustered (43×), and 26× on already-sorted zigzag.
The ratio grows with passes — 8× at one pass, 45× at ten, 54× at twenty —
because the radii tighten as the tour improves while exhaustive pays the same
O(n²) every pass. Tour quality at the shipped cap came out slightly *better*
(0.56% and 0.80% shorter), which is luck of a different search order, not a
claim of the method.

**Two things it does not preserve, both worth knowing before quoting old
numbers.** The tour is not the same: both versions take the first improving move
they meet and the kd-tree meets them in a different order, so the two walk to
different local optima and no recorded travel figure reproduces to the digit.
And low pass counts got worse, not better — 7.7% worse at one pass, 3.1% at
three, 0.6% at five — because each position now applies at most one move per
scan, so a pass pushes less far while costing far less.

### Why results are only quoted from committed runs

`docs/benchmarks.md` quotes nothing that is not backed by a file in
`benchmarks/results/`. Several earlier findings were recorded only in prose from
runs that were then overwritten, and when the committed baseline finally
arrived, two of them did not survive contact with it — the numbers disagreed and
one conclusion ("corner handling is superlinear") was simply wrong. Every result
file carries an environment block and a SHA-256 prefix of the binary; CI does
not validate that the committed binary is current, so the hash is the only thing
tying a number to a build.

Read that hash as a fingerprint of one file, not of one source revision. MSVC
stamps a build timestamp into the binary, so recompiling identical sources gives
a different hash — measured, not assumed: the pruning work built the same
`sort_curves.cpp` twice and got `657faa5468b2434e` and `b69a189822a4c2ab`. The
hash can therefore tell you two result files ran against different binaries, but
never that a binary matches the source it came from. Only the committed DLL
itself does that, and only if it is committed alongside the results.

## Future directions

### Write up the sort baseline

`baseline-windows-amd64-20260914-heavy` is committed and covers all four
functions, but `docs/benchmarks.md` still has no write-up of the two sort
groups. The numbers exist; the reading of them does not.

### What the first full baseline settled

- **The `sort_curves` greedy jump between n=8,000 and 12,000 did not
  reproduce.** The ad-hoc run that showed ~3.7× the time for 1.5× the input gave
  1.9×, then 1.18× on to 16,000. Treat the original as an artifact.
- **Direction-fixed mode is not slower.** `if_flip=False` beat the flip-enabled
  greedy path at every n measured (153.4 ms against 218.8 ms at 16,000), which
  is what intuition predicted all along.
- **The windowed 2-opt quality claim was refuted**, and the implementation
  removed — see "Why the windowed 2-opt was removed" above.

### Still open

- **`redistribute` at `out_n = 102`** is slower on the Python side than the
  `out_n = 367` row, while the native row is perfectly in line. Bridge-side,
  unexplained, worth one rerun before theorising.
- **Almost everything measured is `uniform`, one seed.** The conclusions above
  are directional and the effect sizes are large, but no seed sweep has been
  run. The one exception is `sort_curves_convergence`, which covers clustered
  and zigzag at n=16,000; the `grid` fixtures and every other group are still
  uniform-only.
- **`max_passes = 10` is enough for every shape, and is no longer a speed lever.**
  The worry was that some structure converged more slowly than uniform and was
  being cut off unseen. `sort_curves_convergence` says not: against each shape's
  own 20-pass tour, 10 passes gives up 0.04% on uniform and 0.01% on
  clustered_100x, while zigzag arrives already 2-opt-optimal. Clustered
  converges *faster* than uniform in relative terms — the opposite of the
  concern. **But the trade-off half of this bullet was invalidated within hours
  of being written**, and that is the part worth remembering: it said cutting to
  5 passes cost 1.11% for half the runtime, measured against the exhaustive
  implementation that pruning then replaced. On the pruned search the same cut
  costs 0.6%, three passes costs 3.1%, one pass costs 7.7%, and none of it buys
  much, because a pass is now 45× cheaper and the whole sweep finishes in under
  half a second. Tuning `max_passes` for speed is no longer a question worth
  asking at n=16,000.
- **The cap does not get shorter as n grows**, which was the live worry once
  pruning landed: pruned 2-opt trails until it converges, so a cap that is
  generous at 16,000 and short at 50,000 would hand large jobs an unconverged
  tour with nobody able to see it. Measured instead of assumed, in
  `...-convergence-pruned-extended-heavy`. Every shape at 16,000 and generated
  uniform at 50,000 converge by **20** passes, with 30 and 50 not moving a
  digit. The shortfall at the shipped cap of 10 is 0.14% at 16,000 and 0.147% at
  50,000 — the same, not worse. Going to 20 costs 64 ms at 16,000 and 140 ms at
  50,000.
  Two limits on that: the curve fixtures hold 16,000 rows, so 50,000 could only
  come from the generator and is therefore uniform — how a clustered 50,000-curve
  job converges is still unmeasured. And most 50,000 rows ran once, so their
  timings carry noise; the travel figures are deterministic and do not.
  **Raising the cap to 20 is the recommendation this supports**, not for speed
  but because convergence is what makes pruning's one weakness structural rather
  than empirical: it trails the exhaustive pass only while unconverged, so a cap
  that always reaches convergence removes the failure mode instead of clearing
  it by a margin that happened to hold on the shapes measured.
- **`sort_points` has no windowed path and now never will get this one.** Its
  2-opt is cleanly O(n²) — 2.20 s at n=8,000 rising to 248.68 s at 64,000 — so
  if large point sets ever matter, the lever is `max_passes`, or a different
  algorithm, not the one just removed.

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
