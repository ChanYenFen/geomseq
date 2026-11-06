# Design notes

The *why* behind decisions that the code and the other docs do not explain on
their own: what was measured, what was tried and dropped, and what is still
open. Written for whoever maintains this next, human or otherwise.

This is the long version. The rules that fall out of it are in
[CLAUDE.md](../CLAUDE.md); results are in [benchmarks.md](benchmarks.md), and
how to run anything is in [benchmarks/README.md](../benchmarks/README.md).

Read a section when you are about to touch what it covers. Nothing here needs
reading front to back.

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
| `redistribute_arc_lengths` | input n, output density, corner count | Pure 1D marching, no kd-tree — which axis dominates was an open question |
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

### Why `redistribute_arc_lengths` sweeps input and output separately

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

### Why the greedy walk keeps live counts in the static tree

Once 2-opt was pruned, the greedy walk was most of both sort functions, and it
scaled worst: placed points stayed in the kd-tree, so a query near a used-up
region fetched them by the thousand only to skip them. The fix is the same kind
of pruning as 2-opt's: each node counts its unplaced points, and a subtree
whose count is zero is skipped, because it provably cannot hold the answer.

**Why not nanoflann's own removable trees.** The vendored nanoflann has two.
`KDTreeSingleIndexDynamicAdaptor` hides removed points only inside leaves, so it
still walks into subtrees that are empty. `KDTreeSingleIndexIncrementalAdaptor`
does skip dead subtrees, but it is a second tree (2-opt still needs one holding
every point), it stores one point per node behind pointers, and its different
shape would break ties differently. Counts beside the existing static tree keep
one tree, keep its leaf-of-10 layout, and walk it in nanoflann's own order —
which is what makes the output bit-identical rather than merely as good.

**Verified two ways.** Old and new DLLs were run side by side on 1,200
`sort_points` and 912 `sort_curves` configurations (every fixture, a 3D
lattice, heavy duplicates, n from 1 to 64,000, both start points, 2-opt on and
off, `knn_k` 12 and 1): zero differences. Ad hoc, not committed. The committed
guard is `test_greedy_step_*`, which checks every step against a brute-force
nearest; breaking the zero-count skip makes all of its cases fail.

`knn_k` now sizes nothing. It stays in both signatures so the ABI and every
caller are unchanged.

### Why shatter's broad phase is sweep and prune, not Bentley–Ottmann

`shatter_at_crossings` used to test every pair, and almost every test was
wasted: 0.15% of pairs at n=2,000 produced anything. The replacement only
changes *which* pairs reach `segment_contact_param` — boxes sorted along one
axis, an active list, a box-overlap test on the other — and leaves the
predicate, its arguments and the output stage alone.

**The pruning is the same kind as 2-opt's: provable, not tuned.** Every contact
the predicate can accept puts a point of one segment inside the other's box
widened by `max(touch_tol, EPS)`, the widening `on_segment` itself applies. The
boxes are widened by twice that plus a rounding margin scaled to coordinate
magnitude. Too wide costs a few predicate calls; too narrow would drop contacts
silently, so the margin errs one way only.

**Verified, not argued.** The output is not merely close to the pair loop's, it
is the same bytes: the higher index is always passed as `AB`, so each contact
gets the same `t`, and the caller sorts each segment's contacts anyway. An
ad-hoc run built both DLLs and compared them across 675 configurations —
random, polylines with owners and `test_self` both ways, hatching, exact
T-junction lattices, seven lines through one point exactly and 1e-7 apart,
zero-length and collinear segments, coordinates near 1e7, three gaps and three
tolerances — with zero mismatches. The committed guard is
`test_native_broad_phase_finds_every_contact_the_pair_loop_does`, and it was
checked against a deliberately broken pad: the lattice cases fail, the random
ones do not. Float-random input almost never touches exactly, which is why the
lattice cases are there.

**Why not Bentley–Ottmann**, which is output-sensitive at O((n+k) log n) and
the textbook answer. Its status structure must order segments strictly by their
height at the sweep position, and this function deliberately accepts contacts
within `touch_tol`, three or more lines through one point, and endpoints resting
on segments — each a place where that order is undefined, and each a separate
event-handling case to get right. Sweep and prune needs none of that, because
the exact question is still answered by the predicate it already trusts. B–O
would only win where many boxes overlap without their segments meeting — long
segments at shallow angles — and no such input has been measured.

**One mistake worth not repeating.** The first version collected every contact
in one list and sorted it. On a 4,000 × 4,000 cross-hatch, where the contact
count is itself quadratic, that sort made the broad phase *slower* than the pair
loop it replaced (ad hoc: 1,148 ms against 690 ms). A bucket per segment
brought it back to parity, which is the most any method can do when the output
is O(n²). Uniform scatter cannot reveal this, and the committed group still has
no such case.

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
- **The "tested limit" warnings are now measured.** Both components warn above
  16,000, which is the largest n in `baseline-windows-amd64-20260914-heavy` and
  roughly where 2-opt stops feeling instant (~12 s). They started as
  placeholders — 50,000 from an ad-hoc run, 10,000 borrowed from a threshold
  that measured something else entirely — which is worth remembering before
  quoting any other number that has not been re-checked against a committed run.

### Contracts

Component GUIDs are permanent, and new ports are appended, never inserted. A saved
`.gh` file finds a component by GUID and its wires by port index. The library GUID
in `GeomSeqInfo` is permanent for the same reason.

## CI and builds

Moved from the README, which now carries only the short version.

### Runners

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

- **`rhino_utils/`, `gh/` and `gha/`** — they need RhinoCommon, which no runner has. Verified by hand in Rhino.
- **Intel macOS** — GitHub no longer allocates those runners (jobs queue until the 24h limit, then cancel). Built and tested locally on an Intel Mac instead.
- **The committed `.dll` / `.dylib`** — CI compiles its own, so a green badge says nothing about whether the binaries in this repo are current, or built for the right architecture. Rebuild and re-commit them whenever the C++ changes.
- **Undefined behaviour all three toolchains happen to tolerate** — passing on three is evidence of portability, not proof of it.

CI also builds macOS for the host architecture only; the universal binary in
Build is for distribution, and only its native half could be executed anywhere.

### The macOS universal binary

Both `-arch` flags are required on macOS. Without them clang builds only the
host architecture, and the result fails to load in Rhino 8 on the other one —
it runs natively on Apple Silicon, not under Rosetta. Either Mac can build the
universal binary; only the host architecture can be tested locally, and CI's
arm64 runner covers the other half.

### What the tests assert

The sort tests assert the result is a permutation of the input and that travel
distance never exceeds the input order's, and, with 2-opt off, that every greedy
step takes a nearest allowed point (checked by brute force). The redistribute test asserts the
output spans the full arc length, increases strictly, reproduces every corner
exactly, and keeps step sizes within `[low, high]` — except the final step onto
`total_length` and steps rescaled to land on a corner, which the native side
breaks the band for by design. The turn test asserts both ends stay pinned to
their extend points and that no waypoint turns by more than `theta_max_deg`
(see *Watch out for* in the README).

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
- **Most groups are `uniform`, one seed.** The conclusions above are
  directional and the effect sizes are large, but no seed sweep has been run.
  The sort groups are the exception, and this bullet said otherwise until
  2026-11-02: `baseline-windows-amd64-20260915-heavy` runs `sort_points` on all
  seven point fixtures (grid included) and `sort_curves` on clustered and
  zigzag, at every size. The hardest clustering costs 10–20% over uniform.
  Shatter, redistribute and the turn builder are still uniform-only.
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
  **The cap was raised to 20 on the strength of this**, not for speed but
  because convergence is what makes pruning's one weakness structural rather
  than empirical: it trails the exhaustive pass only while unconverged, so a cap
  that always reaches convergence removes the failure mode instead of clearing
  it by a margin that happened to hold on the shapes measured.
- **Two numbers spell "max passes", and the split is deliberate.** Every shipped
  caller is 20 — `sort_curves_native`, `sort_points_native`, both Grasshopper
  components, both GHPython shells — because 20 is where every shape measured
  converges, and a pruned search is only ever the worse of the two
  implementations while it is still unconverged. The cap is there to guarantee
  the search finishes, not to ration time; it costs milliseconds.
  `benchmarks/python/cases.py` stays at 10 and is the one place that must not
  follow: it is the constant every recorded baseline was taken with, so moving
  it would silently make the sort groups incomparable with their own history.
  Points arrived at 20 later than curves, and the reason is worth keeping. While
  `sort_points` still ran the exhaustive pass, doubling its cap would have
  doubled a 248.68 s solve on no measurement at all — a real objection, and it
  expired with the implementation on 2026-09-15 rather than losing an argument.
- **`sort_points` never got a windowed path, and no longer needs one.** It was
  the function that could not take a large input — cleanly O(n²), 2.20 s at
  n=8,000 rising to 248.68 s at 64,000 — and the lever looked like `max_passes`
  or a different algorithm. It turned out to be neither: the pruning rule ported
  from `sort_curves` unchanged, because the move is the same shape, and 64,000
  points went from 248.68 s to 1.50 s with the tour 0.94–1.31% *shorter* across
  six seed-paired rows. The greedy phase then became the expensive half of
  both functions, and on 2026-11-06 got its own pruning (see *Why the greedy
  walk keeps live counts*): 64,000 points now take 588.9 ms in all, 76.0 ms of
  it greedy. **2-opt is the expensive part again**, at 77–87% of the call.

- **`shatter_at_crossings` is now bridge-bound, probably.** With the broad
  phase the Python call at n=16,000 is 42.3 ms
  (`baseline-windows-amd64-20261102-shatter-sweep-heavy`), and an ad-hoc native
  timing put the C++ at about a tenth of that. The group has no native-harness
  row because the bridge used to be under 0.1%; that reason expired with the
  pair loop. Add the row before optimising either side.

### Clustered sequencing for layered and zoned fabrication

A separate algorithm, deliberately not folded into the sort components. They
answer "order these n items from a start point". This one answers "order the
groups, and order within each, without interleaving them" — a hierarchical tour,
with group structure as input and the jumps *between* groups in the objective.
Layer order, colour changes and tool changes all live here rather than there.

`Continuous` on Sort Points and Sort Curves is not this. It does one small,
predictable thing — start each branch where the last one ended — at fixed cost,
and claims nothing about optimising the jumps. That is worth keeping separate
from an algorithm whose cost is still unknown.

**What was measured**, on 30 real points in 3 spatially separated branches, via
the Python bridge. Not committed to `benchmarks/results/`: it was exploratory,
one dataset, and nothing here should be quoted as a baseline.

| | travel | inter-branch jumps |
|---|---|---|
| flattened, one sort | 190.866 | — |
| chained, today's single seed | 195.654 | 10.0 / 26.2 / 12.9 |
| chained, best seed per branch (ceiling) | 189.884 | 16.5 / 15.5 / 13.8 |

The mechanism is visible in the jumps. Sorting a branch from the cursor leaves it
ending wherever 2-opt finished, with no knowledge of where the next branch is;
choosing a different *start* steers where it ends. The ceiling row enters branch 0
further away on purpose (10.0 → 16.5) to finish on its right edge, turning a 26.2
jump into 15.5.

**Three things that were wrong, recorded so they are not repeated.**

*"Grouping can only cost, never gain."* It gained: the ceiling beats the
flattened sort. Fewer constraints would guarantee a better *optimum*, but
flattening is itself only greedy + 2-opt, and the constrained sub-problems get
solved more cleanly than the whole.

*Seeding by distance to the next group.* Candidates picked as "the branch point
nearest the next branch" and "the farthest" lost in all three branches — the best
seed was never among them. The reason is in the numbers: intra-branch length
barely moves across seeds (47.7–49.8, under 4%), so the score is almost entirely
entry + exit, and those heuristics optimise exit while ignoring entry. Two and
three such candidates gained exactly nothing. Four candidates ranked by
entry + exit reached the ceiling on this dataset, in 12 sorts rather than 30.

*Comparing against a flatten that used a different start.* The first flattened
figure came from an unconnected `S`, which falls back to the first input point
(186.979), not from `S = origin` (190.866). Two runs, two start points, one
meaningless comparison.

**The open question, which decides whether this is viable at all.** Cost is the
candidate count times the sorts, so it matters enormously whether that count
stays bounded as branches grow. A first probe — 3 branches at 10/20/40/80 points,
two seeds each — put the fraction of points needed to reach the ceiling anywhere
from 0.07 to 1.00 with no trend. Two samples per size, and a brittle measure
(reaching the ceiling is all-or-nothing, so it jumps from 6 to 41 without the
problem getting harder). Inconclusive, and the right next measurement is not
"how many candidates reach the ceiling" but "how much of the gap does a fixed k
recover" — a continuous quantity, across several layouts and seeds.

Until that is answered, nothing should ship: a fixed k costs a constant factor
that pruning's 45–113× can absorb, while a k that grows with branch size would
undo it entirely. And it needs its own benchmark group before it ships, like
everything else here.

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
6. **Non-uniform arc lengths** (`arc lengths_*`) — `redistribute_arc_lengths` is
   benchmarked only against perfectly even spacing. Real
   `rhino_utils/divide_curves.py` output on a varying-curvature curve is not
   even. Needs a `arc lengths_` kind added to the loader plus real corner indices.
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
