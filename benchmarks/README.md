# benchmarks

Timing harnesses for the core functions. Separate from `tests/` on purpose:
timings make bad assertions (they are machine- and load-dependent), so nothing
here runs under `pytest` and nothing here fails a build.

```
python/cases.py   representative inputs per function -- what counts as a size
python/run.py     timing harness + runner -- how a size gets measured
native/           bench_core.cpp -- pure C++ timing, no Python involved
compare.py        aligns the two JSON baselines into one table
fixtures/         input datasets, one per file; picked up automatically
synth/            one-shot generator for the synthetic fixtures; nothing imports it
gh/               Grasshopper-side export of real geometry into fixtures/
results/          recorded baselines + the comparison
```

Both harnesses emit JSON; `compare.py` is the only thing that renders a
native-vs-Python table, so the format lives in one place.

## Two harnesses, two questions

They are complements, not alternatives, and neither one's numbers substitute
for the other's:

| | measures | answers |
|---|---|---|
| `python/` | wrapper + ctypes + marshaling + native | what a caller actually waits for |
| `native/` | the algorithm alone | what the C++ costs |

The difference between them is the bridge overhead. That subtraction is the
whole reason the native harness exists: for the microsecond-scale functions the
Python harness largely measures itself. `build_turn_waypoints` reports ~5.8 µs
per call for a straight turn that produces 2 points — the native harness puts
the same call at **0.145 µs**, so ~97% of the Python figure was wrapper and
ctypes, not arithmetic.

Both harnesses use identical inputs and report the same `out_n`, so rows can be
read side by side. For these small deterministic cases the inputs are simply
written out in both files rather than shared through fixtures; fixture files
only become necessary if a larger dataset ever needs exact parity.

`native/` deliberately adds no timing entry points to the shipped DLL. It
compiles the same `.cpp` sources a second time into its own executable, so the
production API is untouched.

## Running the Python harness

```
python benchmarks/python/run.py                 # default set, all groups
python benchmarks/python/run.py sort_points     # one function
python benchmarks/python/run.py --heavy         # add the multi-minute 2-opt cases
```

Needs a compiled `geomseq_core` binary for the current platform, same as the
tests — see the repo README's Build section. Results land in
`results/baseline-<sys>-<machine>-<date>`; a partial run tags the filename with
the groups it covered so it cannot overwrite a full baseline.

Default runtime is a few minutes — `sort_points` alone is ~80 s, because every
fixture in `fixtures/` adds a parallel sweep (7 point datasets × 7 sizes × 2-opt
on/off). Deleting fixtures shrinks the run; see
[../docs/benchmark-fixtures.md](../docs/benchmark-fixtures.md).
`--heavy` adds `sort_points` 2-opt at
n = 16k/32k/64k and `sort_curves` at 16k, which together run into several
minutes; the published figures in [../docs/benchmarks.md](../docs/benchmarks.md)
put 64k with 2-opt alone at ~213 s.

## Running the native harness

Covers `build_turn_waypoints` and `redistribute_lookups` only — the two whose
native cost the Python harness cannot resolve. The sort functions are already
96–97% native at scale, so their Python numbers are within a few percent of the
truth and are not duplicated here.

From `benchmarks/native/`:

```
# Windows (x64 Native Tools Command Prompt)
cl /std:c++17 /O2 /EHsc /MT bench_core.cpp ..\..\src\geomseq_core\native\redistribute_lookups.cpp ..\..\src\geomseq_core\native\build_turn_waypoints.cpp /Fe:bench_core.exe

# macOS / Linux
c++ -std=c++17 -O2 -o bench_core bench_core.cpp ../../src/geomseq_core/native/redistribute_lookups.cpp ../../src/geomseq_core/native/build_turn_waypoints.cpp
```

It prints JSON on stdout; redirect it to keep a record:

```
./bench_core > ../results/native.json
```

The binary is gitignored — build it on demand. Results carry the compiler
version, which is what will make a same-machine compiler comparison possible
later; that is not attempted yet.

## Comparing the two

```
python benchmarks/python/run.py build_turn_waypoints redistribute_lookups --out benchmarks/results/python
benchmarks/native/bench_core > benchmarks/results/native.json
python benchmarks/compare.py benchmarks/results/native.json benchmarks/results/python.json
```

Prints a per-case table of `native` / `python` / `bridge` (the difference), so
each function's cost can be read stage by stage. Redirect it into
`results/comparison.md` to keep it.

Cases are matched on the fields both harnesses share — `geometry` +
`theta_max_deg` for turns, `band` + `corners` + `mode` for redistribute.
`input_n` exists only on the Python side (since the ABI change the native call
never receives the input array), so one native row is compared against each
Python row that varies it, and the table says so. Any case the native harness
does not cover shows `--` rather than being dropped.

## Method

Warm up once (the first call absorbs DLL load and reads ~10× high), then report
the **minimum** of up to 5 runs — scheduling noise only ever adds time, so the
minimum is the least noisy estimate of the work itself. A case that has already
burned 2 s stops repeating, so slow cases are not run five times.

Input generation is outside the timed region. Generators are seeded, so a rerun
on the same machine is comparable.

Two things the numbers include that are worth naming, because they are not the
C++ core: **ctypes marshaling** and, for `sort_curves`, the wrapper's Python-side
`apply_order` (which calls `Duplicate`/`Reverse` per curve). This matches what
`docs/benchmarks.md` already measured — it times the wrapper, not the raw DLL
symbol — but it means a slow case is not automatically a slow algorithm.

Every result file carries an environment block including a SHA-256 prefix of the
binary the numbers came from. That is deliberate: `docs/benchmarks.md`'s own rule
is to quote ratios and exponents rather than milliseconds unless the environment
is stated, and CI never validates that the committed binary is current.

## Why each function gets a different axis

Not one table copied four times — the parameter that drives cost differs:

| Function | Axis swept | Why |
|---|---|---|
| `sort_points` | n × 2-opt on/off × dataset | Same sizes and `knn_k`/`max_passes` as `docs/benchmarks.md`, so the columns line up against what is already published |
| `sort_curves` | n straddling 10,000, plus `if_flip` | 10,000 is `TWO_OPT_WINDOW_THRESHOLD`; `if_flip=False` makes the native side skip 2-opt entirely |
| `sort_curves_crossover` | both 2-opt paths forced at the same n | The auto dispatch never runs both at one n; see the crossover section below |
| `redistribute_lookups` | input n, output density, corner count | Pure 1D marching, no kd-tree — which axis dominates was an open question, and the answer was surprising (below) |
| `build_turn_waypoints` | `theta_max_deg` × turn geometry | One call builds one turn in microseconds, below timer resolution, so it is timed in batches of 2,000 |

The sort groups carry a further `data` column naming the input distribution —
`uniform` plus whatever sits in `fixtures/`. That axis is orthogonal to n: it
varies what the input *looks like* rather than how much of it there is, which is
what makes the greedy phase's kd-tree behaviour visible at all. Uniform scatter
is the friendliest case a kd-tree can get, so a sweep over n alone cannot tell
you whether the algorithm degrades on real geometry.

`out_n` columns are the **observed** output size, measured once outside the timed
region. It is there because the declared knob and the actual work can diverge:
`theta_max_deg` only *caps* the per-waypoint turn, so a straight run produces 2
waypoints whether the cap is 30° or 1°.

## What the first baseline turned up

Measured on Windows/AMD64, MSVC, 2026-09-03. These are observations to chase,
not conclusions — each needs isolating before it means anything.

The pre-ABI-change figures quoted below are from a baseline run that was
overwritten before it was committed, so they survive only here. Treat them as
reported, not reproducible; everything after the change is in `results/`.

**`redistribute_lookups` was dominated by input size, not output size** — the
opposite of the design guess, and the reason both axes are swept. Input n over a
1000× range (101 → 100,001) moved the time 113× (17.8 µs → 2.1 ms); output count
over a 36× range (102 → 3,657) moved it 1.6× (178 → 276 µs). The two sweeps
isolate the variables: the first holds the band fixed so `out_n` stays 367 while
input grows, the second holds input at 10,001 while the band moves `out_n`. The
observed `out_n` column is what confirms each control actually held.

Timing the raw DLL symbol against the wrapper found the cause: **the C++ was
1–2% of the wall time**, and the rest was `misc.floats_to_buffer` marshaling an
array the native side then ignored — it only ever read `lookups[n-1]` and the
corner entries. **Fixed** (2026-09-03) by passing `total_length` and resolved
corner arc lengths instead of the whole array; the Python-facing signature did
not change. Before/after on the same harness:

| input_n | before | after | |
|---|---|---|---|
| 101 | 17.8 µs | 13.3 µs | 1.3× |
| 1,001 | 34.4 µs | 14.2 µs | 2.4× |
| 10,001 | 197.9 µs | 16.6 µs | 12× |
| 100,001 | 2,100 µs | 39.7 µs | **53×** |

Scaling over that 990× input range went from 118× to 3.0× (≈ n^0.69 → n^0.16).
The residual is **not** per-element work — with the list already built and warm,
the call is flat at ~12.9 µs across all four sizes. It is allocator and cache
pressure from having just built a large list, which is why a freshly generated
input still costs a little more at 100k.

With the input cost gone, the output count is now the visible axis, as originally
guessed: `out_n` 102 → 3,657 moves the call 7.9 → 107.3 µs (13.6×, was 1.6×).
The return path (`out[:count]`, ~90 ns/element) is now the largest single share.

**Corner handling is superlinear in corner count** and got relatively worse:
1,000 corners costs 347.8 µs against 46.5 µs at zero. Most of that is not the
C++ (a flat ~8 µs) but building the ctypes array by star-unpacking —
`(ctypes.c_double * 1000)(*values)` alone measures ~103 µs. Realistic corner
counts are polyline vertices (single digits), so this is noted, not urgent.

**Corner handling is superlinear in corner count.** 0 → 100 corners costs ~22%
(214 → 261 µs), but 100 → 1,000 nearly doubles it (261 → 491 µs).

**`sort_curves` greedy has a jump between n=8,000 and n=12,000** — 53 ms → 197 ms,
3.7× for 1.5× the input, while 12,000 → 16,000 is only 1.24× (197 → 245 ms). The
README notes greedy's unaddressed O(n²) worst case from filtering used points out
of a static kd-tree. This may be that, or may be an artifact of the generated
point set at that size; it needs a seed sweep before it is worth believing.

**Direction-fixed mode is slower than the flip-enabled greedy path**, even though
it skips 2-opt: 82 ms vs 53 ms at n=8,000.

**`build_turn_waypoints` is almost entirely wrapper overhead.** The Python
harness showed it as call-overhead-bound; the native harness says how much:

| case | out_n | native | Python | native share |
|---|---|---|---|---|
| straight, θ=30° | 2 | 0.145 µs | 5.8 µs | 2.5% |
| right_angle, θ=1° | 92 | 0.625 µs | 20.8 µs | 3.0% |
| hairpin, θ=1° | 182 | 0.948 µs | 43.8 µs | 2.2% |

The algorithm is ~40× cheaper than the call that reaches it. Most of that is not
even ctypes — `build_turn_waypoints_native` does two `math.hypot` guard checks,
two `_unit` calls, a `math.ceil`, four buffer allocations and two list
comprehensions per call, which is more work than the C++ does. If this function
ever matters, the wrapper is the thing to attack. At embroidery-scale turn
counts it still is not the bottleneck.

**The native side of `redistribute_lookups` scales cleanly with output count** —
0.50 / 1.75 / 6.86 / 17.02 µs for out_n 102 / 367 / 1,464 / 3,657, about 4.6 ns
per output point. Earlier ctypes-based measurements reported a flat ~8 µs across
that whole range, because ~1.6 µs of ctypes overhead plus timer noise swamped
the real signal. The C++ harness is what made the scaling visible.

**Corner handling costs the C++ nothing.** At 1,000 corners the native call is
5.8 µs — no worse than the 6.5 µs with none — while the Python call is 347.8 µs.
The entire corner cost is bridge-side (resolving indices to arc lengths, then
`(ctypes.c_double * 1000)(*values)` star-unpacking at ~103 µs).

## The windowed/exhaustive crossover

`sort_curves.cpp` used to pick its 2-opt path solely from a hardcoded
`TWO_OPT_WINDOW_THRESHOLD = 10000` with no override, so the two paths could never
be measured at the same n. A `two_opt_mode` parameter (0 = auto, 1 = exhaustive,
2 = windowed) now makes the comparison possible; it defaults to 0 in the Python
wrapper, so ordinary callers are unaffected. The `sort_curves_crossover` group
runs both paths on identical input.

It reports **travel distance alongside time**, because the windowed path buys
speed with tour quality — a speed-only table would make it look strictly better
than it is, and the threshold is a choice between the two.

| n | exhaustive | windowed | |
|---|---|---|---|
| 2,000 | 134.5 ms, travel 28,319 | 1.10 s, travel 28,189 | windowed 8× slower |
| 5,000 | 1.49 s, travel 40,997 | 3.21 s, travel 41,427 | windowed 2.2× slower, 1.1% worse |
| 12,000 | 11.54 s, travel 58,445 | 9.63 s, travel 62,169 | windowed 1.2× faster, **6.4% worse** |

Below the threshold the windowed path is decisively worse on both axes, which
confirms the threshold's direction. Above it the trade is real but narrower than
the source comment claims: `sort_curves.cpp` says K=500 holds the quality loss to
~2%, and this input gives 6.4% at n=12,000 for only a 1.2× speedup. Whether that
is the input distribution or a genuinely optimistic comment needs a seed sweep
and a run at the 50k size the README's ~3 min → ~43 s figure came from.

The n=12,000 row above is from a one-off run; the group's own 8,000 and 12,000
cases are `--heavy`.

Sanity check on the mode switch itself: at n=5,000 `auto` and `exhaustive`
produce byte-identical travel, and at n=12,000 `auto` matches `windowed` — so
`auto` really is dispatching where the threshold says it should.
