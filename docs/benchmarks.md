# Benchmarks

Where each call's time actually goes. Every number below comes from a recorded
baseline in [`benchmarks/results/`](../benchmarks/results/) — nothing here is
quoted from an ad-hoc run.

| | |
|---|---|
| Machine | Windows 10 AMD64, Intel64 Family 6 Model 165 |
| Toolchain | MSVC 1929, Python 3.10.6 |
| DLL under test | `geomseq_core.dll`, sha256 `ba426c4dd2ace0e4…`, built 2026-09-15 |

`native` is the algorithm alone, `python` is the same call through the wrapper
(ctypes + marshaling included), and `bridge` is the difference. See
[`benchmarks/README.md`](../benchmarks/README.md) for why both are measured.

**Quote the ratios and the shares, not the microseconds**, unless the
environment above is stated alongside them. Absolute timings move noticeably
between runs on the same machine.

## At a glance

### How fast, and where the time goes

Best time through Python, uniform data, at the largest size each group
measures in common. The sort rows ran at `max_passes = 10`; shipped callers
use 20, which adds tens of milliseconds at these sizes.

| function | size | time | where the time goes | source |
|---|---|---|---|---|
| `sort_points` | 16,000 points | 106.7 ms | greedy walk 16.1 ms, 2-opt the rest; at 64,000 the greedy walk is 76.0 of 588.9 ms | `20261106-sort-greedy-prune-heavy` |
| `sort_curves` | 16,000 curves | 201.3 ms | greedy walk 45.7 ms, 2-opt the rest | `20261106-sort-greedy-prune-heavy` |
| `shatter_at_crossings` | 16,000 segments | 42.3 ms | probably the Python bridge — no native measurement yet | `20261102-shatter-sweep-heavy` |
| `redistribute_arc_lengths` | 3,657 output points | 124.3 µs | 87% bridge, 13% algorithm | `comparison.md` |
| `build_turn_waypoints` | 182 waypoints | 41.45 µs | 98% bridge | `comparison.md` |

Source files are `benchmarks/results/baseline-windows-amd64-<source>.md`, or
the file named.

**The one thing to take away:** in both sort functions 2-opt is the expensive
part again — 77–87% of the call — since the greedy walk stopped re-fetching
placed points. For the three small functions the algorithm is a rounding error
beside the wrapper.

### Complexity

*Measured* is the growth between the smallest and largest committed size —
how it behaved over that range on uniform data, not a complexity class.
*Worst case* is from reading the code, with the input that would reach it.

| function | measured | over | worst case | what reaches it | memory |
|---|---|---|---|---|---|
| `sort_points`, greedy | n^1.06 (n^1.12 over the last 4×) | 1k–64k | O(n²) | a subtree is skipped only once *all* its points are placed, so unplaced points left one per leaf across a used region still make a query visit every one of those leaves | O(n) |
| `sort_points`, greedy + 2-opt | n^1.20 | 1k–64k | O(n²) per pass, ≤ 20 passes | ball queries are as wide as the tour's gaps, so a poor tour with long jumps queries most of it; each applied move also reverses up to n entries | O(n) |
| `sort_curves`, greedy | n^1.06 | 1k–16k | O(n²) | as for points, over 2n endpoints | O(n) |
| `sort_curves`, greedy + 2-opt | n^1.18 | 1k–16k | O(n²) per pass, ≤ 20 passes | as for points | O(n) |
| `shatter_at_crossings` | n^1.09 | 500–16k | O(n²) | bounding boxes that overlap without touching: long segments, dense clusters. On a cross-hatch the output itself is O(n²), so nothing can do better | O(n + k), k = contacts — O(n²) on a cross-hatch |
| `redistribute_arc_lengths` | linear in output, 4.6 ns per point natively | 102–3,657 out | O(n_out + corners) | — input length does not reach the native side at all | O(n_out) |
| `build_turn_waypoints` | linear in waypoints, 4.5 ns each natively | 2–182 | O(waypoints) | small `theta_max_deg` on a sharp turn — the count is turn angle over the cap | O(waypoints) |

**Other input shapes**, for the sorts — the same baseline also runs the
synthetic fixtures. The hardest clustering measured costs up to 16% more than
uniform at the top size: `sort_points` 680.8 against 588.9 ms at 64,000
(`clustered_1000x`), while `sort_curves` on `clustered_100x` is level with
uniform (201.8 against 201.3 ms at 16,000). Already-sorted input is far cheaper
— `sort_curves` on `zigzag` takes 42.1 ms at 16,000 — so existing order is
detected. Shatter,
redistribute and the turn builder have been measured on uniform data only.

## `redistribute_arc_lengths` — the work is in the bridge, not the algorithm

### Output count is the real axis, and the C++ tracks it exactly

| band | out_n | native | python | bridge | native % |
|---|---|---|---|---|---|
| 8-20 | 102 | 0.506 µs | 31.3 µs | 30.8 µs | 1.6% |
| 2-8 | 367 | 1.711 µs | 16.8 µs | 15.1 µs | 10.2% |
| 0.5-2 | 1,464 | 6.729 µs | 48.0 µs | 41.3 µs | 14.0% |
| 0.2-0.8 | 3,657 | 16.701 µs | 124.3 µs | 107.6 µs | 13.4% |

The native side is linear to three digits: 4.96 / 4.66 / 4.60 / 4.57 ns per
output point across a 36× range. That linearity is only visible from the C++
harness — through ctypes the per-call overhead swamps it at the small end.

The `out_n = 102` row is the one thing here that does not fit: fewer outputs,
yet the slowest Python time bar the largest case. The native row for it is
perfectly in line, so whatever it is lives on the bridge. Unexplained; it needs
a rerun before it is worth a theory.

### Input size barely matters any more

Holding the band fixed so `out_n` stays 367 while the input grows 990×:

| input_n | python | native |
|---|---|---|
| 101 | 16.3 µs | 1.711 µs |
| 1,001 | 16.9 µs | 1.711 µs |
| 10,001 | 19.0 µs | 1.711 µs |
| 100,001 | 60.0 µs | 1.711 µs |

990× the input moves the call 3.7× (≈ n^0.19), and the native column does not
move at all — since the ABI change the native side never receives the input
array, only `total_length` and the resolved corner arc lengths. It had only ever
read `arc lengths[n-1]` and the corner entries; marshaling the rest was pure waste.

This was the design guess run backwards. Input size was expected to be
incidental and output density to dominate; the first harness run showed the
opposite, which is why both axes are swept separately — one holds the band fixed
while input grows, the other holds input at 10,001 while the band moves `out_n`,
and the observed `out_n` column is what confirms each control actually held.

The residual 3.7× is not per-element work in the call. It is allocator and cache
pressure from having just built a large list outside the timed region.

*(The pre-fix figures that made this a 53× improvement came from a baseline
overwritten before it was committed. The shape of the finding survives; the
numbers are not reproducible and are not quoted.)*

### Corner cost is entirely bridge-side — and it is not superlinear

| corners | native | python | bridge | native % | µs per added corner |
|---|---|---|---|---|---|
| 0 | 6.729 µs | 47.0 µs | 40.3 µs | 14.3% | — |
| 10 | 6.675 µs | 84.8 µs | 78.1 µs | 7.9% | 3.78 |
| 100 | 6.438 µs | 131.2 µs | 124.8 µs | 4.9% | 0.84 |
| 1,000 | 5.974 µs | 375.7 µs | 369.7 µs | 1.6% | 0.33 |

The C++ column is flat — slightly *falling*, in fact. Every microsecond of the
8× Python increase is index-to-arc-length resolution plus building the ctypes
array. The marginal cost per corner falls as the count rises, so the earlier
"superlinear in corner count" reading does not hold against this baseline; the
fixed cost of setting up the array is a large share of the small-count rows.
Realistic corner counts are polyline vertices — single digits — so this is
noted, not urgent.

## `build_turn_waypoints` — ~97% of the call is wrapper

| geometry | θ | out_n | native | python | native % |
|---|---|---|---|---|---|
| straight | 30° | 2 | 0.141 µs | 6.05 µs | 2.3% |
| straight | 1° | 2 | 0.168 µs | 6.28 µs | 2.7% |
| right_angle | 5° | 20 | 0.297 µs | 12.24 µs | 2.4% |
| right_angle | 1° | 92 | 0.619 µs | 22.69 µs | 2.7% |
| hairpin | 5° | 38 | 0.330 µs | 14.94 µs | 2.2% |
| hairpin | 1° | 182 | 0.944 µs | 41.45 µs | 2.3% |

The native share sits between 1.8% and 3.3% across every case in the run — it
does not improve with output size, because both sides have the same shape:

```
native   ≈ 0.14 µs fixed  +   4.5 ns per waypoint
python   ≈ 5.9  µs fixed  + 195   ns per waypoint
```

Both terms are ~40× apart, so this is not a fixed call overhead that
amortizes away — the bridge costs 40× the algorithm *per waypoint* as well.
Most of it is not even ctypes: `build_turn_waypoints_native` does two
`math.hypot` guard checks, two `_unit` calls, a `math.ceil`, four buffer
allocations and two list comprehensions per call, which is more work than the
C++ does. If this function ever matters, the wrapper is the thing to attack. At
embroidery-scale turn counts it does not.

The `straight` rows are why the tables carry an **observed** `out_n` rather than
the declared knob: `theta_max_deg` only *caps* the per-waypoint turn, so a
straight run produces 2 waypoints whether the cap is 30° or 1°, and the timing
is flat across all four values.

## The sort functions

`sort_points` and `sort_curves` now have a committed baseline —
`baseline-windows-amd64-20260914-heavy` — along with three sweeps that exist to
answer one question each: `sort_curves_crossover`, `sort_curves_density` and
`sort_curves_passes`. Those three groups have since been retired from the
harness, because the question they were built to settle is settled: the windowed
2-opt they compared against was removed. Their result files stay as the evidence
for that decision, and [`design-notes.md`](design-notes.md) carries the reasoning.

A fourth group, `sort_curves_convergence`, sweeps the 2-opt pass cap across the
three curve fixtures. It was built to ask whether any input shape was being cut
off at `max_passes = 10`, found that none was, and then served a second purpose:
it is the before-and-after pair for pruning `sort_curves`' 2-opt search.

| | file |
|---|---|
| before | `baseline-windows-amd64-20260914-sort_curves_convergence-heavy` |
| after | `baseline-windows-amd64-20260914-sort_curves_convergence-pruned-heavy` |
| before, re-measured on an idle machine | `baseline-windows-amd64-20260914-sort_curves_convergence-exhaustive-control-heavy` |
| after, extended to 30 and 50 passes plus a 50,000-curve probe | `baseline-windows-amd64-20260914-sort_curves_convergence-pruned-extended-heavy` |

The third file exists because the first was recorded with Rhino running and the
second was not, which biased the comparison in favour of the new code. The
control puts both implementations on the same idle machine; it cost 4–9% of the
original timings and did not change the conclusion. Travel figures in the
control reproduce the first file exactly, which is what confirms it measured the
same implementation.

### Pruned against exhaustive, paired by seed

`sort_curves_prune_check` is the one comparison built to be quoted rather than
explored. It runs the same generated input through both implementations at 20
passes — where each has converged, so the columns hold two finished tours rather
than two points on differently-shaped convergence curves.

The two columns cannot come from one binary: which 2-opt the library carries is
decided at compile time, not by a parameter. They come from the closest thing
available — the same machine, the same MSVC, the same source tree differing in
one file, built 34 minutes apart, measured 24 seconds apart with Rhino closed
throughout.

| column | result file | `geomseq_core.dll` |
|---|---|---|
| pruned | `baseline-windows-amd64-20260914-sort_curves_prune_check-pruned-heavy` | `b69a189822a4c2ab`, 245,248 bytes |
| exhaustive | `baseline-windows-amd64-20260914-sort_curves_prune_check-exhaustive-heavy` | `a7002098686e053f`, 240,640 bytes |

| n | seed | exhaustive | pruned | travel | time | speedup |
|---|---|---|---|---|---|---|
| 16,000 | 1 | 65825.8 | 65204.3 | −0.94% | 20.75 s → 420.5 ms | 49× |
| 16,000 | 2 | 66140.0 | 65667.1 | −0.71% | 23.85 s → 381.4 ms | 63× |
| 16,000 | 3 | 65231.8 | 64851.9 | −0.58% | 24.18 s → 551.1 ms | 44× |
| 16,000 | 4 | 65839.2 | 65572.5 | −0.41% | 22.44 s → 361.0 ms | 62× |
| 16,000 | 5 | 66171.9 | 65675.9 | −0.75% | 20.47 s → 478.7 ms | 43× |
| 50,000 | 1 | 111173.3 | 110916.8 | −0.23% | 295.61 s → 2.62 s | 113× |

Read the rows, not an average. Seed-to-seed spread at fixed n is about 1.3%,
wider than the effect being measured, so the pairing is what carries the result:
pruning is shorter on all six, and on the three curve fixtures already recorded
in `sort_curves_convergence`, for nine inputs with no case of it being worse.

That is an empirical claim and cannot be more than one. Both implementations
settle into 2-opt local optima, and no theorem ranks two local optima — the tours
differ because each takes the first improving move it meets and the kd-tree meets
them in a different order.

The timing columns are not symmetrical in confidence. Every exhaustive row ran
once, because one repetition already exceeded the harness's 2 s budget, so those
figures carry whatever noise the machine had; the pruned rows are the best of
four or five. The travel figures carry none — they are deterministic, and the
50,000 pruned tour reproduces `sort_curves_convergence`'s converged value to the
digit from a separate run.

Headline sort timings and growth rates are in *At a glance* above, from
`baseline-windows-amd64-20260915-heavy`.

## The sort greedy walk with live counts — from quadratic to near-linear

The greedy walk used to ask the kd-tree for 12 neighbours, skip the placed
ones, and double the query until an unplaced one turned up. Placed points never
left the tree, so late in the walk each step fetched thousands of them to throw
away. Each tree node now counts its unplaced points, and the search skips any
subtree whose count is zero. The output is the same, bit for bit: same order,
same reversals, ties broken the same way (the reasoning is in
`nearest_unused.hpp`).

Greedy only, best time through Python, before → after:

| | n | before | after | |
|---|---|---|---|---|
| `sort_points`, uniform | 16,000 | 94.2 ms | 16.1 ms | 5.9× |
| | 64,000 | 1136.8 ms | 76.0 ms | 15.0× |
| `sort_points`, `clustered_1000x` | 64,000 | 1263.4 ms | 83.7 ms | 15.1× |
| `sort_points`, `grid` | 64,000 | 933.7 ms | 73.2 ms | 12.8× |
| `sort_curves`, uniform | 16,000 | 210.7 ms | 45.7 ms | 4.6× |
| `sort_curves`, `clustered_100x` | 16,000 | 268.9 ms | 47.4 ms | 5.7× |

Before is `baseline-windows-amd64-20261106-sort-before-greedy-prune-heavy`
(binary `8f98206b`), after is `…-20261106-sort-greedy-prune-heavy`
(`41833753`), run back to back on the same machine. The speed-up grows with n
because the cost it removed was the superlinear part: uniform greedy went from
n^1.58 to n^1.06 over 1k–64k.

With 2-opt on, the whole call gains 1.5–2.9× at the large sizes — 1633.4 →
588.9 ms for 64,000 uniform points — because 2-opt is now most of what is left.

**What this run does not say.** All of it is through Python, so the greedy
times include marshaling; how much of the remaining 76 ms is the bridge has not
been measured. Memory was not measured either: the counts add one small node
per kd-tree node plus one int and one byte per point.

## `shatter_at_crossings` before the broad phase — the pair loop

`baseline-windows-amd64-20260924-shatter-heavy`, sha256 `756e7af4075d83f7…`.
Superseded; the next section compares against it row for row.

- **A clean quadratic**: 27–29 ns per pair, flat from n=1,000 to 16,000
  (3.45 s at 16,000). `n` counts segments — a 100-span polyline is 100.
- **Density and gap did not change the cost.** The loop tested all n(n−1)/2
  pairs whatever it found.
- **Almost every test was wasted**: at n=2,000 and extent 250, 3,053 contacts
  out of 1,999,000 pairs, or 0.15%. That is what made a broad phase worth
  writing.

### The output buffer, which still applies

The wrapper sizes its first guess at `SHATTER_FIRST_GUESS` × n and calls again
if the native side reports it fell short, so falling short doubles the call.
The multiple is 3, measured: `out_n / n` stays under 2.4 at any gap wide enough
to be worth asking for, and 4n or 6n covered the same configurations as 3n. The
tail is `gap=0`, which removes nothing, so every contact becomes a whole extra
piece — past 20× in dense configurations. That tail is what `out_total` exists
for.

## `shatter_at_crossings` with a broad phase — the cost now follows the work

From `baseline-windows-amd64-20261102-shatter-sweep-heavy`, against
`geomseq_core.dll` sha256 `8f98206b5f86d01d…`: the same cases as the pair-loop
baseline above, on the same machine. Rhino was running alongside, which matters
more now that rows are milliseconds rather than seconds — read the ratios, not
the digits.

Contacts are now found by sweeping bounding boxes (`collect_contacts` in
`shatter_at_crossings.cpp`); only pairs whose boxes overlap reach the contact
predicate. **`out_n` and `vanished` match the pair-loop baseline in all 17
rows**, which is the expected result rather than a lucky one: the sweep skips
only pairs that provably cannot touch, and asks the same predicate the same
question about the rest.

### From quadratic to near-linear

| n | extent | out_n | pair loop | broad phase | speed-up | per segment |
|---|---|---|---|---|---|---|
| 500 | 707 | 520 | 4.1 ms | 972.5 µs | 4.2× | 1.95 µs |
| 1,000 | 1,000 | 1,037 | 14.7 ms | 2.1 ms | 7.0× | 2.10 µs |
| 2,000 | 1,414 | 2,090 | 55.3 ms | 4.4 ms | 12.6× | 2.20 µs |
| 4,000 | 2,000 | 4,160 | 221.6 ms | 10.2 ms | 21.7× | 2.55 µs |
| 8,000 | 2,828 | 8,342 | 919.8 ms | 19.9 ms | 46× | 2.49 µs |
| 16,000 | 4,000 | 16,663 | 3.45 s | 42.3 ms | 82× | 2.64 µs |

Across a 32× range of n, time grows 43×: close to linear. The excess is
consistent with the n log n sort the sweep starts with, but the bridge could
account for it just as well — see below. The speed-up therefore grows with n
rather than settling at a constant, because one side is quadratic and the
other is not.

**What this table cannot say is how much of it is the bridge.** This group has
no `native` column, and the reason recorded for that — the bridge was under
0.1% of a quadratic loop — no longer holds. Per-segment time that is flat
across n is what marshalling looks like as much as what the sweep looks like.
A native harness row would settle it, and should exist before anyone tries to
make this faster.

### Density and gap now cost something, and should

| extent | out_n | pair loop | broad phase |
|---|---|---|---|
| 2,000 | 2,046 | 56.1 ms | 4.5 ms |
| 1,000 | 2,153 | 56.2 ms | 4.7 ms |
| 500 | 2,584 | 58.0 ms | 7.8 ms |
| 250 | 3,820 | 60.8 ms | 8.3 ms |
| 125 | 4,721 | 117.8 ms | 11.3 ms |

All at n=2,000. The pair loop was flat because it tested every pair whatever it
found; the broad phase tests only overlapping boxes, so packing segments tighter
raises the candidate count and the time with it — 2.5× across this range. The
`extent 125` row no longer doubles: the pair-loop run regrew its buffer there,
this one did not.

`gap` moved in the same direction for a different reason. 0, 2, 8 and 32 give
8.5, 7.5, 5.7 and 5.1 ms where the pair loop gave 57–59 ms for all but `gap=0`,
and contacts do not depend on the gap at all. What does is the output: a wider
gap leaves fewer pieces to build into Python tuples. That is an inference from
`out_n` falling alongside the time, not a measurement of the bridge.

Polylines — `self_x` off then on — went from 49.8 and 51.2 ms to 7.5 and 8.3 ms.

### What this run does not cover

- **Inputs whose contact count is itself quadratic.** A cross-hatch, where
  every line meets every line of the other direction, gives the broad phase
  nothing to skip: the output is O(n²) and so is any method that produces it.
  No such case is in the group, so the run says nothing about it.
- **Clustered input**, as for every other group. Overlap between boxes is
  exactly what clustering raises.
