# Benchmarks

Where each call's time actually goes, for the two functions whose cost the
Python-side harness alone cannot resolve. Every number below comes from the
recorded baseline in [`benchmarks/results/`](../benchmarks/results/) — nothing
here is quoted from an ad-hoc run.

| | |
|---|---|
| Machine | Windows 10 AMD64, Intel64 Family 6 Model 165 |
| Toolchain | MSVC 1929, Python 3.10.6 |
| DLL under test | `geomseq_core.dll`, sha256 `fd6ad632ec57ae5e…`, built 2026-09-03 |

`native` is the algorithm alone, `python` is the same call through the wrapper
(ctypes + marshaling included), and `bridge` is the difference. See
[`benchmarks/README.md`](../benchmarks/README.md) for why both are measured.

**Quote the ratios and the shares, not the microseconds**, unless the
environment above is stated alongside them. Absolute timings move noticeably
between runs on the same machine.

## `redistribute_lookups` — the work is in the bridge, not the algorithm

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
read `lookups[n-1]` and the corner entries; marshaling the rest was pure waste.

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
for that decision, and [`../CLAUDE.md`](../CLAUDE.md) carries the reasoning.

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

The remaining headline numbers are not written up here yet. Anything quoted from
them should cite the result file by name, as the rule at the top of this document
requires.
