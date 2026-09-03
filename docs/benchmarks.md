# Benchmarks

Measurements of `sort_points_native`, chosen because it is the simplest of the
four functions — greedy k-NN over a kd-tree, then 2-opt, with no direction or
reversal bookkeeping to muddy the timings.

## Greedy vs 2-opt

| n | greedy only | greedy + 2-opt | slowdown |
|---|---|---|---|
| 1,000 | 1.9 ms | 21 ms | 11× |
| 2,000 | 4.0 ms | 92 ms | 23× |
| 4,000 | 10.0 ms | 495 ms | 50× |
| 8,000 | 27.2 ms | 1.71 s | 63× |
| 16,000 | 49.0 ms | 12.0 s | 245× |
| 32,000 | 183.9 ms | 55.7 s | 303× |
| 64,000 | 500.9 ms | 213 s | 425× |

`slowdown` is the same row's right column divided by its left: how much longer
the whole call takes with `use_two_opt=True`.

## Scaling

Taking the endpoints rather than per-row ratios, which are noisy: n grows 64×
from 1,000 to 64,000, greedy time grows 264×, 2-opt time grows 9,956×.

```
greedy ≈ O(n^1.3)      log(264)  / log(64) = 1.34
2-opt  ≈ O(n^2.2)      log(9956) / log(64) = 2.21
```

Two things worth reading off these numbers:

**The kd-tree does its job, but not perfectly.** A textbook O(n log n) would sit
near n^1.1. The gap is the degradation noted in the README — the tree is static
and visited points are filtered out afterwards, so at large n the search window
repeatedly expands before it finds an unused neighbour.

**2-opt is the real cost, and it is superquadratic.** Each pass is O(n²), and the
number of passes needed to converge also grows with n, which is where the extra
0.2 in the exponent comes from. This is what motivates the windowed variant in
`sort_curves.cpp` for n > 10,000 — note that `sort_points.cpp`, measured here,
has only the exhaustive version, so every figure above is exhaustive 2-opt.

## Method

Single thread, MSVC build on Windows, `knn_k=12`, `two_opt_max_passes=10`.
Points drawn uniformly at random in a 1000 × 1000 square, seeded for
repeatability. Minimum of 3 runs per point (2 for greedy above 16,000; 1 for
2-opt above 8,000, where a single run already takes minutes).

Absolute timings vary noticeably between runs — an earlier session measured
8,000 with 2-opt at 2,337 ms against the 1,708 ms above. **Quote the ratios and
the exponents, not the milliseconds**, unless the environment is stated
alongside them.

## Reproducing

The numbers above were measured with the inline script below. It has since been
folded into a proper harness covering all four functions —
[`benchmarks/`](../benchmarks/README.md), run with `python benchmarks/run.py` —
which keeps these same sizes and `knn_k`/`two_opt_max_passes` so its
`sort_points` rows stay comparable with this table. Recorded runs live in
`benchmarks/results/`, each stamped with the environment and a hash of the
binary it used. The script is kept here as the minimal standalone version.

```python
import sys, time, random
sys.path.insert(0, "src")
sys.path.insert(0, "tests")
from test_sort_points import _Pt
from geomseq_core.geometry_utils import sort_points_native as S

rng = random.Random(1)
def pts(n):
    return [_Pt(rng.uniform(0, 1000), rng.uniform(0, 1000)) for _ in range(n)]

S(pts(200), use_two_opt=True)          # warm up: DLL load + first-call overhead

def bench(n, two_opt, reps):
    best = 1e9
    for _ in range(reps):
        p = pts(n)
        t = time.perf_counter()
        S(p, use_two_opt=two_opt)
        best = min(best, time.perf_counter() - t)
    return best

for n in [1000, 2000, 4000, 8000, 16000, 32000, 64000]:
    print(n, round(bench(n, False, 3) * 1000, 1), round(bench(n, True, 1) * 1000, 1))
```

The warm-up call matters: without it the first measurement absorbs the DLL load
and reads roughly 10× high.
