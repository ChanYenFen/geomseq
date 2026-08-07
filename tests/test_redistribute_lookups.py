"""Property tests for redistribute_lookups_native; cases are inline.

Plain floats in and out, so nothing needs stubbing.

Steps stay within [low, high] except where the native side must break the band
to land exactly on a corner or on total_length -- both are exempted below.
"""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import redistribute_lookups_native

TOL = 1e-9


def even_lookups(total_length, n):
    """Evenly spaced samples, as a real curve division would give."""
    return [total_length * i / (n - 1) for i in range(n)]


# flat_pct is a PERCENT (0-100), not a 0-1 fraction: the native side computes
# fade_len = total_length * (100 - flat_pct) / 200. So 100.0 holds one density
# the whole way, and 0.0 fades across the entire length.
CASES = [
    dict(name="mode0_dense_center", lookups=even_lookups(100.0, 101),
         low=2.0, high=8.0, mode=0, flat_pct=50.0, corner_indices=None),
    dict(name="mode1_dense_sides", lookups=even_lookups(100.0, 101),
         low=2.0, high=8.0, mode=1, flat_pct=50.0, corner_indices=None),
    dict(name="mode0_with_corners", lookups=even_lookups(100.0, 101),
         low=2.0, high=8.0, mode=0, flat_pct=50.0, corner_indices=[17, 43, 78]),
    dict(name="flat_pct_100_uniform", lookups=even_lookups(100.0, 101),
         low=3.0, high=9.0, mode=0, flat_pct=100.0, corner_indices=None),
    dict(name="single_segment", lookups=[0.0, 10.0],
         low=2.0, high=6.0, mode=0, flat_pct=50.0, corner_indices=None),
]


def check_case(name, lookups, low, high, mode, flat_pct, corner_indices):
    total_length = lookups[-1]
    corners = [lookups[i] for i in (corner_indices or [])]

    out = redistribute_lookups_native(lookups, low, high, mode, flat_pct,
                                      corner_indices=corner_indices)

    assert len(out) >= 2, f"{name}: expected at least both endpoints, got {len(out)}"
    assert math.isclose(out[0], 0.0, abs_tol=TOL), f"{name}: starts at {out[0]}"
    assert math.isclose(out[-1], total_length, abs_tol=TOL), (
        f"{name}: ends at {out[-1]}, expected {total_length}")

    for i, (prev, cur) in enumerate(zip(out, out[1:])):
        assert cur > prev, f"{name}: not increasing at index {i}: {prev} -> {cur}"

    for idx, corner in zip(corner_indices or [], corners):
        assert any(abs(v - corner) <= TOL for v in out), (
            f"{name}: corner index {idx} (arc-length {corner}) missing from output")

    steps = [b - a for a, b in zip(out, out[1:])]
    for i, step in enumerate(steps[:-1]):  # last step is truncated onto total_length
        # Corner look-ahead is `remaining <= 2 * step`; 2*high over-approximates it.
        if any(c > out[i] and c - out[i] <= 2.0 * high + TOL for c in corners):
            continue
        assert low - TOL <= step <= high + TOL, (
            f"{name}: step {i} is {step}, outside [{low}, {high}]")

    print(f"{name:<28} n_out={len(out):5d}   step[{min(steps):7.3f},{max(steps):7.3f}]")


def main():
    for case in CASES:
        check_case(**case)
    print("All tests passed")


if __name__ == "__main__":
    main()
