"""Representative inputs per function -- what counts as a size, separate from
run.py's how it gets timed. Each function's axis differs; see ../README.md.
Stand-in geometry mirrors tests/, so no Rhino types are needed."""

import glob
import json
import math
import os
import random

from geomseq_core.geometry_utils import (
    PIC_COINCIDENT,
    PIC_INSIDE,
    PIC_OUTSIDE,
    SHATTER_FIRST_GUESS,
    build_turn_waypoints_native,
    point_in_curve_native,
    redistribute_arc_lengths_native,
    shatter_at_crossings_native,
    sort_curves_native,
    sort_points_native,
)


# --------------------------------------------------------------------------
# Stand-in geometry (same shapes as tests/test_sort_points.py, test_sort_curves.py)
# --------------------------------------------------------------------------

class _Pt:
    __slots__ = ("X", "Y", "Z")

    def __init__(self, x, y, z=0.0):
        self.X, self.Y, self.Z = x, y, z


class _Seg:
    """Stands in for a Rhino curve, backed by [x0, y0, x1, y1]."""

    __slots__ = ("x0", "y0", "x1", "y1")

    def __init__(self, x0, y0, x1, y1):
        self.x0, self.y0, self.x1, self.y1 = x0, y0, x1, y1

    @property
    def PointAtStart(self):
        return _Pt(self.x0, self.y0)

    @property
    def PointAtEnd(self):
        return _Pt(self.x1, self.y1)

    def Duplicate(self):
        return _Seg(self.x0, self.y0, self.x1, self.y1)

    def Reverse(self):
        self.x0, self.x1 = self.x1, self.x0
        self.y0, self.y1 = self.y1, self.y0


# --------------------------------------------------------------------------
# Input generators -- all seeded, so a rerun on the same machine is comparable
# --------------------------------------------------------------------------

EXTENT = 1000.0  # points/curves are scattered in an EXTENT x EXTENT square

def make_points(n, seed=1):
    rng = random.Random(seed)
    return [_Pt(rng.uniform(0, EXTENT), rng.uniform(0, EXTENT)) for _ in range(n)]


def make_segments(n, seed=1, min_len=5.0, max_len=20.0, extent=None):
    """Short randomly-oriented segments -- stitches/toolpath strokes, not a mesh.

    `extent` overrides the module-level square. It exists so a sweep can hold
    density fixed instead of packing more segments into the same area; segment
    lengths deliberately do not scale with it, because a stitch stays the same
    size when the design gets bigger. The default is unchanged, so every
    committed baseline stays comparable."""
    rng = random.Random(seed)
    span = EXTENT if extent is None else extent
    out = []
    for _ in range(n):
        x0, y0 = rng.uniform(0, span), rng.uniform(0, span)
        ang, ln = rng.uniform(0, 2 * math.pi), rng.uniform(min_len, max_len)
        out.append(_Seg(x0, y0, x0 + math.cos(ang) * ln, y0 + math.sin(ang) * ln))
    return out


# --------------------------------------------------------------------------
# Recorded fixtures -- optional, additive; see ../README.md ("Adding a fixture")
# --------------------------------------------------------------------------

FIXTURE_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "fixtures")

_FIXTURE_CACHE = {}


def fixture_paths(kind):
    """Fixture files of `kind` ("points" / "curves"). Kind is in the filename so
    globbing does not have to parse megabytes just to find out."""
    return sorted(glob.glob(os.path.join(FIXTURE_DIR, "%s_*.json" % kind)))


def load_fixture(path):
    """Parse (and cache) one fixture -> its `data` list. Multi-MB files, so this
    is deliberately lazy: nothing is read until a case using it actually runs."""
    if path not in _FIXTURE_CACHE:
        with open(path, encoding="utf-8") as fh:
            doc = json.load(fh)
        _FIXTURE_CACHE[path] = doc["data"]
    return _FIXTURE_CACHE[path]


def fixture_name(path):
    """points_real_lettering.json -> real_lettering."""
    return os.path.basename(path).split("_", 1)[1].rsplit(".", 1)[0]


def sample(data, n, seed=1):
    """`n` items drawn from `data`, seeded, **in the source's own order**.
    Sampling rather than slicing keeps the distribution (clustering included)
    comparable across n; a slice would be one region of the design and would
    muddy the scaling exponent. Indices are re-sorted because input order is
    itself a property under test -- `random.sample` returns its picks in
    random order, which would silently shuffle an already-sorted fixture and
    turn points_zigzag into points_grid at every n below the file's length."""
    if n >= len(data):
        return list(data)
    idx = sorted(random.Random(seed).sample(range(len(data)), n))
    return [data[i] for i in idx]


def even_arc_lengths(total_length, n):
    """Evenly spaced arc-length samples, as a real curve division would give."""
    return [total_length * i / (n - 1) for i in range(n)]


def spread_corners(n_arc_lengths, count):
    """`count` corner indices spread over the interior of an arc length list."""
    if count <= 0:
        return None
    step = (n_arc_lengths - 2) / float(count)
    return [int(1 + i * step) for i in range(count)]


PIC_R = EXTENT * 0.4            # the curve is inscribed in the EXTENT square
PIC_CX = PIC_CY = EXTENT / 2.0


def make_closed_curve(v, shape="circle"):
    """A simple closed polyline of `v` vertices, first repeated as last.

    `shape` is an axis rather than a constant because it is what decides how
    much a y-interval index could ever buy. The crossing count only needs edges
    whose y-span contains the query's y, so what matters is how many edges share
    a height:

      circle -- convex, so a horizontal ray meets exactly 2 edges whatever v is,
                and an index would skip almost everything
      comb   -- a sawtooth whose teeth all span the same band, so a ray through
                that band meets a large fraction of the edges and an index can
                skip little

    Both are simple: the comb's teeth advance monotonically in x, and it closes
    along the bottom."""
    if shape == "circle":
        n = v - 1
        pts = [(PIC_CX + PIC_R * math.cos(2.0 * math.pi * k / n),
                PIC_CY + PIC_R * math.sin(2.0 * math.pi * k / n))
               for k in range(n)]
    elif shape == "comb":
        teeth = v - 3
        pts = [(PIC_CX - PIC_R + 2.0 * PIC_R * k / (teeth - 1),
                PIC_CY + (PIC_R if k % 2 else 0.0))
               for k in range(teeth)]
        pts.append((PIC_CX + PIC_R, PIC_CY - PIC_R))
        pts.append((PIC_CX - PIC_R, PIC_CY - PIC_R))
    else:
        raise ValueError("unknown shape %r" % shape)
    return pts + [pts[0]]


def make_query_points(n, where="uniform", seed=2):
    """`n` query points, placed by `where`.

    This is the axis that decides how many points the bounding-box rejection
    answers outright, which is the difference between timing the scan and timing
    the box:

      uniform -- over the curve's bounding square. The control, and note what it
                 is *not*: every point is inside the box, so the rejection never
                 fires and this row sits at the same cost as `inside`. That is a
                 property of the placement, not of the function.
      spread  -- over a square three times as wide, which is what a real job
                 looks like: one curve somewhere on a sheet of points. This is
                 the mixed row, and the only one where `scanned` lands strictly
                 between 0 and n.
      inside  -- all within the curve, so every one pays both scans
      outside -- all past the box, so the box answers every one and no edge is read

    `inside` sits in the lower half, which is interior to both shapes: inside the
    circle (0.8 R from its centre at worst) and inside the comb's solid base."""
    rng = random.Random(seed)
    if where in ("uniform", "spread"):
        r = PIC_R if where == "uniform" else 3.0 * PIC_R
        return [(rng.uniform(PIC_CX - r, PIC_CX + r),
                 rng.uniform(PIC_CY - r, PIC_CY + r)) for _ in range(n)]
    if where == "inside":
        out = []
        while len(out) < n:
            dx = rng.uniform(-0.3, 0.3) * PIC_R
            dy = rng.uniform(-0.3, 0.3) * PIC_R
            if dx * dx + dy * dy <= (0.3 * PIC_R) ** 2:
                out.append((PIC_CX + dx, PIC_CY - 0.5 * PIC_R + dy))
        return out
    if where == "outside":
        # Past 1.414 R nothing can still be in the box, so 2-4 R clears it by a
        # margin no tol on the sweep below can close.
        out = []
        for _ in range(n):
            a = rng.uniform(0.0, 2.0 * math.pi)
            d = rng.uniform(2.0, 4.0) * PIC_R
            out.append((PIC_CX + d * math.cos(a), PIC_CY + d * math.sin(a)))
        return out
    raise ValueError("unknown where %r" % where)


# --------------------------------------------------------------------------
# Case objects
# --------------------------------------------------------------------------

class Case:
    """One measured configuration. `setup()` builds a fresh input and is NOT
    timed; `run(payload)` is the call under measurement -- keeping them apart
    is why input generation never lands in the numbers."""

    def __init__(self, group, name, setup, run, axis, heavy=False, batch=1,
                 observe=None):
        self.group = group      # which function
        self.name = name        # unique within the group
        self.setup = setup
        self.run = run
        self.axis = axis        # dict of the varying parameters, for the record
        self.heavy = heavy      # skipped unless --heavy
        self.batch = batch      # calls per timed run (>1 for microsecond funcs)
        # observe(payload) -> actual output metrics, called outside the timed
        # region. The axis is an input knob; this is what the call produced --
        # not the same thing (a straight turn ignores theta_max_deg).
        self.observe = observe


# --- sort_points -----------------------------------------------------------
# Continuity with docs/benchmarks.md: same sizes, same knn_k/max_passes, so the
# greedy and 2-opt columns line up against the numbers already published there.

SORT_POINTS_SIZES = [1000, 2000, 4000, 8000, 16000, 32000, 64000]
TWO_OPT_CHEAP_LIMIT = 8000  # above this, exhaustive 2-opt runs into minutes

KNN_K, MAX_PASSES = 12, 10


def _sources(kind, synthetic, wrap):
    """(label, builder, available_n) per data source: every fixture of `kind`,
    plus the in-code synthetic generator as the `uniform` control. available_n
    caps the sweep so a case never claims an n the fixture cannot supply (None
    = unlimited, i.e. generated on demand).

    A fixture named `uniform` *replaces* the generator rather than sitting
    beside it: two sources sharing a label would emit duplicate case names.
    The generator stays as the fallback, so an empty fixtures/ still runs."""
    out = []
    for path in fixture_paths(kind):
        data = load_fixture(path)
        out.append((fixture_name(path),
                    lambda n, d=data: [wrap(row) for row in sample(d, n)],
                    len(data)))
    if not any(label == "uniform" for label, _, _ in out):
        out.append(("uniform", synthetic, None))
    out.sort(key=lambda s: (s[0] != "uniform", s[0]))   # control first
    return out


def _point_sources():
    return _sources("points", make_points, lambda row: _Pt(row[0], row[1]))


def _sort_points_cases():
    cases = []
    sources = _point_sources()
    for two_opt in (False, True):
        for label, build, avail in sources:
            for n in SORT_POINTS_SIZES:
                if avail is not None and n > avail:
                    continue
                cases.append(Case(
                    "sort_points",
                    "%s_%s_n%d" % (label, "2opt" if two_opt else "greedy", n),
                    setup=lambda n=n, build=build: build(n),
                    run=lambda p, t=two_opt: sort_points_native(
                        p, use_two_opt=t, two_opt_max_passes=MAX_PASSES, knn_k=KNN_K),
                    axis=dict(data=label, n=n, two_opt=two_opt),
                    heavy=(two_opt and n > TWO_OPT_CHEAP_LIMIT),
                ))
    return cases


# --- sort_curves -----------------------------------------------------------
# Sizes still straddle 10,000, which used to be where the native side switched
# 2-opt implementations. It no longer switches -- there is only one -- so these
# rows measure a single implementation across the range rather than two either
# side of a boundary. The sizes are kept so they stay comparable with the
# baselines recorded while the boundary existed.
# if_flip=False makes the native side skip 2-opt outright.

SORT_CURVES_SIZES = [1000, 4000, 8000, 12000, 16000]
SORT_CURVES_HEAVY_ABOVE = 12000


def _curve_sources():
    """As _point_sources, for curves. Fixture rows are [x0, y0, x1, y1], the
    same shape tests/fixtures/sort_curves_cases.json already uses."""
    return _sources("curves", make_segments, lambda row: _Seg(*row))


def _sort_curves_cases():
    cases = []
    sources = _curve_sources()
    for two_opt in (False, True):
        for label, build, avail in sources:
            for n in SORT_CURVES_SIZES:
                if avail is not None and n > avail:
                    continue
                axis = dict(data=label, n=n, two_opt=two_opt, if_flip=True)
                if two_opt:
                    # Names the implementation that produced the row, which is
                    # the entire point of the column. It read "exhaustive", with
                    # a comment about "what auto takes at every n" -- both stale:
                    # `auto` died with the windowed dispatch, and the exhaustive
                    # pass was replaced by the pruned search on 2026-09-14. A
                    # column that misnames its own implementation is worse than
                    # no column, so it changes whenever the implementation does.
                    axis["two_opt_path"] = "pruned"
                cases.append(Case(
                    "sort_curves",
                    "%s_%s_n%d" % (label, "2opt" if two_opt else "greedy", n),
                    setup=lambda n=n, build=build: build(n),
                    run=lambda c, t=two_opt: sort_curves_native(
                        c, use_two_opt=t, if_flip=True,
                        two_opt_max_passes=MAX_PASSES, knn_k=KNN_K),
                    axis=axis,
                    heavy=(two_opt and n > SORT_CURVES_HEAVY_ABOVE),
                ))
    for n in [1000, 8000, 16000]:
        cases.append(Case(
            "sort_curves", "uniform_fixeddir_n%d" % n,
            setup=lambda n=n: make_segments(n),
            run=lambda c: sort_curves_native(c, use_two_opt=True, if_flip=False,
                                             two_opt_max_passes=MAX_PASSES, knn_k=KNN_K),
            axis=dict(data="uniform", n=n,
                      two_opt="skipped (if_flip=False)", if_flip=False),
        ))
    return cases


# --- sort_curves: does 2-opt converge by the shipped cap, on every shape? ----
# max_passes = 10 is what ships, and on uniform data it lands within 0.1% of
# converged at both 25,000 and 50,000. That is one distribution. If some input
# shape is still improving at 10, callers with that shape quietly get a less
# converged tour than everyone else -- a quality question, not a speed one, and
# the answer it would imply is to raise the cap, not lower it.
#
# Fixtures rather than the generator, because shape is the axis here. n is
# pinned at 16,000, the fixture length, so zigzag keeps its true serpentine
# order: sample() preserves the file's order only when it returns the whole
# file, and a sampled subset of zigzag is not a sorted input any more.
#
# The sweep runs past the point of interest on purpose. 10 is what ships and 20
# was already flat, but reading "flat" off the last column of a table is how a
# cap gets confirmed by its own boundary; 30 and 50 are there so convergence is
# something the table shows rather than something it runs out of room to deny.
#
# The large-n rows answer a different question and cannot answer the first one.
# Pruned 2-opt trails the exhaustive pass until it converges, so the risk it
# carries is a cap that is generous at 16,000 and short at 50,000 -- more curves
# means more crossings to work through. Only the generator reaches 50,000, and
# the generator makes uniform scatter, so these rows say nothing whatsoever
# about how a clustered or serpentine 50,000-curve job converges. They are
# labelled `generated_uniform` rather than `uniform` because `uniform` is the
# fixture's reserved label, and a table that cannot tell the two apart is worse
# than one that admits the gap.

CONVERGENCE_N = 16000
CONVERGENCE_SWEEP = [1, 2, 3, 5, 10, 20, 30, 50]

CONVERGENCE_BIG_N = 50000
CONVERGENCE_BIG_LABEL = "generated_uniform"


def travel_distance(curves):
    """Sum of gaps: end of one curve to start of the next (same as tests/)."""
    return sum(math.hypot(c.PointAtStart.X - p.PointAtEnd.X,
                          c.PointAtStart.Y - p.PointAtEnd.Y)
               for p, c in zip(curves, curves[1:]))


def _convergence_case(label, build, n, passes):
    def run(c, passes=passes):
        ordered, _ = sort_curves_native(
            c, use_two_opt=True, two_opt_max_passes=passes, knn_k=KNN_K)
        return ordered

    def observe(c, passes=passes):
        ordered, _ = sort_curves_native(
            c, use_two_opt=True, two_opt_max_passes=passes, knn_k=KNN_K)
        return dict(travel=round(travel_distance(ordered), 1))

    return Case(
        "sort_curves_convergence", "%s_n%d_p%d" % (label, n, passes),
        setup=lambda build=build, n=n: build(n),
        run=run, observe=observe,
        axis=dict(data=label, n=n, max_passes=passes),
        heavy=True,
    )


def _convergence_cases():
    cases = []
    for label, build, avail in _curve_sources():
        if avail is not None and CONVERGENCE_N > avail:
            continue
        for passes in CONVERGENCE_SWEEP:
            cases.append(_convergence_case(label, build, CONVERGENCE_N, passes))

    for passes in CONVERGENCE_SWEEP:
        cases.append(_convergence_case(
            CONVERGENCE_BIG_LABEL, make_segments, CONVERGENCE_BIG_N, passes))
    return cases


# --- sort_curves: is the pruned search's quality parity luck? ---------------
# Pruning discards no improving move, but that does not make it produce the same
# tour: both implementations take the first improving move they meet, meet them
# in different orders, and settle into *different* 2-opt local optima. No
# theorem ranks those, so "pruned is never worse" can only ever be an empirical
# claim -- and it was resting on three fixtures at one n and one seed.
#
# So: vary the seed at a fixed n, which is the direct test of whether parity was
# luck, and add one 50,000-curve row, the size where the two have never been
# compared at all. Both at 20 passes, where each has converged, so the
# comparison is between finished tours rather than between two points on
# different convergence curves -- pruning trails while unconverged, and
# comparing mid-flight would measure that instead of the answer.
#
# This group is run twice, once per implementation, and the travel columns
# compared. It cannot be run against both at once: which 2-opt the binary
# carries is a build-time fact, not a parameter.

PRUNE_CHECK_N = 16000
PRUNE_CHECK_SEEDS = [1, 2, 3, 4, 5]
PRUNE_CHECK_BIG_N = 50000
PRUNE_CHECK_PASSES = 20


def _prune_check_cases():
    def run(c):
        ordered, _ = sort_curves_native(
            c, use_two_opt=True, two_opt_max_passes=PRUNE_CHECK_PASSES,
            knn_k=KNN_K)
        return ordered

    def observe(c):
        ordered, _ = sort_curves_native(
            c, use_two_opt=True, two_opt_max_passes=PRUNE_CHECK_PASSES,
            knn_k=KNN_K)
        return dict(travel=round(travel_distance(ordered), 1))

    sizes = [(PRUNE_CHECK_N, seed) for seed in PRUNE_CHECK_SEEDS]
    sizes.append((PRUNE_CHECK_BIG_N, 1))

    return [
        Case(
            "sort_curves_prune_check", "generated_n%d_s%d" % (n, seed),
            setup=lambda n=n, seed=seed: make_segments(n, seed=seed),
            run=run, observe=observe,
            axis=dict(data="generated_uniform", n=n, seed=seed,
                      max_passes=PRUNE_CHECK_PASSES),
            heavy=True,
        )
        for n, seed in sizes
    ]


# --- redistribute_arc_lengths --------------------------------------------------
# Both input n and output count are swept: which dominates was an open question
# and the answer moved once the ABI stopped passing the input array.

REDIST_TOTAL = 1000.0


def _observe_out_n(call):
    """Actual output length, measured once outside the timed region."""
    return lambda payload: dict(out_n=len(call(payload)))


def _redistribute_cases():
    cases = []

    # 1. input resolution, band held fixed -> output count should barely move
    for n_in in [101, 1001, 10001, 100001]:
        call = lambda lk: redistribute_arc_lengths_native(lk, 2.0, 8.0, 0, 50.0)
        cases.append(Case(
            "redistribute_arc_lengths", "input_n%d" % n_in,
            setup=lambda n_in=n_in: even_arc_lengths(REDIST_TOTAL, n_in),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=n_in, band="2-8", corners=0, mode=0),
        ))

    # 2. output density, input held fixed
    for low, high in [(8.0, 20.0), (2.0, 8.0), (0.5, 2.0), (0.2, 0.8)]:
        call = (lambda low, high: lambda lk: redistribute_arc_lengths_native(
            lk, low, high, 0, 50.0))(low, high)
        cases.append(Case(
            "redistribute_arc_lengths", "band_%g_%g" % (low, high),
            setup=lambda: even_arc_lengths(REDIST_TOTAL, 10001),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=10001, band="%g-%g" % (low, high), corners=0, mode=0),
        ))

    # 3. corner count -- each corner forces a look-ahead and a rescaled step
    for nc in [0, 10, 100, 1000]:
        call = (lambda nc: lambda lk: redistribute_arc_lengths_native(
            lk, 0.5, 2.0, 0, 50.0, corner_indices=spread_corners(10001, nc)))(nc)
        cases.append(Case(
            "redistribute_arc_lengths", "corners_%d" % nc,
            setup=lambda: even_arc_lengths(REDIST_TOTAL, 10001),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=10001, band="0.5-2", corners=nc, mode=0),
        ))

    # 4. mode 0 vs 1 -- expected flat, recorded to confirm rather than assume
    for mode in [0, 1]:
        call = (lambda mode: lambda lk: redistribute_arc_lengths_native(
            lk, 0.5, 2.0, mode, 50.0))(mode)
        cases.append(Case(
            "redistribute_arc_lengths", "mode%d" % mode,
            setup=lambda: even_arc_lengths(REDIST_TOTAL, 10001),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=10001, band="0.5-2", corners=0, mode=mode),
        ))
    return cases


# --- build_turn_waypoints --------------------------------------------------
# One call is microseconds, below timer resolution, so it runs in batches.
# Geometry reuses tests/, whose gaps avoid the wrapper's junction warnings.

TURN_BATCH = 2000

TURN_GEOMETRIES = {
    # name: (E_prev, E, S, S_next)
    "straight":    ((0.0, 0.0), (10.0, 0.0), (30.0, 0.0), (40.0, 0.0)),
    "right_angle": ((0.0, 0.0), (10.0, 0.0), (20.0, 10.0), (20.0, 20.0)),
    "hairpin":     ((0.0, 0.0), (10.0, 0.0), (10.0, 5.0), (0.0, 5.0)),
}


def _turn_args(geom_name, theta_max_deg, step_len=1.0, extend_len=2.0):
    E_prev, E, S, S_next = TURN_GEOMETRIES[geom_name]
    a_vec = (E[0] - E_prev[0], E[1] - E_prev[1])
    b_vec = (S_next[0] - S[0], S_next[1] - S[1])
    return (E[0], E[1], a_vec[0], a_vec[1], S[0], S[1], b_vec[0], b_vec[1],
            theta_max_deg, step_len, extend_len)


def _run_turn_batch(args):
    for _ in range(TURN_BATCH):
        build_turn_waypoints_native(*args)


def _observe_turn(args):
    exit_pts, entry_pts = build_turn_waypoints_native(*args)
    return dict(out_n=len(exit_pts) + len(entry_pts))


def _turn_cases():
    cases = []
    for geom in ["straight", "right_angle", "hairpin"]:
        for theta in [30.0, 10.0, 5.0, 1.0]:
            cases.append(Case(
                "build_turn_waypoints", "%s_theta%g" % (geom, theta),
                setup=lambda geom=geom, theta=theta: _turn_args(geom, theta),
                run=_run_turn_batch, observe=_observe_turn,
                # theta_max_deg only *caps* the per-waypoint turn; how many
                # waypoints that actually costs depends on how far the path
                # has to turn, so the observed out_n is the honest size here.
                axis=dict(geometry=geom, theta_max_deg=theta),
                batch=TURN_BATCH,
            ))
    return cases


# --------------------------------------------------------------------------

# --- sort_points: where does its pruned 2-opt converge? --------------------
# sort_curves' cap of 20 was measured on curves. Nothing says points converge at
# the same rate, and borrowing the number would repeat the mistake the windowed
# 2-opt was retired for -- a constant carried from a place it was never measured.
# The pruned search is only ever the worse of the two implementations while it
# is still unconverged, so where it converges is exactly what decides the cap.
#
# Fixtures rather than the generator, because shape is the axis. n is pinned at
# 64,000, the fixture length, so points_zigzag keeps its serpentine order:
# sample() preserves the file's order only when it returns the whole file, and a
# sampled subset of zigzag is not an ordered input any more. Three sources, the
# same three shapes the curve group uses -- the uniform control, one clustered,
# one arriving already ordered.
#
# 64,000 is also where sort_points actually hurts: the exhaustive pass takes
# 248.68 s there, which is the number this work exists to change.

POINTS_CONVERGENCE_N = 64000
POINTS_CONVERGENCE_SWEEP = [1, 2, 3, 5, 10, 20, 30]
POINTS_CONVERGENCE_DATA = ("uniform", "clustered_100x", "zigzag")


def point_path_length(points):
    """Sum of gaps between consecutive points. No start_pt term: the sort is
    being compared against itself, and the first hop is identical either way."""
    return sum(math.hypot(b.X - a.X, b.Y - a.Y)
               for a, b in zip(points, points[1:]))


def _points_convergence_cases():
    cases = []
    for label, build, avail in _point_sources():
        if label not in POINTS_CONVERGENCE_DATA:
            continue
        if avail is not None and POINTS_CONVERGENCE_N > avail:
            continue
        for passes in POINTS_CONVERGENCE_SWEEP:
            def run(p, passes=passes):
                ordered, _ = sort_points_native(
                    p, use_two_opt=True, two_opt_max_passes=passes, knn_k=KNN_K)
                return ordered

            def observe(p, passes=passes):
                ordered, _ = sort_points_native(
                    p, use_two_opt=True, two_opt_max_passes=passes, knn_k=KNN_K)
                return dict(travel=round(point_path_length(ordered), 1))

            cases.append(Case(
                "sort_points_convergence",
                "%s_n%d_p%d" % (label, POINTS_CONVERGENCE_N, passes),
                setup=lambda build=build: build(POINTS_CONVERGENCE_N),
                run=run, observe=observe,
                axis=dict(data=label, n=POINTS_CONVERGENCE_N, max_passes=passes),
                heavy=True,
            ))
    return cases


# --- sort_points: is the pruned search ever the worse one? -----------------
# The same question `sort_curves_prune_check` answers, asked again rather than
# inherited. Pruning discards no improving move, but it does not reproduce the
# other implementation's tour: both take the first improving move they meet and
# the kd-tree meets them in a different order, so the two settle into different
# 2-opt local optima and no theorem ranks those. Whether the pruned one is ever
# the worse is therefore empirical, and the curve result is evidence about
# curves.
#
# 20 passes, where `sort_points_convergence` shows every shape converged, so the
# columns hold finished tours rather than two points on differently-shaped
# convergence curves -- pruning trails while unconverged, and comparing
# mid-flight would measure that instead of the answer.
#
# Five seeds at 16,000 ask whether any parity is luck. The 64,000 row is the one
# that matters: the exhaustive pass takes 248.68 s there, which is the number
# this work exists to change.
#
# Run twice, once per implementation, and compare the travel columns. It cannot
# be run against both at once: which 2-opt the binary carries is a build-time
# fact, not a parameter.

POINTS_PRUNE_CHECK_N = 16000
POINTS_PRUNE_CHECK_SEEDS = [1, 2, 3, 4, 5]
POINTS_PRUNE_CHECK_BIG_N = 64000
POINTS_PRUNE_CHECK_PASSES = 20


def _points_prune_check_cases():
    def run(p):
        ordered, _ = sort_points_native(
            p, use_two_opt=True, two_opt_max_passes=POINTS_PRUNE_CHECK_PASSES,
            knn_k=KNN_K)
        return ordered

    def observe(p):
        ordered, _ = sort_points_native(
            p, use_two_opt=True, two_opt_max_passes=POINTS_PRUNE_CHECK_PASSES,
            knn_k=KNN_K)
        return dict(travel=round(point_path_length(ordered), 1))

    sizes = [(POINTS_PRUNE_CHECK_N, seed) for seed in POINTS_PRUNE_CHECK_SEEDS]
    sizes.append((POINTS_PRUNE_CHECK_BIG_N, 1))

    return [
        Case(
            "sort_points_prune_check", "generated_n%d_s%d" % (n, seed),
            setup=lambda n=n, seed=seed: make_points(n, seed=seed),
            run=run, observe=observe,
            axis=dict(data="generated_uniform", n=n, seed=seed,
                      max_passes=POINTS_PRUNE_CHECK_PASSES),
            heavy=True,
        )
        for n, seed in sizes
    ]


# --------------------------------------------------------------------------

# --- shatter_at_crossings --------------------------------------------------
# n counts SEGMENTS, not curves: a polyline explodes into one entry per span, so
# twenty polylines of a hundred spans is n = 2,000, not 20. That is why n leads
# here. It was a clean O(n^2) pair loop when this group was written; the broad
# phase that replaced it is what baseline-...-20261102-shatter-sweep-heavy
# measures, on these same cases, so the two runs compare row for row.
#
# Density is a separate axis on purpose. Packing more segments into the same
# square would raise n and density together, which is the exact confound
# sort_curves_density had to be added to undo -- so the n sweep grows the extent
# as sqrt(n) and holds segments per unit area fixed, and density gets its own
# sweep at fixed n.
#
# There is no native-harness counterpart, and that reasoning has expired. It was
# written when the bridge marshalled 6n doubles against a pair loop costing ~28
# ns per pair -- under 0.1% of the call. With the broad phase the native call at
# n = 16,000 is ~4.4 ms (timed ad hoc, not committed) inside a ~42 ms Python
# call, so the bridge is now most of what this group measures.

SHATTER_BASE_N = 1000
SHATTER_SIZES = [500, 1000, 2000, 4000, 8000, 16000]
SHATTER_N = 2000              # fixed n for every axis that is not n itself
SHATTER_EXTENT = 250.0        # contacts on most segments, but still under 2n output
SHATTER_GAP = 2.0
SHATTER_TOL = 0.01


def fixed_density_extent(n):
    """Extent holding segments per unit area constant as n grows."""
    return EXTENT * math.sqrt(float(n) / SHATTER_BASE_N)


def make_polyline_segments(n_curves, per_curve, seed=1, extent=None):
    """Exploded polylines: consecutive segments share a vertex and an owner, so
    the joint-skipping path is exercised rather than assumed. Returns
    (segments, owners), which is what the GH component hands the wrapper."""
    rng = random.Random(seed)
    span = EXTENT if extent is None else extent
    segs, owners = [], []
    for owner in range(n_curves):
        x, y = rng.uniform(0, span), rng.uniform(0, span)
        for _ in range(per_curve):
            ang, ln = rng.uniform(0, 2 * math.pi), rng.uniform(5.0, 20.0)
            nx, ny = x + math.cos(ang) * ln, y + math.sin(ang) * ln
            segs.append(_Seg(x, y, nx, ny))
            owners.append(owner)
            x, y = nx, ny
    return segs, owners


def _observe_shatter(call):
    """out_n on its own is misleading: past a certain density it FALLS, because
    segments stop being cut and start being consumed whole. `vanished` is what
    tells those two apart. `regrew` flags the rows where out_n passed the
    wrapper's first guess and the native call therefore ran twice."""
    def observe(payload):
        out = call(payload)
        n = len(out)
        out_n = sum(len(g) for g in out)
        return dict(out_n=out_n,
                    vanished=sum(1 for g in out if not g),
                    regrew=int(out_n > SHATTER_FIRST_GUESS * n))
    return observe


def _plain(segs):
    return shatter_at_crossings_native(segs, SHATTER_GAP, touch_tol=SHATTER_TOL)


def _shatter_cases():
    cases = []

    # 1. n, density held fixed -- the quadratic on its own
    for n in SHATTER_SIZES:
        cases.append(Case(
            "shatter_at_crossings", "n%d" % n,
            setup=lambda n=n: make_segments(n, seed=11, extent=fixed_density_extent(n)),
            run=_plain, observe=_observe_shatter(_plain),
            axis=dict(n=n, extent=int(fixed_density_extent(n)),
                      shape="loose", self_x=0, gap=SHATTER_GAP),
            heavy=(n > 4000),
        ))

    # 2. density, n held fixed -- the pair loop did not care how many contacts
    #    there were; the broad phase does, since denser boxes overlap more, so
    #    this axis now moves with the candidate count as well as output size.
    for ext in [2000.0, 1000.0, 500.0, 250.0, 125.0]:
        cases.append(Case(
            "shatter_at_crossings", "extent%d" % int(ext),
            setup=lambda ext=ext: make_segments(SHATTER_N, seed=11, extent=ext),
            run=_plain, observe=_observe_shatter(_plain),
            axis=dict(n=SHATTER_N, extent=int(ext),
                      shape="loose", self_x=0, gap=SHATTER_GAP),
        ))

    # 3. the same segment count arriving as polylines, with self-crossing off
    #    then on. Off skips every same-owner pair, which is 100 x 20 x 19 / 2 of
    #    them here; on skips only the joints. The difference is what owners buy.
    for test_self in (False, True):
        call = (lambda ts: lambda p: shatter_at_crossings_native(
            p[0], SHATTER_GAP, touch_tol=SHATTER_TOL,
            segment_owner=p[1], test_self=ts))(test_self)
        cases.append(Case(
            "shatter_at_crossings", "polyline_self%d" % int(test_self),
            setup=lambda: make_polyline_segments(100, 20, seed=11, extent=SHATTER_EXTENT),
            run=call,
            observe=(lambda c: lambda p: _observe_shatter(c)(p))(call),
            axis=dict(n=SHATTER_N, extent=int(SHATTER_EXTENT),
                      shape="polyline", self_x=int(test_self), gap=SHATTER_GAP),
        ))

    # 4. gap width. It cannot change how many pairs are tested, only how much
    #    each contact removes -- recorded to confirm that rather than assume it,
    #    and because it is what drives segments to vanish.
    for gap in [0.0, 2.0, 8.0, 32.0]:
        call = (lambda g: lambda segs: shatter_at_crossings_native(
            segs, g, touch_tol=SHATTER_TOL))(gap)
        cases.append(Case(
            "shatter_at_crossings", "gap%g" % gap,
            setup=lambda: make_segments(SHATTER_N, seed=11, extent=SHATTER_EXTENT),
            run=call,
            observe=(lambda c: lambda p: _observe_shatter(c)(p))(call),
            axis=dict(n=SHATTER_N, extent=int(SHATTER_EXTENT),
                      shape="loose", self_x=0, gap=gap),
        ))

    return cases


# --- point_in_curve --------------------------------------------------------
# No row of this group existed before 2026-11-16, so the only claim the function
# had made was "the plane transform happens once per call", never that the
# classification itself was cheap. These are the rows the edge index will be
# measured against, which is why they are recorded before it is written.

PIC_EPS = 1e-9          # the native side's floor on tol; mirrored for `scanned`
PIC_SIZES = [1000, 2000, 4000, 8000, 16000, 32000, 64000]
PIC_N = 8000            # query points, when n is not the axis
PIC_V = 1000            # curve vertices, when V is not the axis


def _pic_call(payload):
    verts, query, tol = payload
    return point_in_curve_native([verts], query, tol=tol)


def _observe_point_in_curve(payload):
    """Time alone cannot read these rows. A point the bounding box rejects comes
    back PIC_OUTSIDE without an edge ever being read, and so does a point that
    was scanned and found outside -- the output cannot tell them apart, and the
    cost is proportional to the second kind. `scanned` is that count, recomputed
    here from the box rather than taken from the output.

    The histogram is a cost term too, not a description: a scanned point always
    pays distance_to_polyline, and pays the crossing count on top of it only
    when it is not coincident."""
    verts, query, tol = payload
    rel, status = _pic_call(payload)

    xs = [p[0] for p in verts]
    ys = [p[1] for p in verts]
    lo_x, hi_x, lo_y, hi_y = min(xs), max(xs), min(ys), max(ys)
    span = tol if tol > PIC_EPS else PIC_EPS

    scanned = 0
    for qx, qy in query:
        dx = max(lo_x - qx, 0.0, qx - hi_x)
        dy = max(lo_y - qy, 0.0, qy - hi_y)
        if math.hypot(dx, dy) <= span:
            scanned += 1

    return dict(scanned=scanned,
                outside=sum(1 for r in rel if r == PIC_OUTSIDE),
                coincident=sum(1 for r in rel if r == PIC_COINCIDENT),
                inside=sum(1 for r in rel if r == PIC_INSIDE),
                status=status)


def _pic_case(name, n, v, shape, where, tol, heavy=False):
    return Case(
        "point_in_curve", name,
        setup=lambda: (make_closed_curve(v, shape),
                       make_query_points(n, where), tol),
        run=_pic_call, observe=_observe_point_in_curve,
        axis=dict(n=n, v=v, shape=shape, where=where, tol=tol),
        heavy=heavy,
    )


def _point_in_curve_cases():
    cases = []

    # 1. n, with V and the placement fixed. Linear is the expectation; the row
    #    that matters is the smallest one, read against the native harness,
    #    because the wrapper rebuilds both arrays per call and at a short query
    #    list against a dense curve that marshaling could outweigh the work.
    for n in PIC_SIZES:
        cases.append(_pic_case("n%d" % n, n, PIC_V, "circle", "uniform", 0.0,
                               heavy=(n > 16000)))

    # 2. V, with n fixed -- the axis the edge index attacks. Today every scanned
    #    point reads all V edges twice, so this is expected to be linear in V as
    #    well, and that product is the whole case for an index.
    for v in [100, 250, 1000, 4000, 16000]:
        cases.append(_pic_case("v%d" % v, PIC_N, v, "circle", "uniform", 0.0,
                               heavy=(v > 4000)))

    # 3. where the query points fall -- the axis that isolates the bounding-box
    #    rejection. `outside` is the floor, where the box answers everything and
    #    no edge is read; `inside` the ceiling. `uniform` is the control but sits
    #    at the ceiling too, because it is drawn over the box and so never gives
    #    the rejection anything to do; `spread` is the mixed row and the one a
    #    real job resembles. Read these four against `scanned`, not against each
    #    other: it is the count the cost tracks.
    for where in ["uniform", "spread", "inside", "outside"]:
        cases.append(_pic_case("where_%s" % where, PIC_N, PIC_V,
                               "circle", where, 0.0))

    # 4. shape, with n, V and placement fixed. Recorded now because the index's
    #    benefit is V over the active set size: the circle's is 2 at every V and
    #    the comb's is a large fraction of V, so this axis is the difference
    #    between "the index helped" and "that shape was always cheaper". Added
    #    with the index instead, the two could not be told apart.
    for shape in ["circle", "comb"]:
        cases.append(_pic_case("shape_%s" % shape, PIC_N, PIC_V,
                               shape, "uniform", 0.0))

    # 5. tol. It cannot change how many edges a scanned point reads, only how
    #    many points the box lets through and how many land on the boundary
    #    rather than in it -- recorded to confirm that rather than assume it.
    for tol in [0.0, 1.0, 10.0, 100.0]:
        cases.append(_pic_case("tol%g" % tol, PIC_N, PIC_V,
                               "circle", "uniform", tol))

    return cases


GROUPS = ["sort_points", "sort_curves", "sort_curves_convergence",
          "sort_curves_prune_check", "sort_points_convergence",
          "sort_points_prune_check", "redistribute_arc_lengths",
          "build_turn_waypoints", "shatter_at_crossings", "point_in_curve"]


def all_cases():
    return (_sort_points_cases() + _sort_curves_cases()
            + _convergence_cases() + _prune_check_cases()
            + _points_convergence_cases() + _points_prune_check_cases()
            + _redistribute_cases() + _turn_cases()
            + _shatter_cases() + _point_in_curve_cases())
