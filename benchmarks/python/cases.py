"""Representative inputs per function -- what counts as a size, separate from
run.py's how it gets timed. Each function's axis differs; see ../README.md.
Stand-in geometry mirrors tests/, so no Rhino types are needed."""

import glob
import json
import math
import os
import random

from geomseq_core.geometry_utils import (
    build_turn_waypoints_native,
    redistribute_lookups_native,
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


def make_segments(n, seed=1, min_len=5.0, max_len=20.0):
    """Short randomly-oriented segments -- stitches/toolpath strokes, not a mesh."""
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        x0, y0 = rng.uniform(0, EXTENT), rng.uniform(0, EXTENT)
        ang, ln = rng.uniform(0, 2 * math.pi), rng.uniform(min_len, max_len)
        out.append(_Seg(x0, y0, x0 + math.cos(ang) * ln, y0 + math.sin(ang) * ln))
    return out


# --------------------------------------------------------------------------
# Recorded fixtures -- optional, additive; see ../../docs/benchmark-fixtures.md
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


def even_lookups(total_length, n):
    """Evenly spaced arc-length samples, as a real curve division would give."""
    return [total_length * i / (n - 1) for i in range(n)]


def spread_corners(n_lookups, count):
    """`count` corner indices spread over the interior of a lookup list."""
    if count <= 0:
        return None
    step = (n_lookups - 2) / float(count)
    return [int(1 + i * step) for i in range(count)]


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
# Sizes straddle TWO_OPT_WINDOW_THRESHOLD (10,000); same-n comparison lives in
# the crossover group. if_flip=False makes the native side skip 2-opt outright.

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
                    axis["two_opt_path"] = "exhaustive" if n <= 10000 else "windowed"
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


# --- sort_curves: windowed vs exhaustive at the SAME n -----------------------
# Only possible since two_opt_mode was added; under `auto` the paths never
# overlap. Travel is observed too -- windowed buys speed with tour quality.

CROSSOVER_SIZES = [2000, 5000, 8000, 12000]
CROSSOVER_HEAVY_ABOVE = 5000


def travel_distance(curves):
    """Sum of gaps: end of one curve to start of the next (same as tests/)."""
    return sum(math.hypot(c.PointAtStart.X - p.PointAtEnd.X,
                          c.PointAtStart.Y - p.PointAtEnd.Y)
               for p, c in zip(curves, curves[1:]))


def _crossover_cases():
    cases = []
    for n in CROSSOVER_SIZES:
        for mode, label in [(1, "exhaustive"), (2, "windowed")]:
            def run(c, mode=mode):
                ordered, _ = sort_curves_native(
                    c, use_two_opt=True, two_opt_mode=mode,
                    two_opt_max_passes=MAX_PASSES, knn_k=KNN_K)
                return ordered

            def observe(c, mode=mode):
                ordered, _ = sort_curves_native(
                    c, use_two_opt=True, two_opt_mode=mode,
                    two_opt_max_passes=MAX_PASSES, knn_k=KNN_K)
                return dict(travel=round(travel_distance(ordered), 1))

            cases.append(Case(
                "sort_curves_crossover", "uniform_%s_n%d" % (label, n),
                setup=lambda n=n: make_segments(n),
                run=run, observe=observe,
                axis=dict(data="uniform", n=n, path=label,
                          auto_would_pick=("exhaustive" if n <= 10000 else "windowed")),
                heavy=(n > CROSSOVER_HEAVY_ABOVE),
            ))
    return cases


# --- redistribute_lookups --------------------------------------------------
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
        call = lambda lk: redistribute_lookups_native(lk, 2.0, 8.0, 0, 50.0)
        cases.append(Case(
            "redistribute_lookups", "input_n%d" % n_in,
            setup=lambda n_in=n_in: even_lookups(REDIST_TOTAL, n_in),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=n_in, band="2-8", corners=0, mode=0),
        ))

    # 2. output density, input held fixed
    for low, high in [(8.0, 20.0), (2.0, 8.0), (0.5, 2.0), (0.2, 0.8)]:
        call = (lambda low, high: lambda lk: redistribute_lookups_native(
            lk, low, high, 0, 50.0))(low, high)
        cases.append(Case(
            "redistribute_lookups", "band_%g_%g" % (low, high),
            setup=lambda: even_lookups(REDIST_TOTAL, 10001),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=10001, band="%g-%g" % (low, high), corners=0, mode=0),
        ))

    # 3. corner count -- each corner forces a look-ahead and a rescaled step
    for nc in [0, 10, 100, 1000]:
        call = (lambda nc: lambda lk: redistribute_lookups_native(
            lk, 0.5, 2.0, 0, 50.0, corner_indices=spread_corners(10001, nc)))(nc)
        cases.append(Case(
            "redistribute_lookups", "corners_%d" % nc,
            setup=lambda: even_lookups(REDIST_TOTAL, 10001),
            run=call, observe=_observe_out_n(call),
            axis=dict(input_n=10001, band="0.5-2", corners=nc, mode=0),
        ))

    # 4. mode 0 vs 1 -- expected flat, recorded to confirm rather than assume
    for mode in [0, 1]:
        call = (lambda mode: lambda lk: redistribute_lookups_native(
            lk, 0.5, 2.0, mode, 50.0))(mode)
        cases.append(Case(
            "redistribute_lookups", "mode%d" % mode,
            setup=lambda: even_lookups(REDIST_TOTAL, 10001),
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

GROUPS = ["sort_points", "sort_curves", "sort_curves_crossover",
          "redistribute_lookups", "build_turn_waypoints"]


def all_cases():
    return (_sort_points_cases() + _sort_curves_cases() + _crossover_cases()
            + _redistribute_cases() + _turn_cases())
