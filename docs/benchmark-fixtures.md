# Benchmark fixtures

Recorded inputs for `benchmarks/`, living in `benchmarks/fixtures/`. Separate
from `tests/fixtures/` on purpose — the two have opposite requirements:

| | `tests/fixtures/` | `benchmarks/fixtures/` |
|---|---|---|
| wants | small, awkward, edge cases | large, regular, scalable |
| size | a few hundred entries | tens of thousands |
| packing | many cases per file, all loaded | **one dataset per file**, loaded on demand |

That last row is the practical reason for the split. `tests/test_sort_points.py`
turns *every* entry in its fixture into a parametrized test with `use_two_opt=True`.
A 64,000-point benchmark dataset dropped in there would become a single test
that runs exhaustive 2-opt at 64k — roughly 3½ minutes, against the suite's
current 0.06 s.

## Format

One JSON file per dataset, named `points_<name>.json` or `curves_<name>.json`.
The kind is in the filename so the harness can list fixtures without parsing
them.

```json
{
  "kind": "points",
  "name": "real_lettering",
  "source": "exported from <file/job>, <date>",
  "units": "mm",
  "notes": "single-colour satin lettering, one hoop",
  "data": [[x, y], [x, y], ...]
}
```

`data` rows are `[x, y]` for points and `[x0, y0, x1, y1]` for curves — the
same shape `tests/fixtures/sort_curves_cases.json` already uses. Only `kind`
and `data` are read by the harness; the rest is provenance, and worth filling
in because a number is not interpretable without knowing what produced it.

The metadata keys are written one per line so the header is readable; `data`
stays on a single line. Indenting the array as well costs 2.4× the file size
and a quarter of a million lines of coordinates, for nothing anyone reads —
at 64,000 points that is 2.90 MB against 1.19 MB.

## How the harness uses them

Fixtures are **additive, not a replacement**. Synthetic uniform data stays as
the control: when a measurement moves, you need to be able to tell whether the
algorithm changed or the data did. Every fixture adds a parallel set of rows
tagged with its name in the `data` column, so uniform and real sit side by side
in the same table.

Sizes are drawn by *seeded sampling*, not slicing. A slice of a real design is
one region of it, so n=1,000 and n=64,000 would have different characters and
the scaling exponent would be meaningless. Sampling keeps the distribution —
clustering included — comparable across n. A sweep size larger than the fixture
is skipped rather than silently truncated.

Nothing needs configuring: drop a file in and its cases appear.

## Datasets worth recording

Roughly in priority order. The point of each is to break an assumption the
synthetic generator quietly makes — uniform density, uniform scale, square
extent, evenly spaced arc lengths.

### 1. Clustered real toolpath — `points_*` and `curves_*`

The headline gap. `make_points` scatters uniformly over a 1000×1000 square,
which is the friendliest possible case for a kd-tree. Real stitch and cut data
is locally dense and globally sparse.

This directly probes the degradation the repo README flags as unaddressed: the
greedy k-NN phase filters already-used points out of a *static* tree, so the
search window has to keep expanding to find an unused neighbour — and how badly
that bites depends on clustering. Uniform data may be hiding the worst case.

### 2. Non-uniform density — `points_*`

A design with both a dense region and a sparse one in the same file (fill plus
outline, say). Stresses the kd-tree differently from uniform clustering, and is
what makes the greedy phase's window expansion pathological rather than merely
slower.

### 3. Grid / hatch structured — `points_*`

Infill and hatching produce many collinear, equidistant points and exact ties in
nearest-neighbour distance. Tie-breaking behaviour is untested at scale and does
not occur at all in float-random data.

### 4. Mixed curve lengths — `curves_*`

`make_segments` draws lengths uniformly from 5–20. Real toolpaths mix long
travel strokes with very short stitches, often an order of magnitude apart. That
changes how much 2-opt has to gain and how often reversal matters.

### 5. Long thin extent — `points_*` or `curves_*`

A border or a single row of lettering — far wider than tall. The synthetic
generator always produces a square, and a square is where kd-tree splits behave
best.

### 6. Non-uniform arc-length lookups — `lookups_*`

`redistribute_lookups` is currently benchmarked only against `even_lookups`,
perfectly evenly spaced. Real `rhino_utils/divide_curves.py` output on a curve
with varying curvature is not evenly spaced. Would need a `lookups_` kind added
to the loader, plus real corner indices from an actual polyline.

### 7. Real turn geometry — `turns_*`

`build_turn_waypoints` uses four hand-written cases copied from the tests. Real
E→S pairs harvested from a toolpath would cover the documented failure mode: a
gap small relative to `step_len`, where the exit→entry junction kinks past
`theta_max_deg` (see the repo README's Notes).

## Before committing real data

The repository is public. Coordinates exported from a client job are that
client's design, recoverable from the fixture. Prefer a pattern you own, or one
that is already public, over anything from live work.
