"""GH entry point: load a benchmark fixture JSON back into Rhino geometry.

The inverse of export_fixture.py, and the same kind of tooling: it touches no
geomseq_core code, only Rhino and json. Use it to look at what a fixture
actually contains before trusting a number measured on it -- clustering,
extent and existing order are all obvious on screen and invisible in a table.

Inputs   read_json  (bool) load on True; leave False while wiring things up
         file_path  (str)  full path to one fixture, e.g.
                           ...\\benchmarks\\fixtures\\points_grid.json
         sample_n   (int)  optional; 0 or unconnected loads every row. Otherwise
                           draws this many rows the SAME way the harness does,
                           so what you see is what gets benchmarked.
         seed       (int)  optional; matches cases.sample's default of 1
Outputs  geo, kind, count, info, log

`geo` is Point3d for kind="points" and LineCurve for kind="curves", so the
output can go straight into the same components the exporter was fed from.
Z is always 0: the fixture format is 2D and never stored it.

Do not name an input `dir`, `type`, `id`, `file` or any other builtin -- an
unconnected input is simply not injected, so the name quietly resolves to the
builtin instead of raising NameError, and the failure surfaces somewhere
confusing.

Fixture format: benchmarks/README.md ("Adding a fixture").
"""

import json
import os
import random

import Rhino.Geometry as rg

KINDS = ("points", "curves")   # what benchmarks/python/cases.py knows how to load
DEFAULT_SEED = 1               # matches cases.sample


def sample(data, n, seed=DEFAULT_SEED):
    """`n` rows drawn from `data`, seeded, IN THE SOURCE'S OWN ORDER.

    Deliberately identical to sample() in benchmarks/python/cases.py -- if this
    drew differently, the geometry on screen would not be the geometry that was
    timed. Sampling rather than slicing keeps the distribution comparable across
    n; the indices are re-sorted because input order is itself under test, and
    random.sample hands its picks back shuffled.
    """
    if n >= len(data):
        return list(data)
    idx = sorted(random.Random(seed).sample(range(len(data)), n))
    return [data[i] for i in idx]


def point_geo(row):
    return rg.Point3d(row[0], row[1], 0.0)


def curve_geo(row):
    return rg.LineCurve(rg.Point3d(row[0], row[1], 0.0),
                        rg.Point3d(row[2], row[3], 0.0))


def describe(doc, path):
    """The provenance header, which is the point of filling it in on export."""
    lines = ["file: %s" % os.path.basename(path)]
    for key in ("kind", "name", "source", "units", "notes"):
        value = doc.get(key)
        if value:
            lines.append("%s: %s" % (key, value))
    params = doc.get("params")
    if params:
        lines.append("params: %s" % json.dumps(params, sort_keys=True))
    lines.append("rows in file: %d" % len(doc.get("data") or []))
    return "\n".join(lines)


# Top-level, not guarded by __name__: a GH script component does not
# necessarily run as "__main__", and a guard that fails is silent -- no
# output, no error, which is the worst failure mode for a component.
log = []
geo = []
kind = None
count = 0
info = ""

if not read_json:                               # type: ignore # noqa: F821
    log.append("read_json is False -- nothing loaded.")
elif not isinstance(file_path, str) or not os.path.isfile(file_path):  # type: ignore # noqa: F821
    log.append("file_path is not an existing file: %r" % (file_path,))  # type: ignore # noqa: F821
else:
    with open(file_path, encoding="utf-8") as fh:  # type: ignore # noqa: F821
        doc = json.load(fh)

    kind = doc.get("kind")
    data = doc.get("data") or []
    info = describe(doc, file_path)             # type: ignore # noqa: F821

    # The harness globs on the filename prefix but reads the `kind` field, so
    # a file whose name and field disagree loads as one thing and benchmarks
    # as another. Worth catching here rather than in a timing table.
    prefix = os.path.basename(file_path).split("_", 1)[0]  # type: ignore # noqa: F821
    if kind != prefix:
        log.append("WARNING: kind is %r but the filename says %r -- the "
                   "harness globs on the filename." % (kind, prefix))

    if kind not in KINDS:
        log.append("kind must be one of %s, got %r." % (KINDS, kind))
    elif not data:
        log.append("no rows in 'data'.")
    else:
        wanted = int(sample_n or 0)             # type: ignore # noqa: F821
        if wanted > len(data):
            log.append("sample_n %d exceeds the file's %d rows -- the "
                       "harness SKIPS a sweep size it cannot fill rather "
                       "than truncating, so loading all instead."
                       % (wanted, len(data)))
            wanted = 0
        rows = data if wanted <= 0 else sample(
            data, wanted, int(seed) if seed else DEFAULT_SEED)  # type: ignore # noqa: F821

        build = point_geo if kind == "points" else curve_geo
        try:
            geo = [build(r) for r in rows]
        except (TypeError, IndexError):
            geo = []
            log.append("row shape does not match kind=%r -- points want "
                       "[x, y] and curves [x0, y0, x1, y1]." % (kind,))

        count = len(geo)
        if count:
            log.append("loaded %d of %d %s%s"
                       % (count, len(data), kind,
                          "" if wanted <= 0 else " (seeded sample, source order)"))

log = "\n".join(log)
