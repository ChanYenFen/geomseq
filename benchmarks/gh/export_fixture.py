"""GH entry point: dump selected geometry to a benchmark fixture JSON.

Tooling, not part of the shipped library -- it touches no geomseq_core code,
only Rhino and json. Format and wanted datasets: docs/benchmark-fixtures.md

Inputs   save_json (bool)  write on True; leave False while wiring things up
         geo       (list)  Point3d when kind="points", Curve when kind="curves".
                           Set the GH input to List Access and NO type hint, so
                           one component can do both.
         kind      (str)   "points" or "curves" -- also the filename prefix the
                           harness globs for
         file_name (str)   label for this dataset, e.g. "real_lettering"
         save_dir  (str)   existing directory, e.g. ...\\benchmarks\\fixtures
Outputs  path, log

Do not name an input `dir`, `type`, `id` or any other builtin: an unconnected
input is simply not injected, so the name quietly resolves to the builtin
instead of raising NameError, and the failure surfaces somewhere confusing.

IMPORTANT: feed the geometry in its ORIGINAL, UNSORTED order. Exporting the
output of a previous sort makes the benchmark measure sorting an already-sorted
input -- greedy has nothing to do and 2-opt converges early, so the numbers come
out flattering and meaningless.

Only curve endpoints are recorded, which is all sort_curves itself reads.
"""

import json
import os
import re

KINDS = ("points", "curves")   # what benchmarks/python/cases.py knows how to load
PRECISION = 4                  # decimals; 1e-4 mm is well past any CAD tolerance
Z_TOLERANCE = 1e-9             # above this, Z is real data and dropping it matters


def slugify(text):
    """'Real Lettering v2' -> 'real_lettering_v2'; becomes the data column label."""
    return re.sub(r"[^a-z0-9]+", "_", (text or "").lower()).strip("_")


def point_row(p):
    return [round(p.X, PRECISION), round(p.Y, PRECISION)]


def curve_row(c):
    s, e = c.PointAtStart, c.PointAtEnd
    return [round(s.X, PRECISION), round(s.Y, PRECISION),
            round(e.X, PRECISION), round(e.Y, PRECISION)]


def z_values(kind, items):
    if kind == "points":
        return [abs(p.Z) for p in items]
    return [z for c in items for z in (abs(c.PointAtStart.Z), abs(c.PointAtEnd.Z))]


if __name__ == "__main__":
    log = []
    path = None

    label = slugify(file_name)                      # type: ignore # noqa: F821
    # A file_name of "points_foo" would otherwise become points_points_foo.json.
    for k in KINDS:
        if label.startswith(k + "_"):
            label = label[len(k) + 1:]

    if not save_json:                               # type: ignore # noqa: F821
        log.append("save_json is False -- nothing written.")
    elif not geo:                                   # type: ignore # noqa: F821
        log.append("no geometry supplied.")
    elif kind not in KINDS:                         # type: ignore # noqa: F821
        log.append("kind must be one of %s, got %r -- the harness globs on that "
                   "prefix and would not find the file." % (KINDS, kind))  # type: ignore # noqa: F821
    elif not label:
        log.append("file_name is empty (or slugifies to nothing).")
    elif not isinstance(save_dir, str) or not os.path.isdir(save_dir):  # type: ignore # noqa: F821
        log.append("save_dir is not an existing directory: %r" % (save_dir,))  # type: ignore # noqa: F821
    else:
        items = list(geo)                           # type: ignore # noqa: F821

        # The fixture format is 2D, but the core does receive Z. If the geometry
        # is off the world XY plane, dropping Z here would make the fixture
        # disagree with what Rhino actually feeds the DLL.
        try:
            max_z = max(z_values(kind, items))      # type: ignore # noqa: F821
        except AttributeError:
            max_z = 0.0
            log.append("WARNING: could not read Z -- does geo match kind=%r?"
                       % (kind,))                   # type: ignore # noqa: F821
        if max_z > Z_TOLERANCE:
            log.append("WARNING: max |Z| is %.6g -- Z is dropped by this format. "
                       "Project to XY first if that matters." % max_z)

        row = point_row if kind == "points" else curve_row   # type: ignore # noqa: F821
        data = [row(g) for g in items]

        payload = {
            "kind": kind,                           # type: ignore # noqa: F821
            "name": label,
            "source": "exported via benchmarks/gh/export_fixture.py",
            "units": "",   # fill in, or read Rhino.RhinoDoc.ActiveDoc.ModelUnitSystem
            "notes": "",   # describe the pattern -- a number is not interpretable without it
            "data": data,
        }

        path = os.path.join(save_dir, "%s_%s.json" % (kind, label))  # type: ignore # noqa: F821
        replaced = os.path.exists(path)

        # Metadata one key per line so the header is scannable; `data` stays on
        # a single line. Indenting it too costs 2.4x the size and 256k lines of
        # coordinates nobody reads (measured at 64k points).
        meta = {k: v for k, v in payload.items() if k != "data"}
        head = json.dumps(meta, indent=2, ensure_ascii=False)[:-2]  # drop "\n}"
        with open(path, "w", encoding="utf-8") as fh:
            fh.write(head + ',\n  "data": '
                     + json.dumps(data, separators=(",", ":")) + "\n}\n")

        log.append("%s %d %s -> %s"
                   % ("REPLACED" if replaced else "wrote", len(data), kind, path))  # type: ignore # noqa: F821
        log.append("fill in 'units' and 'notes'; check the input was unsorted.")

    log = "\n".join(log)
