"""GH entry point: browse benchmarks/fixtures/ and pick one by index.

Feeds `file_path` into read_fixture.py, so the pair works as browse -> load
without retyping paths. Tooling, like the other two scripts here: it touches no
geomseq_core code.

Inputs   folder_path (str)  directory holding the fixtures, e.g.
                            ...\\benchmarks\\fixtures
         select_ix   (int)  index from the table printed on `out`; wraps, so
                            any number is valid
         kind_filter (str)  optional; "points" or "curves" to list only those
         rescan      (bool) True to rescan the folder (NOT named `reload`: that is a
                            builtin in IronPython, so an unconnected input would
                            resolve to it and read as permanently True)
Outputs  file_path, name, kind, info

The index table goes to the component's built-in `out` socket via print().

Do not name an input `dir`, `type`, `id` or any other builtin -- an unconnected
input is simply not injected, so the name quietly resolves to the builtin
instead of raising NameError, and the failure surfaces somewhere confusing.

Fixture format: benchmarks/README.md ("Adding a fixture").
"""

import json
import os
from pathlib import Path

import scriptcontext as sc
from Grasshopper.Kernel import GH_RuntimeMessageLevel as RML

KINDS = ("points", "curves")   # what benchmarks/python/cases.py knows how to load


def read_header(path):
    """Metadata only, without touching `data`.

    Fixtures are multi-MB and export_fixture.py writes the metadata one key per
    line with `data` on a single line, precisely so a listing can stop reading
    at that point. Parsing ten files in full just to show a table would cost
    ~9 MB of JSON for a header nobody needed loaded.
    """
    lines = []
    with open(path, encoding="utf-8") as fh:
        for line in fh:
            if line.lstrip().startswith('"data"'):
                break
            lines.append(line)
    text = "".join(lines).rstrip().rstrip(",")
    # The outer brace is on the line we stopped before. Cannot decide by looking
    # at the last character: a `params` block ends in `}` too, so the truncation
    # is closed either way. Try it closed, fall back to as-read for a file that
    # had no `data` line to stop at.
    try:
        return json.loads(text + "\n}")
    except ValueError:
        return json.loads(text)


def summarize(path):
    """(kind, name, note) for the table -- never raises; a bad file is listed
    with its error rather than dropped, since a missing row is harder to
    notice than a broken one."""
    stem = os.path.basename(path)
    try:
        doc = read_header(path)
    except (ValueError, OSError) as exc:
        return "?", stem, "UNREADABLE HEADER: %s" % exc

    kind = doc.get("kind") or "?"
    name = doc.get("name") or stem
    bits = []
    params = doc.get("params") or {}
    if params.get("n"):
        bits.append("n=%s" % params["n"])
    size = os.path.getsize(path)
    bits.append("%.1f MB" % (size / 1048576.0) if size >= 1048576
                else "%.0f KB" % (size / 1024.0))
    if doc.get("units"):
        bits.append(doc["units"])

    # The harness globs on the filename prefix but reads the `kind` field, so a
    # file whose name and field disagree loads as one thing and benchmarks as
    # another. Cheap to catch here; invisible in a timing table.
    if kind != stem.split("_", 1)[0]:
        bits.append("!! kind/filename mismatch")
    return kind, name, ", ".join(bits)


# Top-level, not guarded by __name__: a GH script component does not
# necessarily run as "__main__", and a guard that fails is silent -- no output,
# no error, which is the worst failure mode for a component.
file_path = None
name = None
kind = None
info = ""

folder = Path(folder_path) if folder_path else None  # type: ignore # noqa: F821

if folder is None or not folder.is_dir():
    ghenv.Component.AddRuntimeMessage(                # type: ignore # noqa: F821
        RML.Error, "Directory does not exist: %s" % folder)
    names = []
else:
    wanted = (kind_filter or "").strip().lower()      # type: ignore # noqa: F821
    if wanted and wanted not in KINDS:
        ghenv.Component.AddRuntimeMessage(            # type: ignore # noqa: F821
            RML.Warning,
            "kind_filter must be one of %s, got %r -- ignoring."
            % (KINDS, wanted))
        wanted = ""

    cache_key = "FIXTURE_LST_%s_%s" % (folder, wanted)
    if rescan or cache_key not in sc.sticky:          # type: ignore # noqa: F821
        pattern = "%s_*.json" % wanted if wanted else "*.json"
        names = sorted(p.name for p in folder.glob(pattern))
        sc.sticky[cache_key] = names
    else:
        names = sc.sticky[cache_key]

if not names:
    if folder is not None and folder.is_dir():
        ghenv.Component.AddRuntimeMessage(            # type: ignore # noqa: F821
            RML.Remark, "No fixture JSON found in %s" % folder)
    print("[empty]  (%s)" % folder)
else:
    rows = [(n,) + summarize(str(folder / n)) for n in names]
    width = max(len(r[0]) for r in rows)

    print("index | %-*s | kind   | dataset" % (width, "file"))
    print("-" * (width + 62))
    for i, (fname, fkind, fname_field, note) in enumerate(rows):
        print("{:>5} | {:<{w}} | {:<6} | {} -- {}".format(
            i, fname, fkind, fname_field, note, w=width))

    # Unconnected int inputs arrive as None, and None % int raises. Wrap with
    # modulo so any index -- negative or past the end -- still selects a row.
    pick = int(select_ix or 0) % len(rows)            # type: ignore # noqa: F821
    fname, kind, name, note = rows[pick]
    file_path = str(folder / fname)
    info = "[%d] %s\nkind: %s   dataset: %s\n%s" % (
        pick, fname, kind, name, note)
    print("\nselected [%d] %s" % (pick, fname))
