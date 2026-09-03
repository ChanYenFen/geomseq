"""Align the native and Python baselines: per case, the native cost, the full
Python-side cost, and the bridge between them (ctypes + marshaling + wrapper).
Usage: python benchmarks/compare.py results/native.json results/python.json"""

import json
import sys

# Fields identifying the same case on both sides. The native harness has no
# input_n axis for redistribute_lookups, so several Python rows share one
# native row -- which is exactly what makes that column worth showing.
KEYS = {
    "build_turn_waypoints": ["geometry", "theta_max_deg"],
    "redistribute_lookups": ["band", "corners", "mode"],
}
# Python-only axes, shown as extra columns and flagged as unmatched.
PY_ONLY = {"redistribute_lookups": ["input_n"]}


def norm(v):
    """30.0 and 30 must hash alike; the two harnesses format numbers differently."""
    if isinstance(v, float) and v == int(v):
        return str(int(v))
    return str(v)


def key_of(d, fields):
    return tuple(norm(d.get(f)) for f in fields)


def load_native(path):
    # utf-8-sig, not utf-8: redirecting bench_core's stdout through PowerShell
    # prepends a BOM. Reads plain UTF-8 (every other platform) just as well.
    with open(path, encoding="utf-8-sig") as fh:
        doc = json.load(fh)
    out = {}
    for r in doc["records"]:
        fields = KEYS[r["group"]]
        out[(r["group"],) + key_of(r["key"], fields)] = r
    return doc["env"], out


def load_python(path):
    # utf-8-sig, not utf-8: redirecting bench_core's stdout through PowerShell
    # prepends a BOM. Reads plain UTF-8 (every other platform) just as well.
    with open(path, encoding="utf-8-sig") as fh:
        doc = json.load(fh)
    rows = []
    for r in doc["records"]:
        if r.get("skipped") or r["group"] not in KEYS:
            continue
        rows.append(dict(
            group=r["group"],
            axis=r["axis"],
            out_n=(r.get("observed") or {}).get("out_n"),
            per_call_us=r["best_seconds"] / r["batch"] * 1e6,
        ))
    return doc["env"], rows


def main(argv):
    if len(argv) < 3:
        print(__doc__)
        return 2
    n_env, native = load_native(argv[1])
    p_env, py_rows = load_python(argv[2])

    out = ["# native vs Python -- where each call's time goes", ""]
    out.append("`native` is the algorithm alone (`benchmarks/native/bench_core`). "
               "`python` is the same call through the wrapper "
               "(`benchmarks/python/run.py`): ctypes, marshaling and wrapper "
               "included. `bridge` is the difference.")
    out.append("")
    out.append("| | |")
    out.append("|---|---|")
    out.append("| Native toolchain | %s |" % n_env.get("toolchain"))
    out.append("| Python | %s on %s %s |"
               % (p_env.get("python"), p_env.get("system"), p_env.get("machine")))
    b = p_env.get("binary") or {}
    out.append("| DLL under test | `%s...` built %s |"
               % (b.get("sha256_prefix"), b.get("mtime")))
    out.append("")

    for group, fields in KEYS.items():
        rows = [r for r in py_rows if r["group"] == group]
        if not rows:
            continue
        extra = PY_ONLY.get(group, [])
        header = fields + extra + ["out_n", "native", "python", "bridge", "native %"]
        out.append("## `%s`" % group)
        out.append("")
        out.append("| " + " | ".join(header) + " |")
        out.append("|" + "|".join(["---"] * len(header)) + "|")

        for r in rows:
            nat = native.get((group,) + key_of(r["axis"], fields))
            cells = [norm(r["axis"].get(f)) for f in fields]
            cells += [norm(r["axis"].get(f)) for f in extra]
            cells.append(str(r["out_n"]))
            if nat is None:
                cells += ["--", "%.2f us" % r["per_call_us"], "--", "--"]
            else:
                nu, pu = nat["per_call_us"], r["per_call_us"]
                cells += ["%.3f us" % nu, "%.2f us" % pu,
                          "%.2f us" % (pu - nu), "%.1f%%" % (100 * nu / pu)]
            out.append("| " + " | ".join(cells) + " |")
        out.append("")
        if extra:
            out.append("`%s` is a Python-side axis only -- the native call does not "
                       "receive it, so one native row is compared against every "
                       "Python row that varies it." % ", ".join(extra))
            out.append("")

    print("\n".join(out))
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
