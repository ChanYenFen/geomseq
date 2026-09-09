# benchmarks

Timing harnesses for the core functions. Separate from `tests/` on purpose:
timings make bad assertions (they are machine- and load-dependent), so nothing
here runs under `pytest` and nothing here fails a build. Results and what they
mean live in [../docs/benchmarks.md](../docs/benchmarks.md); design rationale
lives in [../CLAUDE.md](../CLAUDE.md). This file is how to run things.

```
python/cases.py   representative inputs per function -- what counts as a size
python/run.py     timing harness + runner -- how a size gets measured
native/           bench_core.cpp -- pure C++ timing, no Python involved
compare.py        aligns the two JSON baselines into one table
fixtures/         input datasets, one per file; picked up automatically
synth/            one-shot generator for the synthetic fixtures; nothing imports it
gh/               Grasshopper-side export of real geometry into fixtures/
results/          recorded baselines + the comparison
```

Both harnesses emit JSON; `compare.py` is the only thing that renders a
native-vs-Python table, so the format lives in one place.

## Two harnesses, two questions

They are complements, not alternatives, and neither one's numbers substitute
for the other's:

| | measures | answers |
|---|---|---|
| `python/` | wrapper + ctypes + marshaling + native | what a caller actually waits for |
| `native/` | the algorithm alone | what the C++ costs |

The difference between them is the bridge overhead, and that subtraction is the
whole reason the native harness exists: for the microsecond-scale functions the
Python harness largely measures itself.

Both harnesses use identical inputs and report the same `out_n`, so rows can be
read side by side. For these small deterministic cases the inputs are simply
written out in both files rather than shared through fixtures; fixture files
only become necessary if a larger dataset ever needs exact parity.

`native/` deliberately adds no timing entry points to the shipped DLL. It
compiles the same `.cpp` sources a second time into its own executable, so the
production API is untouched.

## Running the Python harness

```
python benchmarks/python/run.py                 # default set, all groups
python benchmarks/python/run.py sort_points     # one function
python benchmarks/python/run.py --heavy         # add the multi-minute 2-opt cases
```

Needs a compiled `geomseq_core` binary for the current platform, same as the
tests -- see the repo README's Build section. Results land in
`results/baseline-<sys>-<machine>-<date>`; a partial run tags the filename with
the groups it covered so it cannot overwrite a full baseline.

Default runtime is a few minutes -- `sort_points` alone is ~80 s, because every
fixture in `fixtures/` adds a parallel sweep (7 point datasets x 7 sizes x 2-opt
on/off). Deleting fixtures shrinks the run. `--heavy` adds `sort_points` 2-opt
at n = 16k/32k/64k and `sort_curves` at 16k, which together run into several
minutes.

## Running the native harness

Covers `build_turn_waypoints` and `redistribute_lookups` only -- the two whose
native cost the Python harness cannot resolve. The sort functions are already
96-97% native at scale, so their Python numbers are within a few percent of the
truth and are not duplicated here.

From `benchmarks/native/`:

```
# Windows (x64 Native Tools Command Prompt)
cl /std:c++17 /O2 /EHsc /MT bench_core.cpp ..\..\src\geomseq_core\native\redistribute_lookups.cpp ..\..\src\geomseq_core\native\build_turn_waypoints.cpp /Fe:bench_core.exe

# macOS / Linux
c++ -std=c++17 -O2 -o bench_core bench_core.cpp ../../src/geomseq_core/native/redistribute_lookups.cpp ../../src/geomseq_core/native/build_turn_waypoints.cpp
```

It prints JSON on stdout; redirect it to keep a record:

```
./bench_core > ../results/native.json
```

The binary is gitignored -- build it on demand. Results carry the compiler
version, which is what will make a same-machine compiler comparison possible
later; that is not attempted yet.

## Comparing the two

```
python benchmarks/python/run.py build_turn_waypoints redistribute_lookups --out benchmarks/results/python
benchmarks/native/bench_core > benchmarks/results/native.json
python benchmarks/compare.py benchmarks/results/native.json benchmarks/results/python.json
```

Prints a per-case table of `native` / `python` / `bridge` (the difference), so
each function's cost can be read stage by stage. Redirect it into
`results/comparison.md` to keep it.

Cases are matched on the fields both harnesses share -- `geometry` +
`theta_max_deg` for turns, `band` + `corners` + `mode` for redistribute.
`input_n` exists only on the Python side (since the ABI change the native call
never receives the input array), so one native row is compared against each
Python row that varies it, and the table says so. Any case the native harness
does not cover shows `--` rather than being dropped.

## Method

Warm up once (the first call absorbs DLL load and reads ~10x high), then report
the **minimum** of up to 5 runs -- scheduling noise only ever adds time, so the
minimum is the least noisy estimate of the work itself. A case that has already
burned 2 s stops repeating, so slow cases are not run five times.

Input generation is outside the timed region. Generators are seeded, so a rerun
on the same machine is comparable.

Two things the numbers include that are worth naming, because they are not the
C++ core: **ctypes marshaling** and, for `sort_curves`, the wrapper's
Python-side `apply_order` (which calls `Duplicate`/`Reverse` per curve). The
harness times the wrapper, not the raw DLL symbol, so a slow case is not
automatically a slow algorithm.

Every result file carries an environment block including a SHA-256 prefix of the
binary the numbers came from. That is deliberate: `docs/benchmarks.md`'s rule is
to quote ratios and shares rather than microseconds unless the environment is
stated, and CI never validates that the committed binary is current.

## Adding a fixture

Drop a JSON file in `fixtures/` and its cases appear -- nothing to configure.
One dataset per file, named `points_<name>.json` or `curves_<name>.json`; the
kind is in the filename so the harness can list fixtures without parsing them.

```json
{
  "kind": "points",
  "name": "real_lettering",
  "source": "exported from <file/job>, <date>",
  "units": "mm",
  "notes": "single-colour satin lettering, one hoop",
  "data": [[0.0, 0.0], [1.0, 2.0]]
}
```

`data` rows are `[x, y]` for points and `[x0, y0, x1, y1]` for curves -- the
same shape `tests/fixtures/sort_curves_cases.json` uses. Only `kind` and `data`
are read; the rest is provenance, and worth filling in because a number is not
interpretable without knowing what produced it. Write the metadata keys one per
line and keep `data` on a single line: indenting the array costs 2.4x the file
size for nothing anyone reads (2.90 MB against 1.19 MB at 64,000 points).

Sizes are drawn by seeded **sampling**, not slicing, and the file's own order is
preserved -- see `sample()` in `python/cases.py` for why. A sweep size larger
than the fixture is skipped rather than silently truncated, so make the file
large enough for the top of the sweep (64,000 points / 16,000 curves).

The label `uniform` is reserved: a fixture named `uniform` replaces the in-code
generator rather than sitting beside it. Fixtures are otherwise additive --
each adds a parallel set of rows tagged with its name in the `data` column, so
uniform and everything else sit side by side in the same table.

The synthetic set already in `fixtures/` is generated by
`synth/make_fixtures.py`, which nothing imports; each file repeats its own
settings in a `params` block, so the datasets stay interpretable if the script
goes away.

### Don't put benchmark data in `tests/fixtures/`

The two have opposite requirements:

| | `tests/fixtures/` | `benchmarks/fixtures/` |
|---|---|---|
| wants | small, awkward, edge cases | large, regular, scalable |
| size | a few hundred entries | tens of thousands |
| packing | many cases per file, all loaded | one dataset per file, loaded on demand |

`tests/test_sort_points.py` turns *every* entry in its fixture into a
parametrized test with `use_two_opt=True`. A 64,000-point dataset dropped in
there becomes a single test running exhaustive 2-opt at 64k -- roughly 3.5
minutes, against the suite's current 0.06 s.

### Before committing real data

The repository is public. Coordinates exported from a client job are that
client's design, recoverable from the fixture. Prefer a pattern you own, or one
that is already public, over anything from live work.
