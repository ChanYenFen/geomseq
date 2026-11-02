# CLAUDE.md

Rules for working in this repo, one or two lines each. The reasoning behind
every rule — what was measured, what went wrong before — is in
[docs/design-notes.md](docs/design-notes.md); read the named section before
changing what a rule protects.

- Results and what they mean: [docs/benchmarks.md](docs/benchmarks.md)
- How to run anything: [benchmarks/README.md](benchmarks/README.md)
- Build and test: [README.md](README.md)

## Benchmarks

- **Benchmarks are not tests.** Nothing under `benchmarks/` runs in pytest or
  can fail a build, and no large dataset goes into `tests/fixtures/`.
  *(Why benchmarks are not tests)*
- **Quote only committed runs.** Every number in `docs/benchmarks.md` must come
  from a file in `benchmarks/results/`. Ad-hoc numbers may appear in design
  notes only, labelled as ad hoc. *(Why results are only quoted from committed runs)*
- **Commit the DLL with the results it produced.** The sha256 in a result file
  identifies one binary, not one source revision — MSVC stamps a timestamp, so
  rebuilding identical sources changes the hash.
- **`uniform` is the control.** Never replace it, only add datasets beside it;
  it is a reserved label. *(Why `uniform` stays the control)*
- **Fixtures are sampled, not sliced, and keep file order.** See `sample()` in
  `benchmarks/python/cases.py`. *(Why fixtures are sampled, not sliced)*
- **`max_passes` in `cases.py` stays 10**, while every shipped caller uses 20.
  10 is what all recorded baselines used; changing it breaks comparability.
  *(Still open → Two numbers spell "max passes")*
- **Changing an existing case breaks comparability** with every baseline that
  recorded it. Prefer adding a case; retire one only with its results kept.

## Algorithms

- **Pruning must be provable.** A search may skip a candidate only if it can
  prove the candidate cannot help (2-opt's gap radius, shatter's padded boxes).
  Never a fixed constant that trades quality for speed on the caller's behalf —
  that is why the windowed 2-opt was removed. *(Why the windowed 2-opt was
  removed; Why the 2-opt search is pruned)*
- **Verify a pruned search against the exhaustive one.** 2-opt: a converged tour
  has zero improving moves left. Shatter: output identical to the pair loop, and
  the guard test must fail when the pruning is deliberately broken.
- **Shatter's broad phase is sweep and prune, not Bentley–Ottmann**, because the
  function accepts touches within `touch_tol` and multi-line junctions.
  *(Why shatter's broad phase is sweep and prune)*
- **Clustered sequencing is a separate algorithm.** Do not fold it into the sort
  components, and do not ship it before the candidate-count question is
  measured. *(Clustered sequencing for layered and zoned fabrication)*

## Grasshopper plug-in (`src/gha/`)

- **GUIDs are permanent; new ports are appended, never inserted.** Saved `.gh`
  files find components by GUID and wires by port index. Same for the library
  GUID in `GeomSeqInfo`. *(Contracts)*
- **The Python layer stays.** pytest is the only verification outside Rhino;
  only `src/gh/*_component.py` retires, once its C# twin is checked in Rhino.
- **Read points as `GH_Point`, and keep list inputs `Optional`.** A null
  `Point3d` arrives as the origin; a required input stops `SolveInstance` from
  running. *(Why some component behaviour is not obvious)*
- **The "tested limit" warnings (16,000) come from a committed baseline.** Do not
  replace them with a number that has not been.

## Working on this machine

- **Close Rhino before rebuilding the DLL.** Rhino holds `geomseq_core.dll`
  open, and a stale DLL behind a newer wrapper corrupts memory inside Rhino.
  While it is open, copy `src/` and `tests/` elsewhere and test there.
- **The macOS dylib can only be rebuilt on a Mac**, so it can lag behind the
  DLL. Check its last commit before assuming it matches.

## Data

- **The repository is public.** Coordinates exported from a client job are that
  client's design. Use a pattern you own or one already public.
  *(Real geometry worth recording)*

## Open questions

The live list is under *Future directions → Still open* in
[docs/design-notes.md](docs/design-notes.md). Update it there, not here.
