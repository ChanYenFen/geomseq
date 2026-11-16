# GeomSeq for Grasshopper

A Rhino 8 Grasshopper plug-in. It P/Invokes the same native library the Python
bridge uses, but does not go *through* that bridge — no Python is involved at
runtime. It carries its own version number and compatibility promises, and is
released separately from the library itself.

Installed, it is two files dropped into `Grasshopper/Libraries`:

| Windows | macOS |
|---|---|
| `GeomSeq.gha` + `geomseq_core.dll` | `GeomSeq.gha` + `geomseq_core.dylib` |

Rhino 8 only. On Windows, Rhino must run on .NET Core, which is the default. If
it was switched to .NET Framework with `SetDotNetRuntime`, the plugin will not load.

## Components

Four tabs under **GeomSeq**. `Division`, `Sequence` and `Travel` are the stages
a job runs through — prepare the geometry, order it, then join the pieces up.
`Analysis` sits beside them rather than in them: it answers questions about
geometry instead of changing it.

| Component | Tab | Inputs | Outputs |
|---|---|---|---|
| Divide Curves (`DivCrv`) | Division | `C` curves · `S` segment length (one value, or one per curve) · `J` join ends (default true) · `O` overlap (default false) · `OL` overlap length | `P` division points, one branch per curve · `S` continuous arc length · `C` indices of the segment joints |
| Redistribute Arc Lengths (`Redist`) | Division | `S` arc lengths, one curve per branch · `lo` smallest step (> 0) · `hi` largest step · `M` mode (0 dense centre, 1 dense ends) · `F` flat percent · `C` corners to keep exactly | `S` respaced arc lengths |
| Sample Curve Points (`SampleCrv`) | Division | `C` one curve per branch of `S` · `S` arc-length positions | `P` points on each curve |
| Shatter Crossings (`ShatterX`) | Division | `C` polylines, one branch at a time · `D` gap · `T` touch tolerance (default 0, exact only) · `X` also cut self-crossings (default false) · `P` plane (default: world XY) | `S` surviving pieces · `N` pieces per input curve |
| Point In Curve (`PtInCrv`) | Analysis | `C` one closed curve · `P` points · `Pl` plane (default: world XY) · `T` boundary tolerance (default 0, exact only) · `Ct` chord tolerance (default 0, takes the document's) | `R` 0 outside / 1 on the boundary / 2 inside · `P'` each point dropped onto the plane, as Rhino's own `P'` |
| Point In Curves (`PtInCrvs`) | Analysis | `C` closed curves · `P` points · `Pl` plane · `T` boundary tolerance · `Ct` chord tolerance, all as above | `R` as above · `I` index in `C` of the first curve containing the point, or -1 · `P'` as above |
| Sort Curves (`SortCrv`) | Sequence | `C` curves · `S` start (optional, default: origin) · `T` output travel (default false) · `F` allow reversal (default true; false keeps every curve's direction and skips 2-opt) | `C` sorted curves, reversals applied · `i` input indices · `R` reversed · `D` travel per branch · `T` travel lines (only when input `T` is true) |
| Sort Points (`SortPt`) | Sequence | `P` points · `S` start (optional, default: origin) · `Ct` continuous (default true; each branch starts where the last ended) | `P` sorted points · `i` input indices · `D` travel per branch |
| Build Waypoints (`Wp`) | Travel | `C` curves in travel order | `W` two points per move — the end of one curve, the start of the next — one branch per move, branch k leaving curve k |
| Build Turn Waypoints (`Turn`) | Travel | `P` points, one branch per turn · `A` max angle in degrees (default 30) · `St` fillet step (> 0) · `E` extend past each end · `Pl` plane (default: world XY) | `W` waypoints through the turn |

On the two sorts, `D` is per branch and includes the move *into* that branch —
from `S` for the first, from the previous branch's last item for the rest — so
summing the branches gives the whole journey. `i` always refers to positions in
the original input, within the item's own branch.

`Redistribute Arc Lengths` and `Sample Curve Points` are the two halves of one
job: the first decides where the positions go, the second turns them back into
points. `Divide Curves` feeds both.

## Runtime messages

Grasshopper solves on the UI thread, so a busy Rhino and a crashed one look
alike. Every component reports through GH's message levels rather than throwing.

| Level | Situation |
|---|---|
| 🔴 Error | Native library failed to load — wrong platform or architecture; the message names what it detected |
| 🔴 Error | The native call threw (`DllNotFound`, `EntryPointNotFound`, `BadImageFormat`) — same cause, caught later |
| 🟡 Warning | Null or degenerate items skipped; the message names their input indices |
| ⚪ Remark | `n` above the largest measured input — still runs, timing up there is simply untested |
| ⚪ Remark | Empty input list |

Only the load failures stop the component. The size note is a disclosure, not a
cap, which is why it is a Remark: nothing is wrong, the input has just left the
range anyone has numbers for.

Only the two sorts carry that Remark, and each threshold is that component's
largest measured input, nothing more: **50,000 for Sort Curves** (2.62 s) and
**64,000 for Sort Points** (1.50 s). Both prune their 2-opt to the candidates
that could shorten the tour, so both are quick at every size anyone has
measured, and above these numbers the honest statement is "untested", not
"slow".

The others have no such number because nothing has been committed to
`benchmarks/results/` for them, and a threshold that was never measured would
be worth less than none.

The thresholds used to mean something else. They marked where an exhaustive
O(n²) pass stopped feeling instant — 16,000 for points, where that pass took
11.94 s and 64,000 took 248.68 s. Pruning moved 64,000 points to 1.50 s, which
left those numbers marking nothing, and left an orange bubble firing on work
that finishes before the mouse button comes up.

## Development

For working on the plug-in, not for installing it. Needs .NET SDK 7 or later
(8 builds `net7.0` fine):

```
dotnet build src/gha/GeomSeq.csproj -c Release
```

Output in `bin/Release/net7.0/` (plain `dotnet build` → `bin/Debug/net7.0/`):
the `.gha` plus the dev binary from `src/geomseq_core/native/`. Release builds
must point elsewhere: `-p:GeomSeqNativeDir=<dir with the CI-built library>/`.

On Windows the `Rhino 8 (Windows)` launch profile runs Rhino against the build
output directly, installing nothing. No macOS profile yet, so copy both files
into `Libraries`:

```
# Windows
%APPDATA%\Grasshopper\Libraries
# macOS
~/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper (b45a29b1-4343-4035-989e-044e8580d9cf)/Libraries
```

**Then fully quit and relaunch Rhino.** Grasshopper scans `Libraries` only at
startup; closing the Grasshopper window is not enough. Same for every rebuild.

Milestone check: drag **Sort Curves** on, connect curves, confirm `C` is
reordered, `i` is a permutation, and `D` is no larger than the input order's.

## Two contracts

- **Component GUIDs are permanent.** Saved `.gh` files find components by them.
- **New ports go at the end.** Grasshopper saves wires by port index, so inserting
  a port in the middle rewires every existing file.

## Not done yet

Release gates, none of which exist yet: a CI job that builds the native library
and the `.gha` together; a `lipo` check that the macOS binary has both x86_64 and
arm64; `THIRD_PARTY_LICENSES.md` inside the package; a Yak manifest and store
page that say Rhino 7 is not supported; measured n → time figures to publish.

Every core module now has a component. Anything else outstanding lives in
*Future directions → Still open* in [../../docs/design-notes.md](../../docs/design-notes.md)
and is deliberately not repeated here — this file said "two of the library's
four modules ship here" while seven did, which is what a second copy of a todo
list does.

One thing that is only true of the package, so it belongs here: every component
but **Point In Curve**, **Point In Curves** and **Build Waypoints** has
artwork. Those three draw Grasshopper's default icon until
`point_in_curve.png`, `point_in_curves.png` and `build_waypoints.png` land in
`Resources/`, which is all it takes — the csproj embeds that folder by
wildcard.
