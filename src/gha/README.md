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

Two of the library's four modules ship here; both live under
**GeomSeq › Sequence**. `redistribute_lookups` and `build_turn_waypoints` exist
in the core but have no component yet (see Not done yet).

| Component | Inputs | Outputs |
|---|---|---|
| Sort Curves (`SortCrv`) | `C` curves · `S` start (optional, default: start of the first curve) · `T` output travel (default false) · `F` allow reversal (default true; false keeps every curve's direction and skips 2-opt) | `C` sorted curves, reversals applied · `i` input indices · `R` reversed · `D` total travel · `T` travel lines (only when input `T` is true) |
| Sort Points (`SortPt`) | `P` points · `S` start (optional, default: first point) | `P` sorted points · `i` input indices · `D` total path length |

`D` includes the move from `S` to the first item, and `i` always refers to
positions in the original input list.

## Runtime messages

Grasshopper solves on the UI thread, so a busy Rhino and a crashed one look
alike. Both components report through GH's message levels rather than throwing.

| Level | Situation |
|---|---|
| 🔴 Error | Native library failed to load — wrong platform or architecture; the message names what it detected |
| 🔴 Error | The native call threw (`DllNotFound`, `EntryPointNotFound`, `BadImageFormat`) — same cause, caught later |
| 🟡 Warning | Null or degenerate items skipped; the message names their input indices |
| 🟡 Warning | `n` above the largest profiled input — still runs, but Rhino will look frozen |
| ⚪ Remark | Empty input list |

Only the load failures stop the component. The size warning is a disclosure,
not a cap.

Thresholds are 50,000 curves (from a ~43 s ad-hoc run) and 10,000 points —
**the point figure has never been measured**. It moves once `sort_points` gets
a windowed 2-opt and a committed baseline.

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

Deferred components: `redistribute_lookups`, `build_turn_waypoints`, and the
RhinoCommon-dependent `divide_curves` / `sample_curve_points` pair.
