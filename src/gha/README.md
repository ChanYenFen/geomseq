# GeomSeq for Grasshopper

A Rhino 8 Grasshopper plugin that calls the same native library as the Python
bridge. Installed, it is two files dropped into `Grasshopper/Libraries`:

| Windows | macOS |
|---|---|
| `GeomSeq.gha` + `geomseq_core.dll` | `GeomSeq.gha` + `geomseq_core.dylib` |

Rhino 8 only. On Windows, Rhino must run on .NET Core, which is the default. If
it was switched to .NET Framework with `SetDotNetRuntime`, the plugin will not load.

## Components

Both live under **GeomSeq › Sequence**.

| Component | Inputs | Outputs |
|---|---|---|
| Sort Curves (`SortCrv`) | `C` curves · `S` start (optional, default: start of the first curve) · `T` output travel (default false) · `F` allow reversal (default true; false keeps every curve's direction and skips 2-opt) | `C` sorted curves, reversals applied · `i` input indices · `R` reversed · `D` total travel · `T` travel lines (only when input `T` is true) |
| Sort Points (`SortPt`) | `P` points · `S` start (optional, default: first point) | `P` sorted points · `i` input indices · `D` total path length |

`D` includes the move from `S` to the first item. Null and degenerate items are
skipped with a warning that names their input indices, and `i` always refers to
positions in the original input list.

## Build

Needs a .NET SDK 7 or later (8 builds `net7.0` fine):

```
dotnet build src/gha/GeomSeq.csproj -c Release
```

Output in `src/gha/bin/Release/net7.0/`: `GeomSeq.gha` and a copy of the
native library. That copy is the binary committed under
`src/geomseq_core/native/`, for development only. A release build must point
at CI's output instead:

```
dotnet build src/gha/GeomSeq.csproj -c Release -p:GeomSeqNativeDir=<dir containing the CI-built library>/
```

## Try it in Rhino

Either:

- **Visual Studio**: run the `Rhino 8 (Windows)` launch profile. It starts Rhino
  with `RHINO_PACKAGE_DIRS` pointing at the build output, so nothing is installed.
- **Anything else**: copy both files from the output folder into
  `%APPDATA%\Grasshopper\Libraries` (macOS: `~/Library/Application Support/McNeel/Rhinoceros/8.0/Plug-ins/Grasshopper (b45a29b1-4343-4035-989e-044e8580d9cf)/Libraries`)
  and restart Rhino.

Milestone check: drag **Sort Curves** onto the canvas, connect a set of curves,
and confirm that `C` comes back reordered, `i` is a permutation of the input
indices, and `D` is not larger than the travel in input order.

## Two contracts

- **Component GUIDs are permanent.** Saved `.gh` files find components by them.
- **New ports go at the end.** Grasshopper saves wires by port index, so inserting
  a port in the middle rewires every existing file.

## Not done yet

Release gates, none of which exist yet: a CI job that builds the native library
and the `.gha` together; a `lipo` check that the macOS binary has both x86_64 and
arm64; `THIRD_PARTY_LICENSES.md` inside the package; a Yak manifest and store
page that say Rhino 7 is not supported.
