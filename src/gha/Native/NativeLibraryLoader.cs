using System;
using System.Collections.Generic;
using System.IO;
using System.Reflection;
using System.Runtime.CompilerServices;
using System.Runtime.InteropServices;

namespace GeomSeq.Native;

/// <summary>
/// Loads geomseq_core from the folder GeomSeq.gha was installed to, and routes every
/// <c>[DllImport("geomseq_core")]</c> in this assembly to that handle.
/// </summary>
/// <remarks>
/// Without this, DllImport probes Rhino's own directory and PATH, never the plugin's folder,
/// so a correctly installed library is still not found.
/// </remarks>
internal static class NativeLibraryLoader
{
    internal const string LibraryName = "geomseq_core";

    // Every entry point the components call. Checked at load time so a stale binary is
    // reported once, clearly, instead of as EntryPointNotFoundException mid-solve.
    private static readonly string[] RequiredExports = { "sort_curves", "sort_points" };

    private static readonly object Gate = new();
    private static IntPtr _handle;

    static NativeLibraryLoader()
    {
        // Allowed once per assembly; the static constructor guarantees that.
        NativeLibrary.SetDllImportResolver(typeof(NativeLibraryLoader).Assembly, Resolve);
    }

    /// <summary>
    /// Loads the library if it is not loaded yet. Call before any native entry point.
    /// A failure is not cached, so copying the missing file in and recomputing is enough.
    /// </summary>
    public static bool TryEnsureLoaded(out string error)
    {
        lock (Gate)
        {
            error = _handle != IntPtr.Zero ? string.Empty : Load() ?? string.Empty;
            return _handle != IntPtr.Zero;
        }
    }

    private static IntPtr Resolve(string libraryName, Assembly assembly, DllImportSearchPath? searchPath)
        => libraryName == LibraryName ? _handle : IntPtr.Zero;

    /// <returns>null on success, otherwise the message to show on the component.</returns>
    private static string? Load()
    {
        string? fileName = OperatingSystem.IsWindows() ? "geomseq_core.dll"
                         : OperatingSystem.IsMacOS()   ? "geomseq_core.dylib"
                         : null;
        if (fileName == null)
            return $"GeomSeq runs on Windows and macOS only; detected {Platform}.";

        var searched = new List<string>();
        foreach (string dir in CandidateDirectories())
        {
            string path = Path.Combine(dir, fileName);
            if (searched.Contains(path))
                continue;
            searched.Add(path);
            if (!File.Exists(path))
                continue;

            IntPtr handle;
            try
            {
                handle = NativeLibrary.Load(path);
            }
            catch (Exception e) when (e is DllNotFoundException or BadImageFormatException)
            {
                string hint = OperatingSystem.IsMacOS()
                    ? "It must be a universal binary containing both x86_64 and arm64."
                    : "It must be a 64-bit (x64) Windows DLL.";
                return $"Found {path} but it would not load on {Platform}. {hint}\n{e.Message}";
            }

            foreach (string export in RequiredExports)
            {
                if (!NativeLibrary.TryGetExport(handle, export, out _))
                {
                    NativeLibrary.Free(handle);
                    return $"{path} has no '{export}' entry point, so it is older than this plugin. " +
                           "Install GeomSeq.gha and the native library from the same release.";
                }
            }

            _handle = handle;
            return null;
        }

        string where = searched.Count == 0 ? "(the plugin's install folder could not be determined)"
                                           : string.Join("\n  ", searched);
        return $"{fileName} was not found next to GeomSeq.gha ({Platform}). Searched:\n  {where}";
    }

    private static IEnumerable<string> CandidateDirectories()
    {
        // Where the runtime loaded this assembly from. Empty when Grasshopper loads the .gha
        // from a byte array (its "memory load" option), hence the second source.
        string location = typeof(NativeLibraryLoader).Assembly.Location;
        if (!string.IsNullOrEmpty(location))
            yield return Path.GetDirectoryName(location)!;

        // Where Grasshopper says it read the .gha from, which survives memory loading.
        string? ghLocation = GrasshopperLocation();
        if (!string.IsNullOrEmpty(ghLocation))
            yield return Directory.Exists(ghLocation) ? ghLocation : Path.GetDirectoryName(ghLocation)!;
    }

    private static string? GrasshopperLocation()
    {
        try
        {
            return QueryGrasshopperLocation();
        }
        catch (Exception)
        {
            return null;  // fallback only; the error message lists what was searched
        }
    }

    // Kept out of line so that, with no Grasshopper loaded (a test host), the failure is
    // thrown inside the catch above rather than when the JIT compiles the caller.
    [MethodImpl(MethodImplOptions.NoInlining)]
    private static string? QueryGrasshopperLocation()
        => Grasshopper.Instances.ComponentServer?.FindAssembly(GeomSeqInfo.LibraryId)?.Location;

    private static string Platform
    {
        get
        {
            string os = OperatingSystem.IsWindows() ? "Windows"
                      : OperatingSystem.IsMacOS()   ? "macOS"
                      : RuntimeInformation.OSDescription;
            return $"{os} {RuntimeInformation.ProcessArchitecture.ToString().ToLowerInvariant()}";
        }
    }
}
