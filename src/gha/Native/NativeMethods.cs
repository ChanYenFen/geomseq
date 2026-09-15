using System.Runtime.InteropServices;

namespace GeomSeq.Native;

/// <summary>
/// Raw geomseq_core entry points. Each signature mirrors the <c>extern "C"</c> definition in
/// src/geomseq_core/native/*.cpp and its ctypes argtypes in native_bridge.py.
/// </summary>
/// <remarks>
/// The ABI is double, int, and pointers to them, nothing else: every argument is blittable,
/// so the runtime passes it through untouched. The caller allocates and pins every buffer;
/// the native side only writes into them, and never allocates or frees.
/// Call <see cref="NativeLibraryLoader.TryEnsureLoaded"/> first.
/// </remarks>
internal static unsafe class NativeMethods
{
    [DllImport(NativeLibraryLoader.LibraryName, EntryPoint = "sort_curves",
               CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    internal static extern void SortCurves(
        double* endpoints,        // 6n: sx,sy,sz, ex,ey,ez per curve
        int     n,
        double* startPt,          // 3
        int     useTwoOpt,        // 0/1
        int     twoOptMaxPasses,
        int     knnK,
        int     ifFlip,           // 0/1
        int*    outOrder,         // n: original curve index per position
        int*    outReversal,      // n: 0/1
        double* outTravelPoints); // 6n: segment k runs from curve k-1's exit (or startPt) to curve k's entry

    [DllImport(NativeLibraryLoader.LibraryName, EntryPoint = "sort_points",
               CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    internal static extern void SortPoints(
        double* points,           // 3n: x,y,z per point
        int     n,
        double* startPt,          // 3
        int     useTwoOpt,        // 0/1
        int     twoOptMaxPasses,
        int     knnK,
        int*    outOrder);        // n: original point index per position

    // Unlike the sort entry points, the output length is not known from the input:
    // outLookups is sized by the caller from a bound (see GeomSeqCore) and outCount
    // says how much of it was actually written.
    [DllImport(NativeLibraryLoader.LibraryName, EntryPoint = "redistribute_lookups",
               CallingConvention = CallingConvention.Cdecl, ExactSpelling = true)]
    internal static extern void RedistributeLookups(
        double  totalLength,      // arc length of the whole curve
        double  low,              // smallest step, at the density peak; must be > 0 or the march never advances
        double  high,             // largest step, at the sparsest point
        int     mode,             // 0 = dense_center, 1 = dense_sides
        double  flatPct,          // percent of totalLength held at constant density, centred
        double* cornerLengths,    // numCorners, ascending; may be null when numCorners == 0
        int     numCorners,
        double* outLookups,       // caller-sized; see GeomSeqCore.RedistributeLookups
        int*    outCount);        // entries actually written
}
