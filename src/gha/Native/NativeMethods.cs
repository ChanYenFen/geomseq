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
        int     twoOptMode,       // 0 = auto; 1 and 2 exist for benchmarks only
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
}
