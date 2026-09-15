using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace GeomSeq.Native;

/// <summary>
/// Rhino geometry to flat buffers and back, around <see cref="NativeMethods"/>. The C#
/// counterpart of misc.py plus the *_native wrappers in geometry_utils.py.
/// </summary>
internal static class GeomSeqCore
{
    // Both 2-opt caps are 20, and both were measured rather than assumed. Each
    // function's search now prunes to the candidates that could shorten the
    // tour, and a pruned search is only ever the worse of the two
    // implementations while it is still unconverged -- so the cap exists to
    // guarantee convergence, not to ration time. 20 is where every shape
    // measured converges: curves in sort_curves_convergence, points in
    // sort_points_convergence, with 30 and 50 passes moving nothing.
    //
    // They stay two constants rather than one because they were two different
    // numbers as recently as 2026-09-15, when points still ran the exhaustive
    // pass and raising their cap would have doubled a 248 s solve. They may
    // diverge again; the components are not obliged to agree.
    //
    // See CLAUDE.md, "Three different numbers now spell max passes" -- the third
    // is benchmarks/python/cases.py, which stays at 10 on purpose so recorded
    // baselines remain comparable with their own history.
    private const int UseTwoOpt            = 1;
    private const int CurveTwoOptMaxPasses = 20;
    private const int PointTwoOptMaxPasses = 20;
    private const int KnnK                 = 12;

    internal sealed class CurveSortResult
    {
        public CurveSortResult(int[] order, int[] reversal, double[] travelPoints)
        {
            Order = order;
            Reversal = reversal;
            TravelPoints = travelPoints;
        }

        /// <summary>Position k holds the index, in the list passed in, of the k-th curve.</summary>
        public int[] Order { get; }

        /// <summary>0/1 per position, as the native side wrote it.</summary>
        public int[] Reversal { get; }

        /// <summary>6 doubles per segment: segment k leads into the curve at position k.</summary>
        public double[] TravelPoints { get; }

        public Line TravelSegment(int k)
        {
            double[] t = TravelPoints;
            int o = k * 6;
            return new Line(t[o], t[o + 1], t[o + 2], t[o + 3], t[o + 4], t[o + 5]);
        }

        /// <summary>
        /// Sum of all travel segments, including the move from the start point. Computed here
        /// because the native side returns the segments, not their total.
        /// </summary>
        public double TravelLength()
        {
            double total = 0.0;
            for (int k = 0; k < Order.Length; k++)
                total += TravelSegment(k).Length;
            return total;
        }
    }

    /// <summary>
    /// Sorts curves by their endpoints. Curves must be non-null. With <paramref name="allowFlip"/>
    /// false, every curve keeps its direction and the native side skips 2-opt.
    /// </summary>
    public static unsafe CurveSortResult SortCurves(IReadOnlyList<Curve> curves, Point3d start, bool allowFlip)
    {
        int n = curves.Count;
        var order    = new int[n];
        var reversal = new int[n];
        var travel   = new double[6 * n];
        if (n == 0)
            return new CurveSortResult(order, reversal, travel);

        double[] endpoints = new double[6 * n];
        for (int k = 0; k < n; k++)
        {
            Point3d s = curves[k].PointAtStart;
            Point3d e = curves[k].PointAtEnd;
            int o = k * 6;
            endpoints[o]     = s.X;
            endpoints[o + 1] = s.Y;
            endpoints[o + 2] = s.Z;
            endpoints[o + 3] = e.X;
            endpoints[o + 4] = e.Y;
            endpoints[o + 5] = e.Z;
        }
        double[] startPt = { start.X, start.Y, start.Z };

        // Pinned for the duration of the call: the GC must not move a buffer the native side is writing.
        fixed (double* ep = endpoints)
        fixed (double* sp = startPt)
        fixed (int* op = order)
        fixed (int* rp = reversal)
        fixed (double* tp = travel)
        {
            NativeMethods.SortCurves(ep, n, sp, UseTwoOpt, CurveTwoOptMaxPasses, KnnK,
                                     ifFlip: allowFlip ? 1 : 0, op, rp, tp);
        }

        return new CurveSortResult(order, reversal, travel);
    }

    /// <summary>Returns, for each position, the index in <paramref name="points"/> of the point there.</summary>
    public static unsafe int[] SortPoints(IReadOnlyList<Point3d> points, Point3d start)
    {
        int n = points.Count;
        var order = new int[n];
        if (n == 0)
            return order;

        double[] flat = new double[3 * n];
        for (int k = 0; k < n; k++)
        {
            Point3d p = points[k];
            int o = k * 3;
            flat[o]     = p.X;
            flat[o + 1] = p.Y;
            flat[o + 2] = p.Z;
        }
        double[] startPt = { start.X, start.Y, start.Z };

        fixed (double* pp = flat)
        fixed (double* sp = startPt)
        fixed (int* op = order)
        {
            NativeMethods.SortPoints(pp, n, sp, UseTwoOpt, PointTwoOptMaxPasses, KnnK, op);
        }

        return order;
    }

    /// <summary>
    /// Redistributes arc-length lookups to a new density profile. Returns the new lookups.
    /// </summary>
    /// <remarks>
    /// Two things here are the caller's job rather than the native side's, and both bite.
    ///
    /// The output length is not derivable from the input, so the buffer is sized from a
    /// bound and the native side reports how much it filled. The bound is
    /// geometry_utils.redistribute_lookups_native's, copied deliberately: it divides by
    /// <c>min(low, high)</c>, because the native marching step bottoms out at the smaller
    /// of the two and using <paramref name="low"/> alone would undercount — and undercounting
    /// here is a buffer overrun, not a short answer.
    ///
    /// <paramref name="low"/> must be positive. At zero or below the native loop never
    /// advances and Rhino hangs with no message, so it throws here rather than trusting
    /// every caller to have checked. <paramref name="high"/> below <paramref name="low"/>
    /// is merely meaningless, so it is clamped — the component reports that separately,
    /// since silently repairing a caller's input without saying so is its own defect.
    /// </remarks>
    public static unsafe double[] RedistributeLookups(
        IReadOnlyList<double> lookups, double low, double high, int mode, double flatPct,
        IReadOnlyList<int>? cornerIndices)
    {
        if (lookups.Count == 0)
            return new double[0];

        if (low <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(low), low, "low must be greater than 0.");

        if (high < low)
            high = low;

        double totalLength = lookups[lookups.Count - 1];

        // Resolved here, not passed as indices: the native side never sees the lookup
        // array. It used to, and ignored all but these entries -- at 100k samples that
        // was ~97% of the call spent marshaling an array it did not read.
        int numCorners = cornerIndices?.Count ?? 0;
        var cornerLengths = new double[numCorners];
        for (int k = 0; k < numCorners; k++)
        {
            int idx = cornerIndices![k];
            if (idx < 0 || idx >= lookups.Count)
                throw new ArgumentOutOfRangeException(nameof(cornerIndices), idx,
                    "Corner index is outside the lookup list; the native side would read out of bounds.");
            cornerLengths[k] = lookups[idx];
        }

        double minStep = low < high ? low : high;
        var outLookups = new double[(int)(totalLength / minStep) + numCorners + 10];
        int outCount = 0;

        // A zero-length array pins to null, which is what the ABI expects for no corners.
        fixed (double* cl = cornerLengths)
        fixed (double* ol = outLookups)
        {
            NativeMethods.RedistributeLookups(totalLength, low, high, mode, flatPct,
                                              cl, numCorners, ol, &outCount);
        }

        var result = new double[outCount];
        for (int k = 0; k < outCount; k++)
            result[k] = outLookups[k];
        return result;
    }

    /// <summary>Length of start → ordered[0] → ordered[1] → … .</summary>
    public static double PathLength(Point3d start, IReadOnlyList<Point3d> ordered)
    {
        double total = 0.0;
        Point3d prev = start;
        foreach (Point3d p in ordered)
        {
            total += prev.DistanceTo(p);
            prev = p;
        }
        return total;
    }
}
