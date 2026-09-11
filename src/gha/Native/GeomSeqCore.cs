using System.Collections.Generic;
using Rhino.Geometry;

namespace GeomSeq.Native;

/// <summary>
/// Rhino geometry to flat buffers and back, around <see cref="NativeMethods"/>. The C#
/// counterpart of misc.py plus the *_native wrappers in geometry_utils.py.
/// </summary>
internal static class GeomSeqCore
{
    // The settings the Python components shipped with.
    private const int UseTwoOpt       = 1;
    private const int TwoOptMaxPasses = 10;
    private const int KnnK            = 12;

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
            NativeMethods.SortCurves(ep, n, sp, UseTwoOpt, TwoOptMaxPasses, KnnK,
                                     ifFlip: allowFlip ? 1 : 0, twoOptMode: 0, op, rp, tp);
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
            NativeMethods.SortPoints(pp, n, sp, UseTwoOpt, TwoOptMaxPasses, KnnK, op);
        }

        return order;
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
