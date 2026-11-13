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
    // 2-opt pass caps: 20 is where every measured shape converges.
    // See docs/design-notes.md, "Two numbers spell max passes".
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
    /// Redistributes arc lengths to a new density profile. Returns the new arcLengths.
    /// </summary>
    /// <remarks>
    /// <paramref name="low"/> must be positive, or the native loop never advances; it throws.
    /// <paramref name="high"/> below <paramref name="low"/> is clamped to it.
    /// The output buffer is sized from the same bound as
    /// geometry_utils.redistribute_arc_lengths_native: one step of <c>min(low, high)</c> at a
    /// time. Undercounting it overruns the buffer.
    /// </remarks>
    public static unsafe double[] RedistributeArcLengths(
        IReadOnlyList<double> arcLengths, double low, double high, int mode, double flatPct,
        IReadOnlyList<int>? cornerIndices)
    {
        if (arcLengths.Count == 0)
            return new double[0];

        if (low <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(low), low, "low must be greater than 0.");

        if (high < low)
            high = low;

        double totalLength = arcLengths[arcLengths.Count - 1];

        // Corner indices are resolved to arc lengths here; the native side never sees the array.
        int numCorners = cornerIndices?.Count ?? 0;
        var cornerLengths = new double[numCorners];
        for (int k = 0; k < numCorners; k++)
        {
            int idx = cornerIndices![k];
            if (idx < 0 || idx >= arcLengths.Count)
                throw new ArgumentOutOfRangeException(nameof(cornerIndices), idx,
                    "Corner index is outside the arc length list; the native side would read out of bounds.");
            cornerLengths[k] = arcLengths[idx];
        }

        double minStep = low < high ? low : high;
        var outArcLengths = new double[(int)(totalLength / minStep) + numCorners + 10];
        int outCount = 0;

        // A zero-length array pins to null, which is what the ABI expects for no corners.
        fixed (double* cl = cornerLengths)
        fixed (double* ol = outArcLengths)
        {
            NativeMethods.RedistributeArcLengths(totalLength, low, high, mode, flatPct,
                                              cl, numCorners, ol, &outCount);
        }

        var result = new double[outCount];
        for (int k = 0; k < outCount; k++)
            result[k] = outArcLengths[k];
        return result;
    }

    /// <summary>Waypoints for one turn: leaving E, then arriving at S. Flat x,y pairs.</summary>
    internal sealed class TurnWaypoints
    {
        public TurnWaypoints(double[] exit, double[] entry)
        {
            Exit = exit;
            Entry = entry;
        }

        /// <summary>2 doubles per waypoint.</summary>
        public double[] Exit { get; }

        /// <summary>2 doubles per waypoint.</summary>
        public double[] Entry { get; }
    }

    /// <summary>
    /// Builds the smooth turn from the end of one path to the start of the next.
    /// </summary>
    /// <remarks>
    /// 2D only. stepLen must be positive; it throws otherwise. Both buffers are sized
    /// from the bound in the .cpp header, with the native side's own fallback to one
    /// degree when thetaMaxDeg is at or below 1e-6.
    /// </remarks>
    public static unsafe TurnWaypoints BuildTurnWaypoints(
        double ex, double ey, double avx, double avy,
        double sx, double sy, double bvx, double bvy,
        double thetaMaxDeg, double stepLen, double extendLen)
    {
        if (stepLen <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(stepLen), stepLen, "stepLen must be greater than 0.");

        double effectiveTheta = thetaMaxDeg > 1e-6 ? thetaMaxDeg : 1.0;
        int maxPoints = (int)Math.Ceiling(180.0 / effectiveTheta) + 2;

        var exitPts = new double[maxPoints * 2];
        var entryPts = new double[maxPoints * 2];
        int exitCount = 0, entryCount = 0;

        fixed (double* xp = exitPts)
        fixed (double* np = entryPts)
        {
            NativeMethods.BuildTurnWaypoints(ex, ey, avx, avy, sx, sy, bvx, bvy,
                                             thetaMaxDeg, stepLen, extendLen,
                                             xp, &exitCount, np, &entryCount);
        }

        return new TurnWaypoints(Trim(exitPts, exitCount), Trim(entryPts, entryCount));
    }

    private static double[] Trim(double[] buffer, int pointCount)
    {
        var trimmed = new double[pointCount * 2];
        for (int k = 0; k < trimmed.Length; k++)
            trimmed[k] = buffer[k];
        return trimmed;
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

    internal sealed class ShatterResult
    {
        public ShatterResult(double[] pieces, int[] pieceCounts)
        {
            Pieces = pieces;
            PieceCounts = pieceCounts;
        }

        /// <summary>6 doubles per surviving piece, grouped in input-segment order.</summary>
        public double[] Pieces { get; }

        /// <summary>One entry per input segment. 0 means the gaps swallowed it whole.</summary>
        public int[] PieceCounts { get; }

        public Line Piece(int k)
        {
            double[] p = Pieces;
            int o = k * 6;
            return new Line(p[o], p[o + 1], p[o + 2], p[o + 3], p[o + 4], p[o + 5]);
        }
    }

    /// <summary>
    /// Cuts segments where they meet and opens a gap of <paramref name="gapD"/> there.
    /// Segments are already flattened to 6 doubles each, in the plane the caller wants
    /// the contacts measured in -- the native side reads x/y only and carries z along.
    /// </summary>
    public static unsafe ShatterResult ShatterAtCrossings(
        double[] segmentBuffer, int n, double gapD, double touchTol,
        int[]? segmentOwner, bool testSelf)
    {
        if (n <= 0)
            return new ShatterResult(new double[0], new int[0]);

        if (gapD < 0.0)
            throw new ArgumentOutOfRangeException(nameof(gapD), gapD, "gapD must be 0 or more.");

        if (touchTol < 0.0)
            throw new ArgumentOutOfRangeException(nameof(touchTol), touchTol, "touchTol must be 0 or more.");

        if (segmentOwner != null && segmentOwner.Length != n)
            throw new ArgumentException("segmentOwner needs one entry per segment.", nameof(segmentOwner));

        var pieceCounts = new int[n];

        // First guess; if it falls short the native side reports the size needed and
        // the call is repeated once.
        int capacity = 3 * n;
        double[] pieces = new double[capacity * 6];
        int total = 0;

        fixed (double* seg = segmentBuffer)
        fixed (int* counts = pieceCounts)
        {
            fixed (double* buf = pieces)
            fixed (int* owner = segmentOwner)
            {
                NativeMethods.ShatterAtCrossings(seg, n, gapD, touchTol, owner, testSelf ? 1 : 0,
                                                 buf, capacity, counts, &total);
            }

            if (total > capacity)
            {
                capacity = total;
                pieces = new double[capacity * 6];
                fixed (double* buf = pieces)
                fixed (int* owner = segmentOwner)
                {
                    NativeMethods.ShatterAtCrossings(seg, n, gapD, touchTol, owner, testSelf ? 1 : 0,
                                                     buf, capacity, counts, &total);
                }
            }
        }

        if (pieces.Length != total * 6)
        {
            var trimmed = new double[total * 6];
            for (int k = 0; k < trimmed.Length; k++)
                trimmed[k] = pieces[k];
            pieces = trimmed;
        }

        return new ShatterResult(pieces, pieceCounts);
    }
}
