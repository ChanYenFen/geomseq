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

    // point_in_curve's statuses, as native/point_in_curve.cpp defines them.
    // Zero is OK and needs no name; below zero nothing was written. Only
    // NotClosed is named: the checks below make the rest unreachable, and
    // StatusMessage falls back to the number for anything that does arrive.
    private const int PicWarnFlatArea =  1;
    private const int PicErrNotClosed = -3;

    /// <summary>
    /// Classifies points against one closed curve, measured in <paramref name="plane"/>.
    /// </summary>
    /// <remarks>
    /// The native side is 2D and knows nothing of curves or planes: the curve is
    /// discretized and both it and the query points are flattened into the plane
    /// here, once for all of them. <paramref name="tol"/> is how close to the boundary
    /// counts as coincident, and <paramref name="chordTol"/> bounds how far the
    /// discretized boundary may sit from the real one -- see design-notes.md,
    /// "Point in curve, and where it cannot match Rhino's".
    /// </remarks>
    /// <returns>
    /// <c>relationship</c>: 0 outside, 1 on the boundary, 2 inside, one per query
    /// point. These are the integers Grasshopper's own Point In Curve reports in
    /// <c>R</c>; they are NOT Rhino's <see cref="PointContainment"/>, which numbers
    /// the same three states differently.
    /// <c>projected</c>: each query point dropped onto the plane, which is where the
    /// test actually happened -- the <c>P'</c> of Rhino's component. It costs nothing,
    /// being computed to call the native side at all.
    /// <c>collapsed</c>: the curve enclosed almost no area once projected, so
    /// containment means nothing. The answers are still filled in; warn and carry on.
    /// </returns>
    public static unsafe (int[] relationship, Point3d[] projected, bool collapsed) PointInCurve(
        Curve curve, IReadOnlyList<Point3d> queryPoints, Plane plane,
        double tol, double chordTol)
    {
        if (curve == null)
            throw new ArgumentNullException(nameof(curve));

        CheckContainmentArgs(plane, tol, chordTol);

        var verts = new List<double>();
        int[] counts = { FlattenBoundary(curve, plane, chordTol, verts) };

        double[] query = ProjectQuery(queryPoints, plane);
        int      n     = queryPoints.Count;
        var relationship = new int[n];

        int status;
        fixed (double* vp = verts.ToArray())
        fixed (int* cp = counts)
        fixed (double* qp = query)
        fixed (int* rp = relationship)
        {
            status = NativeMethods.PointInCurve(vp, cp, 1, qp, n, tol, rp);
        }

        if (status < 0)
            throw new InvalidOperationException(StatusMessage(status));

        return (relationship, ReadBack(query, plane), status == PicWarnFlatArea);
    }

    /// <summary>
    /// Classifies points against several closed curves, measured in <paramref name="plane"/>,
    /// and says which curve contains each.
    /// </summary>
    /// <remarks>
    /// Each point is tested against the curves in list order and stops at the first
    /// that contains it, on the boundary or inside. Nested curves therefore answer
    /// for whichever containing curve comes first in the list. Discretization and
    /// projection are as for <see cref="PointInCurve"/>, once per curve.
    /// </remarks>
    /// <returns>
    /// <c>relationship</c> and <c>projected</c> as for <see cref="PointInCurve"/>.
    /// <c>index</c>: the containing curve's position in <paramref name="curves"/>, or -1
    /// for a point none contains -- the <c>I</c> of Rhino's own Point In Curves.
    /// <c>collapsed</c>: at least one curve enclosed almost no area once projected.
    /// </returns>
    public static unsafe (int[] relationship, int[] index, Point3d[] projected, bool collapsed) PointInCurves(
        IReadOnlyList<Curve> curves, IReadOnlyList<Point3d> queryPoints, Plane plane,
        double tol, double chordTol)
    {
        if (curves == null)
            throw new ArgumentNullException(nameof(curves));
        if (curves.Count == 0)
            throw new ArgumentException("At least one curve is needed.", nameof(curves));

        CheckContainmentArgs(plane, tol, chordTol);

        var verts  = new List<double>();
        var counts = new int[curves.Count];
        for (int c = 0; c < curves.Count; c++)
        {
            if (curves[c] == null)
                throw new ArgumentNullException(nameof(curves), $"Curve {c} is null.");
            counts[c] = FlattenBoundary(curves[c], plane, chordTol, verts);
        }

        double[] query = ProjectQuery(queryPoints, plane);
        int      n     = queryPoints.Count;
        var relationship = new int[n];
        var index        = new int[n];

        int status;
        fixed (double* vp = verts.ToArray())
        fixed (int* cp = counts)
        fixed (double* qp = query)
        fixed (int* rp = relationship)
        fixed (int* ip = index)
        {
            status = NativeMethods.PointInCurves(vp, cp, curves.Count, qp, n, tol, rp, ip);
        }

        if (status < 0)
            throw new InvalidOperationException(StatusMessage(status));

        return (relationship, index, ReadBack(query, plane), status == PicWarnFlatArea);
    }

    private static void CheckContainmentArgs(Plane plane, double tol, double chordTol)
    {
        if (!plane.IsValid)
            throw new ArgumentException("The plane is not valid.", nameof(plane));

        if (tol < 0.0)
            throw new ArgumentOutOfRangeException(nameof(tol), tol, "tol must be 0 or more.");

        if (chordTol <= 0.0)
            throw new ArgumentOutOfRangeException(nameof(chordTol), chordTol, "chordTol must be greater than 0.");
    }

    /// <summary>
    /// Discretizes one closed curve, flattens it into the plane and appends its
    /// vertices to <paramref name="into"/>, first repeated as last. Returns how many.
    /// </summary>
    private static int FlattenBoundary(Curve curve, Plane plane, double chordTol, List<double> into)
    {
        // Checked here rather than left to the native guard: Curve.IsClosed is the
        // authority, and it can answer for the curve rather than for its discretization.
        if (!curve.IsClosed)
            throw new ArgumentException("The curve must be closed for containment to mean anything.", nameof(curve));

        Polyline boundary = Discretize(curve, chordTol);

        int start = into.Count;
        foreach (Point3d p in boundary)
        {
            if (!plane.RemapToPlaneSpace(p, out Point3d local))
                throw new ArgumentException("The curve could not be mapped into the plane.", nameof(plane));
            into.Add(local.X);
            into.Add(local.Y);
        }

        int written = into.Count - start;

        // Guards the indexing in the fix-up as much as it rejects the input.
        if (written < 6)
            throw new ArgumentException(
                $"The curve discretized to {written / 2} vertices, too few to enclose anything.",
                nameof(curve));

        // The ABI wants the first vertex repeated as the last. A closed curve's
        // polyline normally arrives that way; appending costs one vertex where it
        // does not, and the curve was already known to be closed above.
        double x0 = into[start], y0 = into[start + 1];
        if (x0 != into[into.Count - 2] || y0 != into[into.Count - 1])
        {
            into.Add(x0);
            into.Add(y0);
        }

        int vertexCount = (into.Count - start) / 2;

        // Three vertices that already close is a line out and back, enclosing
        // nothing. Checked after the fix-up, so it catches that case rather than
        // leaving it to the native guard.
        if (vertexCount < 4)
            throw new ArgumentException(
                $"The curve discretized to {vertexCount} vertices, which enclose no area.", nameof(curve));

        return vertexCount;
    }

    private static double[] ProjectQuery(IReadOnlyList<Point3d> queryPoints, Plane plane)
    {
        int n = queryPoints.Count;
        var query = new double[2 * n];
        for (int k = 0; k < n; k++)
        {
            if (!plane.RemapToPlaneSpace(queryPoints[k], out Point3d local))
                throw new ArgumentException("A query point could not be mapped into the plane.", nameof(plane));
            query[k * 2]     = local.X;
            query[k * 2 + 1] = local.Y;
        }
        return query;
    }

    // The flattened query points, read back out of the plane: the point the
    // classification actually answered for, which is what Rhino's P' reports.
    private static Point3d[] ReadBack(double[] query, Plane plane)
    {
        var projected = new Point3d[query.Length / 2];
        for (int k = 0; k < projected.Length; k++)
            projected[k] = plane.PointAt(query[k * 2], query[k * 2 + 1]);
        return projected;
    }

    /// <summary>Flattens a curve to a polyline whose deviation from it is bounded.</summary>
    /// <remarks>
    /// DivideByCount is deliberately not used: equal spacing has no error bound, so
    /// nothing could be claimed about a classification near the boundary. A chord
    /// height bounds it directly.
    /// </remarks>
    private static Polyline Discretize(Curve curve, double chordTol)
    {
        // Already a polyline: its vertices are exact, so there is no error to bound.
        if (curve.TryGetPolyline(out Polyline exact))
            return exact;

        // The zeros switch off every constraint but the chord height.
        PolylineCurve? pc = curve.ToPolyline(0, 0, 0, 0, 0, chordTol, 0, 0, true);
        Polyline? pl = pc?.ToPolyline();
        if (pl == null)
            throw new ArgumentException("The curve could not be discretized into a polyline.", nameof(curve));
        return pl;
    }

    private static string StatusMessage(int status) => status switch
    {
        PicErrNotClosed => "The discretized curve is not closed, though the curve itself is. " +
                           "Raise the tolerance, or report this: the discretization lost the closure.",
        _               => $"point_in_curve refused the input (status {status}).",
    };
}
