using System;
using System.Collections.Generic;
using Rhino.Geometry;

namespace GeomSeq.Native;

/// <summary>
/// The coverage path and lattice resampling around <see cref="NativeMethods"/>: Rhino
/// curves into plane coordinates and back. The C# counterpart of hatch_region_native,
/// plan_coverage_native and resample_on_lattice_native in geometry_utils.py.
/// </summary>
/// <remarks>
/// Everything native is 2D: rings and paths are remapped into the working plane here, and
/// results come back as plane coordinates for the component to place with Plane.PointAt.
/// </remarks>
internal static class CoverageCore
{
    // As the Python wrappers: within this, vertex heights are one level.
    private const double Tol = 1e-9;

    // Field numbers of a coverage result, as coverage_path.cpp defines them.
    private const int FCellCounts = 0, FCellPoints = 1, FLineCounts = 2, FLinePoints = 3,
                      FWalkCounts = 4, FWalkPoints = 5, FOrder = 6, FPieceKinds = 7,
                      FPieceCounts = 8, FPiecePoints = 9, FPieceVisible = 10;

    internal sealed class Hatch
    {
        /// <summary>Per cell: its closed outline, first vertex not repeated.</summary>
        public List<Point2d[]> Cells { get; } = new();

        /// <summary>Per cell: its lines, bottom to top, each left end to right end.</summary>
        public List<(Point2d a, Point2d b)[]> Lines { get; } = new();

        /// <summary>Per cell: its four walks (forward, reverse, mirrored, mirrored reverse).</summary>
        public List<Point2d[][]> Walks { get; } = new();
    }

    internal sealed class Piece
    {
        public int Kind { get; init; }            // 0 fill, 1 travel
        public Point2d[] Points { get; init; } = Array.Empty<Point2d>();
        public double Visible { get; init; }      // travel no later fill covers; 0 for a fill
    }

    internal sealed class Plan
    {
        public Hatch Hatch { get; } = new();
        public List<(int cell, int walk)> Order { get; } = new();
        public List<Piece> Pieces { get; } = new();
    }

    /// <summary>
    /// The region's rings in plane coordinates. Every curve must be closed; a curve that is
    /// not already a polyline is flattened to within <paramref name="chordTol"/>.
    /// </summary>
    /// <returns>
    /// Null if any curve is unusable, with <paramref name="rejected"/> saying which and why
    /// (index, reason).
    /// </returns>
    public static List<Point2d[]>? Rings(IReadOnlyList<Curve?> curves, Plane plane, double chordTol,
                                         out List<(int index, string why)> rejected)
    {
        rejected = new List<(int, string)>();
        var rings = new List<Point2d[]>();
        for (int i = 0; i < curves.Count; i++)
        {
            Curve? c = curves[i];
            string? why = c == null ? "null (not a curve?)"
                        : !c.IsValid ? "invalid"
                        : !c.IsClosed ? "open"
                        : null;
            Polyline? pl = c != null && why == null ? Flatten(c, chordTol) : null;
            if (why == null)
                why = pl == null ? "could not be flattened"
                    : pl.Count < 4 ? $"flattened to {pl.Count} vertices"
                    : null;
            if (why != null || pl == null)
            {
                rejected.Add((i, why ?? "could not be flattened"));
                continue;
            }
            var ring = new Point2d[pl.Count];
            for (int k = 0; k < pl.Count; k++)
            {
                Point3d p = plane.RemapToPlaneSpace(pl[k], out Point3d local) ? local : pl[k];
                ring[k] = new Point2d(p.X, p.Y);
            }
            rings.Add(ring);
        }
        return rejected.Count == 0 ? rings : null;
    }

    /// <summary>A polyline's vertices are exact; anything else is straightened to within chordTol.</summary>
    public static Polyline? Flatten(Curve curve, double chordTol)
    {
        if (curve.TryGetPolyline(out Polyline exact))
            return exact;
        // The zeros switch off every constraint but the chord height.
        return curve.ToPolyline(0, 0, 0, 0, 0, chordTol, 0, 0, true)?.ToPolyline();
    }

    public static unsafe Hatch HatchRegion(List<Point2d[]> rings, double spacing, double angle, double phase, int link)
    {
        (double[] xy, int[] counts) = Pack(rings);
        IntPtr handle;
        int status;
        fixed (double* xp = xy)
        fixed (int* cp = counts)
        {
            status = NativeMethods.CoverageHatchRegion(xp, cp, counts.Length, spacing, angle, phase, link, Tol, &handle);
        }
        Check(status);
        try
        {
            var hatch = new Hatch();
            ReadHatch(handle, hatch);
            return hatch;
        }
        finally
        {
            NativeMethods.CoverageResultFree(handle);
        }
    }

    public static unsafe Plan PlanCoverage(List<Point2d[]> rings, double spacing, double angle, double phase,
                                           int link, Point2d? start, int kicks)
    {
        (double[] xy, int[] counts) = Pack(rings);
        double[] s = start is Point2d p ? new[] { p.X, p.Y } : new double[2];
        IntPtr handle;
        int status;
        fixed (double* xp = xy)
        fixed (int* cp = counts)
        fixed (double* sp = s)
        {
            status = NativeMethods.CoveragePlan(xp, cp, counts.Length, spacing, angle, phase, link,
                                                start.HasValue ? sp : null, kicks, Tol, &handle);
        }
        Check(status);
        try
        {
            var plan = new Plan();
            ReadHatch(handle, plan.Hatch);
            int[] order = Ints(handle, FOrder);
            for (int k = 0; k + 1 < order.Length; k += 2)
                plan.Order.Add((order[k], order[k + 1]));
            int[] kinds = Ints(handle, FPieceKinds);
            List<Point2d[]> points = Split(Points(handle, FPiecePoints), Ints(handle, FPieceCounts));
            double[] visible = Doubles(handle, FPieceVisible);
            for (int k = 0; k < kinds.Length; k++)
                plan.Pieces.Add(new Piece { Kind = kinds[k], Points = points[k], Visible = visible[k] });
            return plan;
        }
        finally
        {
            NativeMethods.CoverageResultFree(handle);
        }
    }

    /// <summary>
    /// Resamples runs, given in plane coordinates, onto the lattice. freeStep and minStep
    /// are already resolved: freeStep &gt; 0, minStep &gt;= 0.
    /// </summary>
    public static unsafe Point2d[] ResampleOnLattice(
        IReadOnlyList<(Point2d[] points, bool free)> runs, double spacing, double step, double angle,
        double phase, double stagger, double freeStep, bool skipRowEnds, double minStep)
    {
        var counts = new int[runs.Count];
        var free = new int[runs.Count];
        var xy = new List<double>();
        for (int r = 0; r < runs.Count; r++)
        {
            counts[r] = runs[r].points.Length;
            free[r] = runs[r].free ? 1 : 0;
            foreach (Point2d p in runs[r].points)
            {
                xy.Add(p.X);
                xy.Add(p.Y);
            }
        }
        double[] flat = xy.ToArray();
        int capacity = flat.Length + 16;
        while (true)
        {
            var output = new double[2 * capacity];
            int need, status;
            fixed (double* xp = flat)
            fixed (int* cp = counts)
            fixed (int* fp = free)
            fixed (double* op = output)
            {
                status = NativeMethods.ResampleOnLattice(xp, cp, fp, runs.Count, spacing, step, angle, phase,
                                                         stagger, freeStep, skipRowEnds ? 1 : 0, minStep, Tol,
                                                         op, capacity, &need);
            }
            if (status != 0)
                throw new ArgumentException(ResampleMessage(status));
            if (need <= capacity)
            {
                var result = new Point2d[need];
                for (int i = 0; i < need; i++)
                    result[i] = new Point2d(output[2 * i], output[2 * i + 1]);
                return result;
            }
            capacity = need;
        }
    }

    private static (double[] xy, int[] counts) Pack(List<Point2d[]> rings)
    {
        var counts = new int[rings.Count];
        var xy = new List<double>();
        for (int i = 0; i < rings.Count; i++)
        {
            counts[i] = rings[i].Length;
            foreach (Point2d p in rings[i])
            {
                xy.Add(p.X);
                xy.Add(p.Y);
            }
        }
        return (xy.ToArray(), counts);
    }

    private static void ReadHatch(IntPtr handle, Hatch hatch)
    {
        hatch.Cells.AddRange(Split(Points(handle, FCellPoints), Ints(handle, FCellCounts)));

        Point2d[] linePts = Points(handle, FLinePoints);
        int at = 0;
        foreach (int n in Ints(handle, FLineCounts))
        {
            var lines = new (Point2d, Point2d)[n];
            for (int k = 0; k < n; k++, at += 2)
                lines[k] = (linePts[at], linePts[at + 1]);
            hatch.Lines.Add(lines);
        }

        List<Point2d[]> walks = Split(Points(handle, FWalkPoints), Ints(handle, FWalkCounts));
        for (int c = 0; c + 3 < walks.Count; c += 4)
            hatch.Walks.Add(new[] { walks[c], walks[c + 1], walks[c + 2], walks[c + 3] });
    }

    private static unsafe int[] Ints(IntPtr handle, int field)
    {
        int n = NativeMethods.CoverageResultSize(handle, field);
        var values = new int[Math.Max(n, 0)];
        fixed (int* vp = values)
        {
            if (n > 0 && NativeMethods.CoverageResultInts(handle, field, vp, n) != n)
                throw new InvalidOperationException($"Reading coverage result field {field} failed.");
        }
        return values;
    }

    private static unsafe double[] Doubles(IntPtr handle, int field)
    {
        int n = NativeMethods.CoverageResultSize(handle, field);
        var values = new double[Math.Max(n, 0)];
        fixed (double* vp = values)
        {
            if (n > 0 && NativeMethods.CoverageResultDoubles(handle, field, vp, n) != n)
                throw new InvalidOperationException($"Reading coverage result field {field} failed.");
        }
        return values;
    }

    private static Point2d[] Points(IntPtr handle, int field)
    {
        double[] v = Doubles(handle, field);
        var pts = new Point2d[v.Length / 2];
        for (int i = 0; i < pts.Length; i++)
            pts[i] = new Point2d(v[2 * i], v[2 * i + 1]);
        return pts;
    }

    private static List<Point2d[]> Split(Point2d[] items, int[] counts)
    {
        var out_ = new List<Point2d[]>(counts.Length);
        int at = 0;
        foreach (int c in counts)
        {
            var part = new Point2d[c];
            Array.Copy(items, at, part, 0, c);
            out_.Add(part);
            at += c;
        }
        return out_;
    }

    private static void Check(int status)
    {
        if (status != 0)
            throw new ArgumentException(CoverageMessage(status));
    }

    // The messages coverage_path.py raises, as geometry_utils.py maps them.
    private static string CoverageMessage(int status) => status switch
    {
        -1 => "Bad argument to the native coverage path.",
        -2 => "Tolerance cannot be negative.",
        -3 => "A ring needs at least three distinct vertices.",
        -4 => "The region has no height across the line direction.",
        -5 => "A scanline crossed the boundary an odd number of times: the curves are not closed, or they cross each other.",
        -6 => "Spacing must be greater than 0.",
        -7 => "Link must be 0 (zig-zag), 1 (one-way) or 2 (one-way with seam lines).",
        -8 => "Kicks cannot be negative.",
        -9 => "The cells left lie in another piece of the region: give one connected region at a time.",
        _  => $"The native coverage path failed (status {status}).",
    };

    private static string ResampleMessage(int status) => status switch
    {
        -2 => "Spacing must be greater than 0.",
        -3 => "Step must be greater than 0.",
        -4 => "Stagger must be greater than 0.",
        -5 => "Free step must be greater than 0.",
        -6 => "Min step cannot be negative.",
        _  => $"The native resample_on_lattice failed (status {status}).",
    };
}
