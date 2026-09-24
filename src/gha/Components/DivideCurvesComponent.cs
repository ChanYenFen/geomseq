using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// Divides curves into points, a continuous arc length, and the indices of the
/// joints between segments. Feeds Redistribute Arc Lengths and Sample Curve Points.
/// </summary>
/// <remarks>
/// Like Sample Curve Points and unlike everything else here, there is no native code
/// behind this: it is rhino_utils/divide_curves.py reimplemented against RhinoCommon,
/// so the same behaviour now exists twice and pytest can reach neither copy of this one.
///
/// It is ported as it behaves, not as it reads, including three things that are almost
/// certainly not what the original author meant. They are marked at each site. Repairing
/// them here would make the two implementations disagree silently, which is worse than
/// either of them being wrong in the same way -- fix them in both, deliberately, or not
/// at all.
/// </remarks>
public sealed class DivideCurvesComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("698e7c0d-e6b9-4c80-9879-4195c8186507");

    // From rhino_utils/divide_curves.py. Above this, a curve is divided once rather
    // than by length -- a hard switch the caller cannot see or decline, so the
    // component reports when it fires.
    private const double MaxSegLength = 300.0;

    public DivideCurvesComponent()
        : base("Divide Curves", "DivCrv",
               "Divides each curve into points, a continuous arc length, and the indices of its segment joints. " +
               "Multi-segment curves are exploded first, so the joints survive as exact positions.",
               "GeomSeq", "Division")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.DivideCurves;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "Curves to divide.", GH_ParamAccess.list);
        p.AddNumberParameter("Segment Length", "S",
            "Target spacing. One value for all curves, or one per curve; a short list reuses its last entry.",
            GH_ParamAccess.list);
        p.AddBooleanParameter("Join Ends", "J",
            "Closed curves only: repeat the first position at the end so the loop closes.",
            GH_ParamAccess.item, true);
        p.AddBooleanParameter("Overlap", "O",
            "Closed curves only: add one point past the end, continuing into the start.",
            GH_ParamAccess.item, false);
        p.AddNumberParameter("Overlap Length", "OL",
            "How far past the end to overlap. Defaults to that curve's Segment Length.",
            GH_ParamAccess.item);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
        p[4].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Points", "P", "Division points, one branch per curve.", GH_ParamAccess.tree);
        p.AddNumberParameter("Arc Lengths", "S",
            "Positions along the whole curve, continuous across segment joints, one branch per curve.",
            GH_ParamAccess.tree);
        p.AddIntegerParameter("Corners", "C",
            "Indices into Arc Lengths where one segment meets the next.", GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var curves = new List<Curve?>();
        da.GetDataList(0, curves);

        var segmentLengths = new List<double>();
        da.GetDataList(1, segmentLengths);

        bool joinEnds = true, overlap = false;
        da.GetData(2, ref joinEnds);
        da.GetData(3, ref overlap);

        double overlapLength = 0.0;
        bool hasOverlapLength = da.GetData(4, ref overlapLength);

        if (curves.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to divide.");
            return;
        }
        if (segmentLengths.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No segment length given.");
            return;
        }

        var pointTree = new GH_Structure<GH_Point>();
        var arcLengthTree = new GH_Structure<GH_Number>();
        var cornerTree = new GH_Structure<GH_Integer>();

        var skipped = new List<int>();
        int singleDivision = 0;

        for (int i = 0; i < curves.Count; i++)
        {
            var path = new GH_Path(i);
            pointTree.EnsurePath(path);
            arcLengthTree.EnsurePath(path);
            cornerTree.EnsurePath(path);

            Curve? curve = curves[i];
            if (curve == null || !curve.IsValid)
            {
                // The branch keeps its path and stays empty, so later curves do not shift.
                skipped.Add(i);
                continue;
            }

            // Scalar broadcast, or per-curve with the last entry reused -- the rule
            // _resolve_segment_length applies in the GHPython component.
            double segLength = segmentLengths[i < segmentLengths.Count ? i : segmentLengths.Count - 1];
            if (segLength <= 0.0)
            {
                skipped.Add(i);
                continue;
            }
            if (segLength > MaxSegLength)
                singleDivision++;

            double thisOverlapLength = hasOverlapLength ? overlapLength : segLength;

            ProcessCurve(curve, segLength, joinEnds, overlap, thisOverlapLength,
                         path, pointTree, arcLengthTree, cornerTree);
        }

        if (skipped.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                Messages.Skipped(skipped, "null curve or non-positive segment length"));
        if (singleDivision > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{singleDivision} curve(s) had a Segment Length above {MaxSegLength:G6} and were divided once " +
                "instead of by length.");

        da.SetDataTree(0, pointTree);
        da.SetDataTree(1, arcLengthTree);
        da.SetDataTree(2, cornerTree);
    }

    /// <summary>process_curve() from divide_curves.py, writing straight into the output trees.</summary>
    private static void ProcessCurve(
        Curve curve, double segLength, bool joinEnds, bool overlap, double overlapLength,
        GH_Path path,
        GH_Structure<GH_Point> pointTree,
        GH_Structure<GH_Number> arcLengthTree,
        GH_Structure<GH_Integer> cornerTree)
    {
        Curve[] segments = GetCurveSegments(curve);
        bool isClosed = curve.IsClosed;

        var points = new List<Point3d>();
        int arcLengthCount = 0;     // mirrors len(crv_arc_lengths) as the Python builds it
        double offset = 0.0;     // arc length of the segments already walked

        for (int j = 0; j < segments.Length; j++)
        {
            Curve seg = segments[j];

            // set_crv_domain: the parameter interval is rescaled to [0, length]. Note this
            // is a linear rescale, not an arc-length reparameterisation, so the values that
            // end up in Arc Lengths equal arc length only where the curve has uniform speed --
            // a line or an arc. Downstream, Sample Curve Points feeds them to
            // LengthParameter, which does treat them as arc lengths. Carried over as-is.
            double segLen = seg.GetLength();
            seg.Domain = new Interval(0.0, segLen);

            int count = GetDivideCount(segLen, segLength);
            double[]? ps = seg.DivideByCount(count, true);
            if (ps == null || ps.Length == 0)
                continue;

            var parameters = new List<double>(ps);
            DedupeSegmentParams(parameters, j == segments.Length - 1, segments.Length == 1,
                                isClosed, joinEnds);

            foreach (double t in parameters)
                points.Add(seg.PointAt(t));

            if (j > 0)
                cornerTree.Append(new GH_Integer(arcLengthCount), path);

            foreach (double t in parameters)
            {
                arcLengthTree.Append(new GH_Number(t + offset), path);
                arcLengthCount++;
            }

            offset += segLen;
        }

        if (overlap && isClosed)
            AddOverlapPoint(points, overlapLength, joinEnds);

        foreach (Point3d pt in points)
            pointTree.Append(new GH_Point(pt), path);
    }

    /// <summary>
    /// get_divide_count(): above MaxSegLength the curve is divided once, whatever its length.
    /// </summary>
    /// <remarks>
    /// The epsilon is not decoration. A length that is exactly n segments of S does not
    /// always divide to exactly n, because S * n need not round-trip in binary: at
    /// S = 0.1 and n = 3 the ratio computes as 3.0000000000000004, Ceiling returns 4, and
    /// every spacing on the curve silently becomes 0.075 -- a quarter short of what was
    /// asked for. Measured over exact multiples of nine different S values from 0.1 to
    /// 6.1, 39 of 531 did this (n = 3, 6, 12, 24, 29 and 48 among them); the error is
    /// worst at small n, where one extra division is a large share of the total.
    ///
    /// Relative rather than absolute, so it still absorbs the same few ULPs when the
    /// ratio is in the thousands. Far too small to swallow a fractional part anyone
    /// meant: a genuine n + 1e-9 still rounds up.
    ///
    /// rhino_utils/divide_curves.py carries the identical line and the identical fix.
    /// Changing one without the other is how the two implementations start disagreeing.
    /// </remarks>
    private static int GetDivideCount(double curveLength, double segmentLength)
    {
        if (segmentLength > MaxSegLength)
            return 1;

        double raw = curveLength / segmentLength;
        return Math.Max(1, (int)Math.Ceiling(raw - raw * 1e-12));
    }

    /// <summary>
    /// get_curve_segments(). check_explodable() is reproduced faithfully and its answer is
    /// always true: the Python loop increments its counter before testing anything, so
    /// `count > 0` holds even for a curve with no discontinuity at all. Every curve is
    /// therefore exploded, which is what this has always done.
    /// </summary>
    private static Curve[] GetCurveSegments(Curve curve)
    {
        // check_explodable() also passes GetLength() as the parameter upper bound, though
        // the domain has not been rescaled at this point, so the two are unrelated. Kept,
        // because the result is discarded either way.
        Curve[]? segments = curve.DuplicateSegments();
        if (segments == null || segments.Length == 0)
            return new[] { curve };
        return segments;
    }

    /// <summary>
    /// _dedupe_segment_params(): drops the position that would repeat the next segment's
    /// start, except where a closed curve's own closing position is wanted.
    /// </summary>
    private static void DedupeSegmentParams(
        List<double> parameters, bool isLast, bool isSingleSegment, bool isClosed, bool joinEnds)
    {
        if (parameters.Count == 0)
            return;

        if (isSingleSegment)
        {
            if (isClosed && joinEnds)
                parameters.Add(parameters[0]);
            return;
        }

        if (isLast)
        {
            if (isClosed && !joinEnds)
                parameters.RemoveAt(parameters.Count - 1);
            return;
        }

        parameters.RemoveAt(parameters.Count - 1);
    }

    /// <summary>
    /// get_overlap(): continues one step past the last point, into the start of the loop.
    /// The Python clamps an over-long overlap to the reference distance and then tests
    /// `overlap_length <= check`, which after the clamp is always true; the same single
    /// point is appended here.
    /// </summary>
    private static void AddOverlapPoint(List<Point3d> points, double overlapLength, bool joinEnds)
    {
        if (points.Count < 2)
            return;

        double check = joinEnds
            ? points[1].DistanceTo(points[0])
            : points[0].DistanceTo(points[points.Count - 1]);

        if (overlapLength > check)
            overlapLength = check;

        Vector3d direction = joinEnds
            ? points[1] - points[0]
            : points[0] - points[points.Count - 1];

        if (!direction.Unitize())
            return;

        points.Add(points[points.Count - 1] + direction * overlapLength);
    }
}
