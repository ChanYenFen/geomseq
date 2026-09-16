using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// Builds the travel path between consecutive strokes: each end is extended into the
/// gap and its corner filleted, so no waypoint turns by more than the given angle.
/// </summary>
/// <remarks>
/// The native turn is planar and cannot be otherwise: it picks which way to turn from
/// the sign of a 2D cross product, and in three dimensions "which way" has no meaning
/// without a reference normal. Adding z to the ABI would not make it 3D -- what it
/// needs is a plane to turn in.
///
/// So the plane is the component's, not the core's. Everything is converted into plane
/// coordinates, the 2D core runs unchanged, and the waypoints are mapped back out. With
/// no plane supplied that plane is world XY, which is exactly what this did before the
/// input existed, so saved definitions keep their results.
/// </remarks>
public sealed class BuildTurnWaypointsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("b931d813-b284-4d8e-b364-bd8a07f7c12f");

    public BuildTurnWaypointsComponent()
        : base("Build Turn Waypoints", "Turn",
               "Smooth travel between the end of one stroke and the start of the next, one branch per turn. " +
               "The turn is built in a plane -- world XY unless another is given.",
               "GeomSeq", "Travel")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.BuildTurnWaypoints;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPointParameter("Points", "P",
            "One branch per stroke, in travel order. A turn is built between each consecutive pair.",
            GH_ParamAccess.tree);
        p.AddNumberParameter("Max Angle", "A", "Largest turn allowed at any one waypoint, in degrees.",
            GH_ParamAccess.item, 30.0);
        p.AddNumberParameter("Step", "St", "Fillet step length. Must be greater than 0.",
            GH_ParamAccess.item);
        p.AddNumberParameter("Extend", "E", "How far past each stroke's end the fillet corner sits.",
            GH_ParamAccess.item, 0.0);
        p.AddPlaneParameter("Plane", "Pl",
            "Plane the turn is built in. Defaults to world XY; points off it are flattened onto it.",
            GH_ParamAccess.item);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[2].Optional = true;
        p[4].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Waypoints", "W",
            "Travel waypoints for each turn: leaving one stroke, then arriving at the next.",
            GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        if (!da.GetDataTree(0, out GH_Structure<GH_Point> strokeTree) || strokeTree.IsEmpty)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No strokes to join.");
            return;
        }

        double thetaMaxDeg = 30.0, stepLen = 0.0, extendLen = 0.0;
        da.GetData(1, ref thetaMaxDeg);
        da.GetData(2, ref stepLen);
        da.GetData(3, ref extendLen);

        Plane plane = Plane.WorldXY;
        if (da.GetData(4, ref plane) && !plane.IsValid)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Plane is not valid.");
            return;
        }

        // Stops the solve rather than reporting and continuing: the fillet advances by
        // this, so at zero the native side makes no progress.
        if (stepLen <= 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Step must be greater than 0.");
            return;
        }

        if (strokeTree.PathCount < 2)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "A turn needs two strokes; only one branch arrived.");
            return;
        }

        var output = new GH_Structure<GH_Point>();
        var tooShort = new List<int>();
        int offPlane = 0;
        int roughJunctions = 0;

        for (int i = 0; i < strokeTree.PathCount - 1; i++)
        {
            List<Point3d> stroke = Branch(strokeTree, i);
            List<Point3d> next = Branch(strokeTree, i + 1);

            // A stroke needs two points to have a heading at all.
            if (stroke.Count < 2 || next.Count < 2)
            {
                tooShort.Add(i);
                continue;
            }

            // Only these four points reach the core, so only their out-of-plane offsets
            // are discarded -- counting every point in the branch would inflate the
            // number with points the turn never looks at.
            Point2d e = ToPlane(stroke[stroke.Count - 1], plane, ref offPlane);
            Point2d ePrev = ToPlane(stroke[stroke.Count - 2], plane, ref offPlane);
            Point2d s = ToPlane(next[0], plane, ref offPlane);
            Point2d sNext = ToPlane(next[1], plane, ref offPlane);

            double avx = e.X - ePrev.X, avy = e.Y - ePrev.Y;
            double bvx = sNext.X - s.X, bvy = sNext.Y - s.Y;

            // Both fillets need room to open up before the junction between them is
            // smooth; the native side caps the angle within each fillet but cannot see
            // the other one. Reported rather than refused -- the result is still usable,
            // just sharper at the join than the angle suggests.
            if (IsJunctionRough(e, s, avx, avy, bvx, bvy, stepLen, extendLen))
                roughJunctions++;

            GeomSeqCore.TurnWaypoints turn;
            try
            {
                turn = GeomSeqCore.BuildTurnWaypoints(e.X, e.Y, avx, avy, s.X, s.Y, bvx, bvy,
                                                      thetaMaxDeg, stepLen, extendLen);
            }
            catch (Exception ex) when (ex is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + ex.Message);
                return;
            }

            // Pathed by the stroke the turn leaves, not by a running count of turns.
            // The GHPython original numbers turns consecutively, so one skipped stroke
            // shifts every later branch -- the same pairing failure that made
            // Redistribute Lookups take trees. A skipped turn leaves a gap instead.
            var path = new GH_Path(i);
            output.EnsurePath(path);
            AppendPairs(output, path, turn.Exit, plane);
            AppendPairs(output, path, turn.Entry, plane);
        }

        if (tooShort.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                Messages.Skipped(tooShort, "turn whose stroke has fewer than two points"));
        if (offPlane > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{offPlane} of the points defining these turns sit off the plane and were flattened onto it. " +
                "The turn is built in plan; supply a Plane if a different one suits the geometry better.");
        if (roughJunctions > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{roughJunctions} turn(s) had too little room between the strokes for the two fillets to meet smoothly; " +
                "the angle holds within each fillet but not across the join. A smaller Extend or Step opens it up.");

        da.SetDataTree(0, output);
    }

    private static List<Point3d> Branch(GH_Structure<GH_Point> tree, int index)
    {
        IList<GH_Point> branch = tree.Branches[index];
        var points = new List<Point3d>(branch.Count);
        foreach (GH_Point? item in branch)
        {
            if (item != null && item.Value.IsValid)
                points.Add(item.Value);
        }
        return points;
    }

    /// <summary>
    /// World point to plane coordinates, by projection onto the plane's axes. Done with
    /// dot products rather than a helper so the discarded component -- the distance off
    /// the plane -- is visible and countable rather than an API's business.
    /// </summary>
    private static Point2d ToPlane(Point3d pt, Plane plane, ref int offPlane)
    {
        Vector3d d = pt - plane.Origin;
        if (Math.Abs(d * plane.ZAxis) > OffPlaneTolerance)
            offPlane++;
        return new Point2d(d * plane.XAxis, d * plane.YAxis);
    }

    // A reporting threshold, not a geometric one. RhinoMath.ZeroTolerance is 1e-12,
    // tight enough that ordinary float noise in user geometry would trip it and make
    // the count meaningless; the document tolerance would be better calibrated but
    // would tie this component to whichever file happens to be open.
    private const double OffPlaneTolerance = 1e-9;

    /// <summary>
    /// The two checks geometry_utils prints, in plane coordinates: the strokes closer
    /// together than the two extends need, and the bridge between the extend points
    /// shorter than one step.
    /// </summary>
    private static bool IsJunctionRough(
        Point2d e, Point2d s, double avx, double avy, double bvx, double bvy,
        double stepLen, double extendLen)
    {
        double gap = Math.Sqrt((s.X - e.X) * (s.X - e.X) + (s.Y - e.Y) * (s.Y - e.Y));
        if (gap < 2.0 * extendLen)
            return true;

        Unit(avx, avy, out double ahx, out double ahy);
        Unit(bvx, bvy, out double bhx, out double bhy);
        double bx = (s.X - bhx * extendLen) - (e.X + ahx * extendLen);
        double by = (s.Y - bhy * extendLen) - (e.Y + ahy * extendLen);
        return Math.Sqrt(bx * bx + by * by) < stepLen;
    }

    /// <summary>Matches native unit_vec: a zero-length vector is returned unchanged.</summary>
    private static void Unit(double vx, double vy, out double ux, out double uy)
    {
        double length = Math.Sqrt(vx * vx + vy * vy);
        if (length == 0.0)
        {
            ux = vx;
            uy = vy;
            return;
        }
        ux = vx / length;
        uy = vy / length;
    }

    private static void AppendPairs(GH_Structure<GH_Point> output, GH_Path path, double[] flat, Plane plane)
    {
        for (int k = 0; k + 1 < flat.Length; k += 2)
        {
            Point3d world = plane.Origin + plane.XAxis * flat[k] + plane.YAxis * flat[k + 1];
            output.Append(new GH_Point(world), path);
        }
    }
}
