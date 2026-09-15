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
public sealed class BuildTurnWaypointsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("b931d813-b284-4d8e-b364-bd8a07f7c12f");

    public BuildTurnWaypointsComponent()
        : base("Build Turn Waypoints", "Turn",
               "Smooth travel between the end of one stroke and the start of the next, one branch per turn. " +
               "2D only: z is ignored on the way in and zero on the way out.",
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

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[2].Optional = true;
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
        int flattenedZ = 0;
        int roughJunctions = 0;

        for (int i = 0; i < strokeTree.PathCount - 1; i++)
        {
            List<Point3d> stroke = Branch(strokeTree, i, ref flattenedZ);
            List<Point3d> next = Branch(strokeTree, i + 1, ref flattenedZ);

            // A stroke needs two points to have a heading at all.
            if (stroke.Count < 2 || next.Count < 2)
            {
                tooShort.Add(i);
                continue;
            }

            Point3d e = stroke[stroke.Count - 1];
            Point3d s = next[0];
            Vector3d aVec = e - stroke[stroke.Count - 2];
            Vector3d bVec = next[1] - s;

            // Both fillets need room to open up before the junction between them is
            // smooth; the native side caps the angle within each fillet but cannot see
            // the other one. Reported rather than refused -- the result is still usable,
            // just sharper at the join than the angle suggests.
            if (IsJunctionRough(e, s, aVec, bVec, stepLen, extendLen))
                roughJunctions++;

            GeomSeqCore.TurnWaypoints turn;
            try
            {
                turn = GeomSeqCore.BuildTurnWaypoints(e.X, e.Y, aVec.X, aVec.Y,
                                                      s.X, s.Y, bVec.X, bVec.Y,
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
            AppendPairs(output, path, turn.Exit);
            AppendPairs(output, path, turn.Entry);
        }

        if (tooShort.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                Messages.Skipped(tooShort, "turn whose stroke has fewer than two points"));
        if (flattenedZ > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{flattenedZ} input point(s) had a non-zero z, which this ignores: the turn is built in plan and comes back at z = 0.");
        if (roughJunctions > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{roughJunctions} turn(s) had too little room between the strokes for the two fillets to meet smoothly; " +
                "the angle holds within each fillet but not across the join. A smaller Extend or Step opens it up.");

        da.SetDataTree(0, output);
    }

    private static List<Point3d> Branch(GH_Structure<GH_Point> tree, int index, ref int flattenedZ)
    {
        IList<GH_Point> branch = tree.Branches[index];
        var points = new List<Point3d>(branch.Count);
        foreach (GH_Point? item in branch)
        {
            if (item == null || !item.Value.IsValid)
                continue;
            if (item.Value.Z != 0.0)
                flattenedZ++;
            points.Add(item.Value);
        }
        return points;
    }

    /// <summary>
    /// The two checks geometry_utils prints: the strokes closer together than the two
    /// extends need, and the bridge between the extend points shorter than one step.
    /// </summary>
    private static bool IsJunctionRough(
        Point3d e, Point3d s, Vector3d aVec, Vector3d bVec, double stepLen, double extendLen)
    {
        double gap = Math.Sqrt((s.X - e.X) * (s.X - e.X) + (s.Y - e.Y) * (s.Y - e.Y));
        if (gap < 2.0 * extendLen)
            return true;

        Vector3d a = Unit(aVec);
        Vector3d b = Unit(bVec);
        double bx = (s.X - b.X * extendLen) - (e.X + a.X * extendLen);
        double by = (s.Y - b.Y * extendLen) - (e.Y + a.Y * extendLen);
        return Math.Sqrt(bx * bx + by * by) < stepLen;
    }

    /// <summary>Matches native unit_vec: a zero-length vector is returned unchanged.</summary>
    private static Vector3d Unit(Vector3d v)
    {
        double length = Math.Sqrt(v.X * v.X + v.Y * v.Y);
        return length == 0.0 ? v : new Vector3d(v.X / length, v.Y / length, 0.0);
    }

    private static void AppendPairs(GH_Structure<GH_Point> output, GH_Path path, double[] flat)
    {
        for (int k = 0; k + 1 < flat.Length; k += 2)
            output.Append(new GH_Point(new Point3d(flat[k], flat[k + 1], 0.0)), path);
    }
}
