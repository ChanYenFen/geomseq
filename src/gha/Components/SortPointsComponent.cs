using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

public sealed class SortPointsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("d146d2e4-f0c0-4ea1-a172-57e49daef001");

    // The largest n actually measured, from baseline-windows-amd64-20260915-heavy:
    // 64,000 points in 1.50 s.
    //
    // It was 16,000, set when this component's 2-opt tested every pair and took
    // 11.94 s there. sort_points.cpp got the pruned search on 2026-09-15 and
    // 64,000 fell from 248.68 s to 1.50 s, which left the old threshold marking
    // nothing at all -- and left the note beside it, warning that no cheaper
    // 2-opt existed here, describing the week before last.
    private const int TestedLimit = 64_000;

    public SortPointsComponent()
        : base("Sort Points", "SortPt",
               "Orders points to minimise travel between them (greedy k-NN + 2-opt). " +
               "With several branches, each is sorted in turn and travel continues from the last point of the previous one.",
               "GeomSeq", "Sequence")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.SortPoints;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPointParameter("Points", "P", "Points to sort, one branch per group.", GH_ParamAccess.tree);
        p.AddPointParameter("Start", "S",
            "Where travel starts. With Continuous on this seeds the first branch only; otherwise every branch. " +
            "Defaults to each branch's own first point.",
            GH_ParamAccess.item);
        p.AddBooleanParameter("Continuous", "Ct",
            "Treat the branches as one journey: each starts where the previous one ended, and its Distance " +
            "includes getting there. Off, every branch is an independent job.",
            GH_ParamAccess.item, true);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Points", "P", "Sorted points, on the paths they arrived on.", GH_ParamAccess.tree);
        p.AddIntegerParameter("Indices", "i", "Input index of each sorted point, within its own branch.", GH_ParamAccess.tree);
        p.AddNumberParameter("Distance", "D",
            "Travel length per branch, including the move into it -- from Start for the first, from the previous " +
            "branch's last point for the rest. Summing the branches therefore gives the whole journey.",
            GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        // Read as GH_Point, not Point3d: a null item in a Point3d list arrives as the origin,
        // indistinguishable from a real point there.
        if (!da.GetDataTree(0, out GH_Structure<GH_Point> tree) || tree.IsEmpty)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No points to sort.");
            return;
        }

        Point3d start = Point3d.Unset;
        bool hasStart = da.GetData(1, ref start) && start.IsValid;
        bool continuous = true;
        da.GetData(2, ref continuous);

        var pointTree = new GH_Structure<GH_Point>();
        var indexTree = new GH_Structure<GH_Integer>();
        var distanceTree = new GH_Structure<GH_Number>();

        // Where the next branch begins. Unset until the first branch that has something
        // to sort, so an empty leading branch cannot strand the journey at the origin.
        Point3d cursor = hasStart ? start : Point3d.Unset;
        int largestBranch = 0;

        for (int b = 0; b < tree.PathCount; b++)
        {
            GH_Path path = tree.Paths[b];
            pointTree.EnsurePath(path);
            indexTree.EnsurePath(path);
            distanceTree.EnsurePath(path);

            var points = new List<Point3d>();
            var sourceIndex = new List<int>();
            var skipped = new List<int>();
            IList<GH_Point> branch = tree.Branches[b];
            for (int k = 0; k < branch.Count; k++)
            {
                GH_Point? g = branch[k];
                if (g == null || !g.Value.IsValid)
                {
                    skipped.Add(k);
                    continue;
                }
                points.Add(g.Value);
                sourceIndex.Add(k);
            }

            if (skipped.Count > 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    Messages.Skipped(skipped, $"null or invalid point in branch {path}"));

            // An empty branch keeps its path and comes back empty, so the branches stay
            // lined up with whatever they were paired against upstream.
            if (points.Count == 0)
                continue;

            if (points.Count > largestBranch)
                largestBranch = points.Count;

            // Each branch starts where the last one ended; without Continuous it falls
            // back to Start, or to its own first point when Start is unconnected.
            Point3d from = continuous && cursor.IsValid ? cursor
                         : hasStart ? start
                         : points[0];

            int[] order;
            try
            {
                order = GeomSeqCore.SortPoints(points, from);
            }
            catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
                return;
            }

            var sorted = new List<Point3d>(order.Length);
            foreach (int j in order)
            {
                sorted.Add(points[j]);
                pointTree.Append(new GH_Point(points[j]), path);
                indexTree.Append(new GH_Integer(sourceIndex[j]), path);
            }

            // PathLength already counts `from` -> first point, so the move into this
            // branch lands in this branch's own Distance rather than nowhere.
            distanceTree.Append(new GH_Number(GeomSeqCore.PathLength(from, sorted)), path);

            cursor = sorted[sorted.Count - 1];
        }

        // Per branch, not in total: each branch is sorted on its own, so the cost that
        // the limit is about is the biggest one, not their sum.
        if (largestBranch > TestedLimit)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                Messages.AboveTestedLimit(largestBranch, "points in one branch", TestedLimit));

        da.SetDataTree(0, pointTree);
        da.SetDataTree(1, indexTree);
        da.SetDataTree(2, distanceTree);
    }
}
