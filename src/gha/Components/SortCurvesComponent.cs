using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;

namespace GeomSeq.Components;

public sealed class SortCurvesComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("30183340-903f-45f8-a5e5-d6509ffbb897");

    // The largest n actually measured for curves: 50,000, in
    // baseline-windows-amd64-20260914-sort_curves_prune_check-pruned-heavy,
    // where a 20-pass solve takes 2.62 s.
    //
    // It was 16,000, from a build whose 2-opt tested every pair and needed ~12 s
    // there. Pruning made that number meaningless -- 16,000 curves now finish in
    // about 0.4 s -- so the warning would have fired, in orange, on work that
    // ends before the user lets go of the mouse. A warning that cries wolf about
    // instant results teaches people to ignore it.
    private const int TestedLimit = 50_000;

    public SortCurvesComponent()
        : base("Sort Curves", "SortCrv",
               "Orders curves to minimise travel between them (greedy k-NN + 2-opt). " +
               "Curves may be reversed when that shortens travel, unless F is false. " +
               "With several branches, each is sorted in turn and travel continues from the end of the previous one.",
               "GeomSeq", "Sequence")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.SortCurves;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "Curves to sort, one branch per group.", GH_ParamAccess.tree);
        p.AddPointParameter("Start", "S",
            "Where travel starts. With Continuous on this seeds the first branch only; otherwise every branch. " +
            "Defaults to the start of each branch's own first curve.",
            GH_ParamAccess.item);
        p.AddBooleanParameter("Travel", "T", "Also output the travel moves as lines.", GH_ParamAccess.item, false);
        p.AddBooleanParameter("Flip", "F",
            "Allow curves to be reversed when that shortens travel. When false, every curve keeps its " +
            "direction and the 2-opt pass is skipped, so travel is usually longer.",
            GH_ParamAccess.item, true);
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
        p.AddCurveParameter("Curves", "C", "Sorted curves, reversals applied, on the paths they arrived on.", GH_ParamAccess.tree);
        p.AddIntegerParameter("Indices", "i", "Input index of each sorted curve, within its own branch.", GH_ParamAccess.tree);
        p.AddBooleanParameter("Reversed", "R", "Whether each sorted curve was reversed.", GH_ParamAccess.tree);
        p.AddNumberParameter("Distance", "D",
            "Travel length per branch, including the move into it -- from Start for the first, from the previous " +
            "branch's last curve for the rest. Summing the branches therefore gives the whole journey.",
            GH_ParamAccess.tree);
        p.AddLineParameter("Travel", "T", "Travel moves, only when input T is true. Line k leads into curve k.", GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        if (!da.GetDataTree(0, out GH_Structure<GH_Curve> tree) || tree.IsEmpty)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to sort.");
            return;
        }

        Point3d start = Point3d.Unset;
        bool hasStart = da.GetData(1, ref start) && start.IsValid;
        bool wantTravel = false;
        da.GetData(2, ref wantTravel);
        bool allowFlip = true;
        da.GetData(3, ref allowFlip);
        bool continuous = true;
        da.GetData(4, ref continuous);

        var curveTree = new GH_Structure<GH_Curve>();
        var indexTree = new GH_Structure<GH_Integer>();
        var reversedTree = new GH_Structure<GH_Boolean>();
        var distanceTree = new GH_Structure<GH_Number>();
        var travelTree = new GH_Structure<GH_Line>();

        // Where the next branch begins. Unset until the first branch that has something
        // to sort, so an empty leading branch cannot strand the journey at the origin.
        Point3d cursor = hasStart ? start : Point3d.Unset;
        int largestBranch = 0;

        for (int b = 0; b < tree.PathCount; b++)
        {
            GH_Path path = tree.Paths[b];
            curveTree.EnsurePath(path);
            indexTree.EnsurePath(path);
            reversedTree.EnsurePath(path);
            distanceTree.EnsurePath(path);
            if (wantTravel)
                travelTree.EnsurePath(path);

            // Sort only usable curves; sourceIndex maps each back to its input position.
            var curves = new List<Curve>();
            var sourceIndex = new List<int>();
            var skipped = new List<int>();
            IList<GH_Curve> branch = tree.Branches[b];
            for (int k = 0; k < branch.Count; k++)
            {
                GH_Curve? g = branch[k];
                Curve? c = g?.Value;
                if (c == null || !c.IsValid || c.IsShort(RhinoMath.ZeroTolerance))
                {
                    skipped.Add(k);
                    continue;
                }
                curves.Add(c);
                sourceIndex.Add(k);
            }

            if (skipped.Count > 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    Messages.Skipped(skipped, $"null or degenerate curve in branch {path}"));

            // An empty branch keeps its path and comes back empty, so the branches stay
            // lined up with whatever they were paired against upstream.
            if (curves.Count == 0)
                continue;

            if (curves.Count > largestBranch)
                largestBranch = curves.Count;

            // Each branch starts where the last one ended; without Continuous it falls
            // back to Start, or to its own first curve when Start is unconnected.
            Point3d from = continuous && cursor.IsValid ? cursor
                         : hasStart ? start
                         : curves[0].PointAtStart;

            GeomSeqCore.CurveSortResult result;
            try
            {
                result = GeomSeqCore.SortCurves(curves, from, allowFlip);
            }
            catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
                return;
            }

            int n = curves.Count;
            Curve? last = null;
            for (int k = 0; k < n; k++)
            {
                int j = result.Order[k];
                bool rev = result.Reversal[k] != 0;
                Curve c = curves[j].DuplicateCurve();
                if (rev)
                    c.Reverse();

                curveTree.Append(new GH_Curve(c), path);
                indexTree.Append(new GH_Integer(sourceIndex[j]), path);
                reversedTree.Append(new GH_Boolean(rev), path);
                if (wantTravel)
                    travelTree.Append(new GH_Line(result.TravelSegment(k)), path);
                last = c;
            }

            // TravelLength already counts `from` -> the first curve, so the move into
            // this branch lands in this branch's own Distance rather than nowhere.
            distanceTree.Append(new GH_Number(result.TravelLength()), path);

            if (last != null)
                cursor = last.PointAtEnd;
        }

        // Per branch, not in total: each branch is sorted on its own, so the cost that
        // the limit is about is the biggest one, not their sum.
        if (largestBranch > TestedLimit)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                Messages.AboveTestedLimit(largestBranch, "curves in one branch", TestedLimit));

        da.SetDataTree(0, curveTree);
        da.SetDataTree(1, indexTree);
        da.SetDataTree(2, reversedTree);
        da.SetDataTree(3, distanceTree);
        if (wantTravel)
            da.SetDataTree(4, travelTree);
    }
}
