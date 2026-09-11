using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

public sealed class SortPointsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("d146d2e4-f0c0-4ea1-a172-57e49daef001");

    // Unmeasured. sort_points has no windowed 2-opt, so every pass is O(n^2); 10,000 is where
    // sort_curves gives up on exhaustive 2-opt for speed. Replace once sort_points has a
    // committed result in benchmarks/results/.
    private const int TestedLimit = 10_000;

    public SortPointsComponent()
        : base("Sort Points", "SortPt",
               "Orders points to minimise travel between them (greedy k-NN + 2-opt).",
               "GeomSeq", "Sequence")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.SortPoints;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddPointParameter("Points", "P", "Points to sort.", GH_ParamAccess.list);
        p.AddPointParameter("Start", "S", "Where travel starts. Defaults to the first point.", GH_ParamAccess.item);

        // Optional so an empty list reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Points", "P", "Sorted points.", GH_ParamAccess.list);
        p.AddIntegerParameter("Indices", "i", "Input index of each sorted point.", GH_ParamAccess.list);
        p.AddNumberParameter("Distance", "D", "Total path length, including the move from S to the first point.", GH_ParamAccess.item);
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
        var input = new List<GH_Point?>();
        da.GetDataList(0, input);
        Point3d start = Point3d.Unset;
        bool hasStart = da.GetData(1, ref start) && start.IsValid;

        if (input.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No points to sort.");
            return;
        }

        var points = new List<Point3d>(input.Count);
        var sourceIndex = new List<int>(input.Count);
        var skipped = new List<int>();
        for (int k = 0; k < input.Count; k++)
        {
            GH_Point? g = input[k];
            if (g == null || !g.Value.IsValid)
            {
                skipped.Add(k);
                continue;
            }
            points.Add(g.Value);
            sourceIndex.Add(k);
        }

        if (skipped.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, Messages.Skipped(skipped, "null or invalid point"));
        if (points.Count == 0)
            return;
        if (points.Count > TestedLimit)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, Messages.AboveTestedLimit(points.Count, "points", TestedLimit));

        if (!hasStart)
            start = points[0];

        int[] order;
        try
        {
            order = GeomSeqCore.SortPoints(points, start);
        }
        catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
            return;
        }

        var sorted = new List<Point3d>(order.Length);
        var indices = new List<int>(order.Length);
        foreach (int j in order)
        {
            sorted.Add(points[j]);
            indices.Add(sourceIndex[j]);
        }

        da.SetDataList(0, sorted);
        da.SetDataList(1, indices);
        da.SetData(2, GeomSeqCore.PathLength(start, sorted));
    }
}
