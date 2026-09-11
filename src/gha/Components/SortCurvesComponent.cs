using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Rhino;
using Rhino.Geometry;

namespace GeomSeq.Components;

public sealed class SortCurvesComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("30183340-903f-45f8-a5e5-d6509ffbb897");

    // Largest input with a known running time: ~43 s for 50k curves (README, ad-hoc run).
    // Not yet backed by a committed file in benchmarks/results/; revisit once the full baseline is.
    private const int TestedLimit = 50_000;

    public SortCurvesComponent()
        : base("Sort Curves", "SortCrv",
               "Orders curves to minimise travel between them (greedy k-NN + 2-opt). " +
               "Curves may be reversed when that shortens travel, unless F is false.",
               "GeomSeq", "Sequence")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.SortCurves;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "Curves to sort.", GH_ParamAccess.list);
        p.AddPointParameter("Start", "S", "Where travel starts. Defaults to the start of the first curve.", GH_ParamAccess.item);
        p.AddBooleanParameter("Travel", "T", "Also output the travel moves as lines.", GH_ParamAccess.item, false);
        p.AddBooleanParameter("Flip", "F",
            "Allow curves to be reversed when that shortens travel. When false, every curve keeps its " +
            "direction and the 2-opt pass is skipped, so travel is usually longer.",
            GH_ParamAccess.item, true);

        // Optional so an empty list reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "Sorted curves, reversals applied.", GH_ParamAccess.list);
        p.AddIntegerParameter("Indices", "i", "Input index of each sorted curve.", GH_ParamAccess.list);
        p.AddBooleanParameter("Reversed", "R", "Whether each sorted curve was reversed.", GH_ParamAccess.list);
        p.AddNumberParameter("Distance", "D", "Total travel length, including the move from S to the first curve.", GH_ParamAccess.item);
        p.AddLineParameter("Travel", "T", "Travel moves, only when input T is true. Line k leads into curve k.", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        var input = new List<Curve?>();
        da.GetDataList(0, input);
        Point3d start = Point3d.Unset;
        bool hasStart = da.GetData(1, ref start) && start.IsValid;
        bool wantTravel = false;
        da.GetData(2, ref wantTravel);
        bool allowFlip = true;
        da.GetData(3, ref allowFlip);

        if (input.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to sort.");
            return;
        }

        // Sort only usable curves; sourceIndex maps each back to its input position.
        var curves = new List<Curve>(input.Count);
        var sourceIndex = new List<int>(input.Count);
        var skipped = new List<int>();
        for (int k = 0; k < input.Count; k++)
        {
            Curve? c = input[k];
            if (c == null || !c.IsValid || c.IsShort(RhinoMath.ZeroTolerance))
            {
                skipped.Add(k);
                continue;
            }
            curves.Add(c);
            sourceIndex.Add(k);
        }

        if (skipped.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, Messages.Skipped(skipped, "null or degenerate curve"));
        if (curves.Count == 0)
            return;
        if (curves.Count > TestedLimit)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, Messages.AboveTestedLimit(curves.Count, "curves", TestedLimit));

        if (!hasStart)
            start = curves[0].PointAtStart;

        GeomSeqCore.CurveSortResult result;
        try
        {
            result = GeomSeqCore.SortCurves(curves, start, allowFlip);
        }
        catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
            return;
        }

        int n = curves.Count;
        var sorted = new List<Curve>(n);
        var indices = new List<int>(n);
        var reversed = new List<bool>(n);
        for (int k = 0; k < n; k++)
        {
            int j = result.Order[k];
            bool rev = result.Reversal[k] != 0;
            Curve c = curves[j].DuplicateCurve();
            if (rev)
                c.Reverse();
            sorted.Add(c);
            indices.Add(sourceIndex[j]);
            reversed.Add(rev);
        }

        da.SetDataList(0, sorted);
        da.SetDataList(1, indices);
        da.SetDataList(2, reversed);
        da.SetData(3, result.TravelLength());

        if (wantTravel)
        {
            var travel = new List<Line>(n);
            for (int k = 0; k < n; k++)
                travel.Add(result.TravelSegment(k));
            da.SetDataList(4, travel);
        }
    }
}
