using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;

namespace GeomSeq.Components;

public sealed class RedistributeLookupsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("32ba6096-d9d0-4d25-b62e-599053ae07db");

    public RedistributeLookupsComponent()
        : base("Redistribute Lookups", "Redist",
               "Respaces arc-length positions along a curve to a density profile, keeping named positions exactly. " +
               "Pure 1D arithmetic -- it never touches the curve; feed the result to a component that evaluates points at arc lengths.",
               "GeomSeq", "Sampling")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.RedistributeLookups;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Lookups", "L",
            "Arc-length positions along one curve, ascending. Only the last one is read as the curve's length.",
            GH_ParamAccess.list);
        p.AddNumberParameter("Low", "lo", "Smallest step, used where the result is densest. Must be greater than 0.",
            GH_ParamAccess.item);
        p.AddNumberParameter("High", "hi", "Largest step, used where the result is sparsest.",
            GH_ParamAccess.item);
        p.AddIntegerParameter("Mode", "M", "0 = dense centre (sparse ends), 1 = dense ends (sparse centre).",
            GH_ParamAccess.item, 0);
        p.AddNumberParameter("Flat", "F",
            "Percent of the curve, centred, held at one density; the rest fades between Low and High.",
            GH_ParamAccess.item, 0.0);
        p.AddIntegerParameter("Corners", "C",
            "Indices into L whose arc lengths must appear in the result exactly, e.g. polyline vertices.",
            GH_ParamAccess.list);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddNumberParameter("Lookups", "L", "Redistributed arc-length positions.", GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        var lookups = new List<double>();
        da.GetDataList(0, lookups);

        double low = 0.0, high = 0.0, flatPct = 0.0;
        int mode = 0;
        da.GetData(1, ref low);
        da.GetData(2, ref high);
        da.GetData(3, ref mode);
        da.GetData(4, ref flatPct);

        var corners = new List<int>();
        da.GetDataList(5, corners);

        if (lookups.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No lookups to redistribute.");
            return;
        }

        // Stops the solve rather than reporting and continuing: at low <= 0 the native
        // marching loop never advances, so Rhino would hang with no message at all.
        if (low <= 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Low must be greater than 0.");
            return;
        }

        if (high < low)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"High ({high:G6}) is below Low ({low:G6}); using {low:G6} for both.");

        double totalLength = lookups[lookups.Count - 1];
        if (high > totalLength)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"High ({high:G6}) is longer than the curve ({totalLength:G6}), so the result is just its two ends.");

        // Out-of-range indices would read past the lookup list, so they are dropped and
        // named rather than failing the whole solve -- the same trade the sort components
        // make for null geometry.
        var kept = new List<int>(corners.Count);
        var skipped = new List<int>();
        for (int k = 0; k < corners.Count; k++)
        {
            if (corners[k] < 0 || corners[k] >= lookups.Count)
                skipped.Add(k);
            else
                kept.Add(corners[k]);
        }
        if (skipped.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning, Messages.Skipped(skipped, "out-of-range corner index"));

        // The ABI wants corner arc lengths ascending, and nothing upstream guarantees the
        // indices arrive that way -- Grasshopper hands over whatever order the wire carries.
        // (geometry_utils.redistribute_lookups_native does not sort either; it has simply
        // never been fed an unsorted list.)
        kept.Sort();

        double[] result;
        try
        {
            result = GeomSeqCore.RedistributeLookups(lookups, low, high, mode, flatPct, kept);
        }
        catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
            return;
        }

        da.SetDataList(0, result);
    }
}
