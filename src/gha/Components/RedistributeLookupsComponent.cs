using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;

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
    //
    // Lookups and Corners are trees rather than lists, and that is not a style choice.
    // Under list access Grasshopper solves once per branch and appends the iteration
    // index to every output path, so a {0;0} {0;1} input came back as {0;0;0} {0;1;0}
    // -- one level deeper than it went in, which breaks pairing with the curves the
    // lookups came from. Taking trees keeps the paths the caller sent.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddNumberParameter("Lookups", "L",
            "Arc-length positions along one curve per branch, ascending. Only the last one in a branch is read as that curve's length.",
            GH_ParamAccess.tree);
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
            "Indices into the matching Lookups branch whose arc lengths must appear in the result exactly, e.g. polyline vertices.",
            GH_ParamAccess.tree);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning. Low and High have no sensible default
        // to fall back on, but leaving them required made this the one component that sat
        // orange on a fresh canvas: Grasshopper warns before SolveInstance ever runs, so
        // the quiet "nothing connected yet" Remark below never got the chance. Unconnected
        // now reaches that Remark; Low missing while Lookups are present still stops the
        // solve, because zero would leave the native marching loop standing still.
        p[0].Optional = true;
        p[1].Optional = true;
        p[2].Optional = true;
        p[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddNumberParameter("Lookups", "L",
            "Redistributed arc-length positions, on the same paths as the input.", GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        if (!da.GetDataTree(0, out GH_Structure<GH_Number> lookupTree) || lookupTree.IsEmpty)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No lookups to redistribute.");
            return;
        }

        da.GetDataTree(5, out GH_Structure<GH_Integer> cornerTree);

        double low = 0.0, high = 0.0, flatPct = 0.0;
        int mode = 0;
        da.GetData(1, ref low);
        da.GetData(2, ref high);
        da.GetData(3, ref mode);
        da.GetData(4, ref flatPct);

        // Stops the solve rather than reporting and continuing: at low <= 0 the native
        // marching loop never advances, so Rhino would hang with nothing on screen.
        if (low <= 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Low must be greater than 0.");
            return;
        }

        if (high < low)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"High ({high:G6}) is below Low ({low:G6}); using {low:G6} for both.");

        int cornerBranches = cornerTree?.PathCount ?? 0;
        if (cornerBranches > 0 && cornerBranches != lookupTree.PathCount)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"Corners has {cornerBranches} branch(es) against Lookups' {lookupTree.PathCount}; " +
                "the last one is reused for the remainder.");

        var output = new GH_Structure<GH_Number>();

        for (int b = 0; b < lookupTree.PathCount; b++)
        {
            GH_Path path = lookupTree.Paths[b];
            IList<GH_Number> branch = lookupTree.Branches[b];

            // An empty branch keeps its path and comes back empty. Dropping it would
            // shift every later branch, which is the same pairing bug in another form.
            var lookups = new List<double>(branch.Count);
            foreach (GH_Number? item in branch)
            {
                if (item != null)
                    lookups.Add(item.Value);
            }

            if (lookups.Count == 0)
            {
                output.EnsurePath(path);
                continue;
            }

            double totalLength = lookups[lookups.Count - 1];
            if (high > totalLength)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                    $"High ({high:G6}) is longer than branch {path} ({totalLength:G6}), so it comes back as just its two ends.");

            // Out-of-range indices would read past the lookup list, so they are dropped
            // and named rather than failing the whole solve -- the same trade the sort
            // components make for null geometry.
            var kept = new List<int>();
            var skipped = new List<int>();
            if (cornerBranches > 0)
            {
                IList<GH_Integer> cornerBranch = cornerTree!.Branches[b < cornerBranches ? b : cornerBranches - 1];
                for (int k = 0; k < cornerBranch.Count; k++)
                {
                    GH_Integer? c = cornerBranch[k];
                    if (c == null || c.Value < 0 || c.Value >= lookups.Count)
                        skipped.Add(k);
                    else
                        kept.Add(c.Value);
                }
            }
            if (skipped.Count > 0)
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    Messages.Skipped(skipped, $"out-of-range corner index in branch {path}"));

            // The ABI wants corner arc lengths ascending, and nothing upstream guarantees
            // the indices arrive that way -- Grasshopper hands over whatever order the wire
            // carries. (geometry_utils.redistribute_lookups_native does not sort either; it
            // has simply never been fed an unsorted list.)
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

            var wrapped = new List<GH_Number>(result.Length);
            foreach (double v in result)
                wrapped.Add(new GH_Number(v));
            output.AppendRange(wrapped, path);
        }

        da.SetDataTree(0, output);
    }
}
