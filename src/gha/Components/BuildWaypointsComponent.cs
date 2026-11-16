using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// The travel move between each pair of consecutive curves: end of one, start of the
/// next. Direction is taken as given -- reversing a curve is the caller's business.
/// </summary>
public sealed class BuildWaypointsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("c4a7e206-5d19-4f83-8e2b-7a61d9f3c054");

    public BuildWaypointsComponent()
        : base("Build Waypoints", "Wp",
               "Travel moves between consecutive curves: the end of each and the start of the next, " +
               "one branch per move. No smoothing -- Build Turn Waypoints fillets the same moves. " +
               "The curves are used in the order and direction given.",
               "GeomSeq", "Travel")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.BuildWaypoints;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C",
            "Curves in travel order. A move is built between each consecutive pair.",
            GH_ParamAccess.list);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Waypoints", "W",
            "Two points per move: the end of one curve, then the start of the next. "
            + "Branch k is the move leaving curve k.",
            GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var curves = new List<Curve>();
        if (!da.GetDataList(0, curves) || curves.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to join.");
            return;
        }

        if (curves.Count < 2)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "A move needs two curves; only one arrived.");
            return;
        }

        var output = new GH_Structure<GH_Point>();
        var skipped = new List<int>();

        for (int i = 0; i < curves.Count - 1; i++)
        {
            // Pathed by the curve the move leaves, so a skipped one leaves a gap
            // rather than shifting later branches.
            if (curves[i] == null || !curves[i].IsValid ||
                curves[i + 1] == null || !curves[i + 1].IsValid)
            {
                skipped.Add(i);
                continue;
            }

            var path = new GH_Path(i);
            output.EnsurePath(path);
            output.Append(new GH_Point(curves[i].PointAtEnd), path);
            output.Append(new GH_Point(curves[i + 1].PointAtStart), path);
        }

        if (skipped.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                Messages.Skipped(skipped, "move whose curve is null or invalid", "built"));

        da.SetDataTree(0, output);
    }
}
