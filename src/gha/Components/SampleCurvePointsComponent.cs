using System;
using System.Collections.Generic;
using System.Drawing;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// The other half of Redistribute Arc Lengths: turns arc-length positions back into points.
/// </summary>
/// <remarks>
/// Nothing here calls the native library. This is RhinoCommon arithmetic -- LengthParameter
/// then PointAt -- reimplemented from rhino_utils/sample_curve_points.py rather than wrapped,
/// because there is no C++ to wrap. That makes it the first piece of behaviour in the plug-in
/// with a second implementation living elsewhere, and pytest cannot reach this one: it needs
/// RhinoCommon. If the two ever disagree, nothing automatic will say so.
/// </remarks>
public sealed class SampleCurvePointsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("5d9255aa-92f0-4273-b534-e2832d12bf30");

    public SampleCurvePointsComponent()
        : base("Sample Curve Points", "SampleCrv",
               "Evaluates each curve at a branch of arc-length positions. " +
               "Pairs with Redistribute Arc Lengths, which decides where those positions go.",
               "GeomSeq", "Division")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.SampleCurvePoints;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    //
    // Curves is a flat list and Arc Lengths is a tree, deliberately: one curve per arc length
    // branch, paired by position. That is the shape the GHPython original consumed
    // (`zip(arc_lengths, curves)`) and the shape Redistribute Arc Lengths emits.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "One curve per branch of Arc Lengths, in the same order.",
            GH_ParamAccess.list);
        p.AddNumberParameter("Arc Lengths", "S",
            "Arc-length positions to evaluate, one branch per curve.", GH_ParamAccess.tree);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Points", "P",
            "Points on each curve, on the same paths as Arc Lengths.", GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        var curves = new List<Curve?>();
        da.GetDataList(0, curves);

        if (!da.GetDataTree(1, out GH_Structure<GH_Number> arcLengthTree) || arcLengthTree.IsEmpty)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No arc lengths to evaluate.");
            return;
        }

        if (curves.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to sample.");
            return;
        }

        if (curves.Count != arcLengthTree.PathCount)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{curves.Count} curve(s) against {arcLengthTree.PathCount} arc length branch(es); " +
                "the last curve is reused for the remainder.");

        var output = new GH_Structure<GH_Point>();
        int pastEnd = 0;

        for (int b = 0; b < arcLengthTree.PathCount; b++)
        {
            GH_Path path = arcLengthTree.Paths[b];
            output.EnsurePath(path);

            Curve? curve = curves[b < curves.Count ? b : curves.Count - 1];
            if (curve == null || !curve.IsValid)
            {
                // The branch keeps its path and stays empty: dropping it would shift
                // every later branch out of step with its curve.
                AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                    $"Branch {path} has no valid curve; it comes back empty.");
                continue;
            }

            foreach (GH_Number? item in arcLengthTree.Branches[b])
            {
                if (item == null)
                {
                    pastEnd++;
                    continue;
                }

                // LengthParameter fails rather than throwing for a position past the
                // curve's end, which is how the Python version silently drops them.
                // Counted here so the count can be reported instead.
                if (curve.LengthParameter(item.Value, out double t))
                    output.Append(new GH_Point(curve.PointAt(t)), path);
                else
                    pastEnd++;
            }
        }

        if (pastEnd > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{pastEnd} position(s) fell outside their curve's length and produced no point.");

        da.SetDataTree(0, output);
    }
}
