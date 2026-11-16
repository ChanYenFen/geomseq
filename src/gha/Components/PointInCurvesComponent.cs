using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// Classifies points against several closed curves, and says which curve contains each.
/// </summary>
/// <remarks>
/// The plural twin of <see cref="PointInCurveComponent"/>, split the way Grasshopper
/// splits Point In Curve from Point In Curves: this one takes a list of curves and adds
/// an index output. Each point stops at the first curve in the list that contains it,
/// so nested curves answer for whichever containing curve is listed first.
/// </remarks>
public sealed class PointInCurvesComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("6f2c1e4a-8b3d-4e7f-9a15-2c4d7e8f1b30");

    private const double NoDocumentTolerance = 0.001;

    public PointInCurvesComponent()
        : base("Point In Curves", "PtInCrvs",
               "Tests points against several closed curves: outside, on a boundary, or inside, " +
               "and which curve. Measured in one plane, so this is for points already coplanar " +
               "with the curves -- Grasshopper's own Point In Curves stays the general one.",
               "GeomSeq", "Analysis")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.PointInCurves;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "Closed curves. An open curve is an error.",
            GH_ParamAccess.list);
        p.AddPointParameter("Points", "P", "Points to test.", GH_ParamAccess.list);
        p.AddPlaneParameter("Plane", "Pl",
            "Plane the containment is measured in. Points are flattened onto it, so height above it is ignored.",
            GH_ParamAccess.item, Plane.WorldXY);
        p.AddNumberParameter("Tolerance", "T",
            "How close to a boundary counts as on it, in model units. Never narrower than " +
            "Ct when any curve had to be flattened: nearer than that, inside and outside " +
            "cannot be told apart. 0 means exact only, for curves that are already polylines.",
            GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Chord Tolerance", "Ct",
            "How far a curve may be straightened when it is flattened to a polyline. " +
            "0 takes the document's absolute tolerance.",
            GH_ParamAccess.item, 0.0);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddIntegerParameter("Relationship", "R",
            "Per point: 0 outside, 1 on a boundary, 2 inside. Same numbering as Rhino's own component.",
            GH_ParamAccess.list);
        p.AddIntegerParameter("Index", "I",
            "Per point: the index in C of the curve containing it, or -1 for none.",
            GH_ParamAccess.list);
        p.AddPointParameter("Projected", "P'",
            "Each point dropped onto the plane -- where it was actually tested.",
            GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        var curvesIn = new List<Curve>();
        if (!da.GetDataList(0, curvesIn) || curvesIn.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to test against.");
            return;
        }

        // A null or open curve stops the solve: its index would shift every answer
        // after it, and Rhino's own component errors on an open curve too.
        for (int c = 0; c < curvesIn.Count; c++)
        {
            if (curvesIn[c] == null)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, $"Curve {c} is null.");
                return;
            }
            if (!curvesIn[c].IsClosed)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                    $"Curve {c} is not closed, so it has no inside.");
                return;
            }
        }

        // Read as GH_Point, not Point3d: a null item in a Point3d list arrives as the
        // origin, indistinguishable from a real point there.
        var wrapped = new List<GH_Point>();
        if (!da.GetDataList(1, wrapped) || wrapped.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No points to test.");
            return;
        }

        Plane plane = Plane.WorldXY;
        da.GetData(2, ref plane);

        double tol = 0.0;
        da.GetData(3, ref tol);
        if (tol < 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Tolerance cannot be negative.");
            return;
        }

        double chordTol = 0.0;
        da.GetData(4, ref chordTol);
        if (chordTol < 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Chord Tolerance cannot be negative.");
            return;
        }
        if (chordTol == 0.0)
            chordTol = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? NoDocumentTolerance;

        // A skipped point keeps its slot in the output, so the results stay lined up
        // with whatever the points were paired against upstream.
        var points  = new List<Point3d>(wrapped.Count);
        var slotOf  = new List<int>(wrapped.Count);
        var skipped = new List<int>();
        for (int k = 0; k < wrapped.Count; k++)
        {
            GH_Point? g = wrapped[k];
            if (g == null || !g.Value.IsValid)
            {
                skipped.Add(k);
                continue;
            }
            points.Add(g.Value);
            slotOf.Add(k);
        }

        if (skipped.Count > 0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                Messages.Skipped(skipped, "null or invalid point", "classified"));

        if (points.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No valid points to test.");
            return;
        }

        int[] classified;
        int[] which;
        Point3d[] onPlane;
        bool collapsed;
        try
        {
            (classified, which, onPlane, collapsed) =
                GeomSeqCore.PointInCurves(curvesIn, points, plane, tol, chordTol);
        }
        catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
            return;
        }
        catch (Exception e) when (e is ArgumentException or InvalidOperationException)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, e.Message);
            return;
        }

        if (collapsed)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Warning,
                "A curve flattens to almost nothing in this plane, so inside and outside have " +
                "stopped meaning anything for it. Measure in the curves' own plane instead.");

        // Rebuilt against the original indices, with a skipped point left as outside,
        // index -1, and its projection at the origin -- it had no position to project.
        var relationship = new int[wrapped.Count];
        var index        = new int[wrapped.Count];
        var projected    = new Point3d[wrapped.Count];
        for (int k = 0; k < wrapped.Count; k++)
            index[k] = -1;
        for (int k = 0; k < points.Count; k++)
        {
            relationship[slotOf[k]] = classified[k];
            index[slotOf[k]]        = which[k];
            projected[slotOf[k]]    = onPlane[k];
        }

        da.SetDataList(0, relationship);
        da.SetDataList(1, index);
        da.SetDataList(2, projected);
    }
}
