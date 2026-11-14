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
/// Classifies points against one closed curve: outside, on the boundary, or inside.
/// </summary>
/// <remarks>
/// The native core is 2D. The curve is discretized to a polyline and both it and the
/// query points are flattened into the Plane input (world XY by default), once for the
/// whole list rather than once per point -- which is where the saving over Rhino's own
/// component comes from. See design-notes.md, "Point in curve, and where it cannot
/// match Rhino's", for where the answers are expected to differ.
/// </remarks>
public sealed class PointInCurveComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("ab5351bd-f74e-43c1-96fd-e201f075eb15");

    // Rhino's own default model tolerance, used only when there is no document to
    // ask -- a test host, or a headless session.
    private const double NoDocumentTolerance = 0.001;

    public PointInCurveComponent()
        : base("Point In Curve", "PtInCrv",
               "Tests points against one closed curve: outside, on the boundary, or inside. " +
               "The curve and the points are measured in one plane, so this is for points already " +
               "coplanar with the curve -- Grasshopper's own Point In Curve stays the general one.",
               "GeomSeq", "Analysis")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.PointInCurve;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curve", "C", "One closed curve. An open curve is an error, as it is for Rhino's own component.",
            GH_ParamAccess.item);
        p.AddPointParameter("Points", "P", "Points to test.", GH_ParamAccess.list);
        p.AddPlaneParameter("Plane", "Pl",
            "Plane the containment is measured in. Points are flattened onto it, so height above it is ignored.",
            GH_ParamAccess.item, Plane.WorldXY);
        p.AddNumberParameter("Tolerance", "T",
            "How close to the boundary counts as on it, in model units. 0 means exact only.",
            GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Chord Tolerance", "Ct",
            "How far the curve may be straightened when it is flattened to a polyline: the largest " +
            "gap allowed between the two. A curve that is already a polyline ignores this -- its " +
            "vertices are exact. 0 takes the document's absolute tolerance.",
            GH_ParamAccess.item, 0.0);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddIntegerParameter("Relationship", "R",
            "Per point: 0 outside, 1 on the boundary, 2 inside. Same numbering as Rhino's own component.",
            GH_ParamAccess.list);
        p.AddPointParameter("Projected", "P'",
            "Each point dropped onto the plane -- where it was actually tested. Same as the P' of " +
            "Rhino's own component. A point already on the plane comes back unchanged, so this " +
            "differing from the input is how you see that a point was off the plane.",
            GH_ParamAccess.list);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        Curve? curve = null;
        if (!da.GetData(0, ref curve) || curve == null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curve to test against.");
            return;
        }

        if (!curve.IsClosed)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                "The curve is not closed, so it has no inside.");
            return;
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
        Point3d[] onPlane;
        bool collapsed;
        try
        {
            (classified, onPlane, collapsed) = GeomSeqCore.PointInCurve(curve, points, plane, tol, chordTol);
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
                "The curve flattens to almost nothing in this plane, so inside and outside have " +
                "stopped meaning anything. Measure in the curve's own plane instead.");

        // Rebuilt against the original indices, with a skipped point left as outside and
        // its projection at the origin -- it had no position to project.
        var relationship = new int[wrapped.Count];
        var projected    = new Point3d[wrapped.Count];
        for (int k = 0; k < points.Count; k++)
        {
            relationship[slotOf[k]] = classified[k];
            projected[slotOf[k]]    = onPlane[k];
        }

        da.SetDataList(0, relationship);
        da.SetDataList(1, projected);
    }
}
