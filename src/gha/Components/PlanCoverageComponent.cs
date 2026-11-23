using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Types;
using Rhino;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// One continuous path over a closed region: the cells in order, each walked one of four
/// ways, joined by travels. No trims: every travel is kept, and the order is chosen to leave
/// as little of them uncovered as possible, then to travel least.
/// </summary>
public sealed class PlanCoverageComponent : GH_Component, IGH_VariableParameterComponent
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("5d1fb514-ae25-4c61-854f-65f643243b35");

    private const double NoDocumentTolerance = 0.001;
    private const string KicksName = "Kicks", ChordName = "Chord Tolerance";

    // Hidden until added with ⊕, in this order after the seven always shown.
    private static readonly OptionalInputs Extra = new(7,
        () => OptionalInputs.Optional<Grasshopper.Kernel.Parameters.Param_Integer>(KicksName, "K",
            "Restarts of the search from a shaken copy of its best order. 0 when absent. Each costs " +
            "about one more search; 20 found the best order on every small shape measured."),
        () => OptionalInputs.Optional<Grasshopper.Kernel.Parameters.Param_Number>(ChordName, "Ct",
            "How far a curve may be straightened when it is flattened. Absent or 0, the document's tolerance."));

    public PlanCoverageComponent()
        : base("Plan Coverage", "Cover",
               "One continuous path over a closed region: cells in order, each walked one of four " +
               "ways, joined by travels kept under cells still to come wherever possible.",
               "GeomSeq", "Travel")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    protected override Bitmap Icon => Icons.PlanCoverage;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C", "Closed curves: the outer boundary and any holes, one region.",
            GH_ParamAccess.list);
        p.AddNumberParameter("Spacing", "S", "Distance between lines. Greater than 0.", GH_ParamAccess.item);
        p.AddNumberParameter("Angle", "A", "Line direction in degrees, in the plane.", GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Phase", "Ph", "Offset of the line grid, in the plane.", GH_ParamAccess.item, 0.0);
        p.AddIntegerParameter("Link", "L",
            "How a cell's lines are joined. 0 zig-zag: alternate directions, joined along the side. " +
            "1 one-way: one direction, joined across the cell. 2 one-way, plus a line through each " +
            "seam between cells.", GH_ParamAccess.item, 0);
        p.AddPointParameter("Start", "St",
            "Where the path starts. Unconnected, the plan picks the first cell and walk.", GH_ParamAccess.item);
        p.AddPlaneParameter("Plane", "Pl", "Working plane.", GH_ParamAccess.item, Plane.WorldXY);
        // Kicks and Chord Tolerance follow, hidden until added: see Extra.

        p[0].Optional = true;
        p[5].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddCurveParameter("Path", "P", "The whole path, one polyline.", GH_ParamAccess.item);
        p.AddCurveParameter("Pieces", "Pc", "The path cut into fills and travels, in order.", GH_ParamAccess.list);
        p.AddIntegerParameter("Kinds", "K", "One per piece: 0 fill, 1 travel.", GH_ParamAccess.list);
        p.AddIntegerParameter("Order", "O", "Cell indices in the order filled.", GH_ParamAccess.list);
        p.AddIntegerParameter("Directions", "D",
            "One per cell in order: 0 forward, 1 reverse, 2 mirrored, 3 mirrored reverse.", GH_ParamAccess.list);
        p.AddNumberParameter("Visible", "V",
            "Travel length over cells already filled, which nothing later covers. 0 when every travel is hidden.",
            GH_ParamAccess.item);
        p.AddNumberParameter("Travel Length", "T", "Total travel length.", GH_ParamAccess.item);
        p.AddPlaneParameter("Frame", "Fr",
            "The plane turned to the lines, its origin on the line grid: feed it to Resample On Lattice.",
            GH_ParamAccess.item);
    }

    bool IGH_VariableParameterComponent.CanInsertParameter(GH_ParameterSide side, int index) => Extra.CanInsert(this, side, index);
    bool IGH_VariableParameterComponent.CanRemoveParameter(GH_ParameterSide side, int index) => Extra.CanRemove(this, side, index);
    IGH_Param IGH_VariableParameterComponent.CreateParameter(GH_ParameterSide side, int index) => Extra.Create(index);
    bool IGH_VariableParameterComponent.DestroyParameter(GH_ParameterSide side, int index) => true;
    void IGH_VariableParameterComponent.VariableParameterMaintenance() { }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        var curves = new List<Curve?>();
        if (!da.GetDataList(0, curves) || curves.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to cover.");
            return;
        }
        double spacing = 0.0, angle = 0.0, phase = 0.0;
        int link = 0;
        Plane plane = Plane.WorldXY;
        if (!da.GetData(1, ref spacing))
            return;
        da.GetData(2, ref angle);
        da.GetData(3, ref phase);
        da.GetData(4, ref link);
        // Read as GH_Point: a missing point must mean "no start", not the origin.
        GH_Point? startIn = null;
        da.GetData(5, ref startIn);
        da.GetData(6, ref plane);
        int kicks = OptionalInputs.Read(this, da, KicksName, 0);
        double chordTol = OptionalInputs.Read(this, da, ChordName, 0.0);

        if (chordTol < 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Chord Tolerance cannot be negative.");
            return;
        }
        if (chordTol == 0.0)
            chordTol = RhinoDoc.ActiveDoc?.ModelAbsoluteTolerance ?? NoDocumentTolerance;

        // One region is one input: a bad ring refuses the whole of it.
        List<Point2d[]>? rings = CoverageCore.Rings(curves, plane, chordTol, out var rejected);
        if (rings == null)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, BadRings(rejected));
            return;
        }

        Point2d? start = null;
        if (startIn != null && startIn.Value.IsValid)
        {
            Point3d s = plane.RemapToPlaneSpace(startIn.Value, out Point3d local) ? local : startIn.Value;
            start = new Point2d(s.X, s.Y);
        }

        CoverageCore.Plan plan;
        try
        {
            plan = CoverageCore.PlanCoverage(rings, spacing, RhinoMath.ToRadians(angle), phase, link, start, kicks);
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

        Point3d At(Point2d q) => plane.PointAt(q.X, q.Y);

        // The pieces end to end, as coverage_path.Plan.path: a piece starts where the last
        // ended, and that point is not repeated.
        var path = new List<Point2d>();
        var pieces = new List<Curve?>();
        var kinds = new List<int>();
        double visible = 0.0, travel = 0.0;
        foreach (CoverageCore.Piece piece in plan.Pieces)
        {
            Point2d[] pts = piece.Points;
            pieces.Add(pts.Length > 1 ? new PolylineCurve(pts.Select(At)) : null);
            kinds.Add(piece.Kind);
            int from = path.Count > 0 && pts.Length > 0 && path[path.Count - 1].DistanceTo(pts[0]) <= 1e-9 ? 1 : 0;
            for (int k = from; k < pts.Length; k++)
                path.Add(pts[k]);
            if (piece.Kind == 1)
            {
                visible += piece.Visible;
                for (int k = 1; k < piece.Points.Length; k++)
                    travel += piece.Points[k - 1].DistanceTo(piece.Points[k]);
            }
        }

        if (visible > 0.0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                $"{visible:G4} of the travel lies over cells already filled.");

        da.SetData(0, path.Count > 1 ? new PolylineCurve(path.Select(At)) : null);
        da.SetDataList(1, pieces);
        da.SetDataList(2, kinds);
        da.SetDataList(3, plan.Order.Select(o => o.cell));
        da.SetDataList(4, plan.Order.Select(o => o.walk));
        da.SetData(5, visible);
        da.SetData(6, travel);
        da.SetData(7, OptionalInputs.Frame(plane, RhinoMath.ToRadians(angle), phase));
    }

    internal static string BadRings(List<(int index, string why)> rejected)
        => "Unusable curve(s), by input index: " +
           string.Join("; ", rejected.Take(20).Select(r => $"{r.index} {r.why}")) +
           (rejected.Count > 20 ? $"; … ({rejected.Count - 20} more)" : "") +
           ". One region is one input, so nothing was computed.";
}
