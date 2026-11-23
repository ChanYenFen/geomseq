using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using GeomSeq.Native;
using Grasshopper;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Rhino;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// Cuts a closed region into monotone cells, lays parallel lines across each, and joins
/// each cell's lines into one polyline. No order between cells and no travel: that is
/// <see cref="PlanCoverageComponent"/>.
/// </summary>
public sealed class HatchRegionComponent : GH_Component, IGH_VariableParameterComponent
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("c8113af8-5e07-45c4-a086-736401a71ea3");

    private const double NoDocumentTolerance = 0.001;
    private const string ChordName = "Chord Tolerance";

    // Hidden until added with ⊕, after the seven always shown.
    private static readonly OptionalInputs Extra = new(7,
        () => OptionalInputs.Optional<Grasshopper.Kernel.Parameters.Param_Number>(ChordName, "Ct",
            "How far a curve may be straightened when it is flattened. Absent or 0, the document's tolerance."));

    public HatchRegionComponent()
        : base("Hatch Region", "Hatch",
               "Cuts a closed region into monotone cells, lays parallel lines across each, " +
               "and joins each cell's lines into one polyline.",
               "GeomSeq", "Division")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.tertiary;
    protected override Bitmap Icon => Icons.HatchRegion;

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
        p.AddIntegerParameter("Direction", "D",
            "How each cell's polyline is walked. 0 bottom to top from the left end of the lowest line, " +
            "1 that reversed, 2 bottom to top from the right end, 3 that reversed. Left, right, bottom " +
            "and top are along and across the lines.", GH_ParamAccess.item, 0);
        p.AddPlaneParameter("Plane", "Pl", "Working plane.", GH_ParamAccess.item, Plane.WorldXY);
        // Chord Tolerance follows, hidden until added: see Extra.

        p[0].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddCurveParameter("Cells", "C", "One closed polyline per cell.", GH_ParamAccess.list);
        p.AddCurveParameter("Lines", "L", "The lines, branch i for cell i, bottom to top, each left end to right end.",
            GH_ParamAccess.tree);
        p.AddCurveParameter("Paths", "P", "One polyline per cell: its lines joined per L, walked per D.",
            GH_ParamAccess.list);
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
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to hatch.");
            return;
        }
        double spacing = 0.0, angle = 0.0, phase = 0.0, chordTol = 0.0;
        int link = 0, direction = 0;
        Plane plane = Plane.WorldXY;
        if (!da.GetData(1, ref spacing))
            return;
        da.GetData(2, ref angle);
        da.GetData(3, ref phase);
        da.GetData(4, ref link);
        da.GetData(5, ref direction);
        da.GetData(6, ref plane);
        chordTol = OptionalInputs.Read(this, da, ChordName, 0.0);

        if (direction < 0 || direction > 3)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Direction must be 0, 1, 2 or 3.");
            return;
        }
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
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, PlanCoverageComponent.BadRings(rejected));
            return;
        }

        CoverageCore.Hatch hatch;
        try
        {
            hatch = CoverageCore.HatchRegion(rings, spacing, RhinoMath.ToRadians(angle), phase, link);
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

        var cells = new List<Curve>();
        foreach (Point2d[] outline in hatch.Cells)
        {
            var pts = outline.Select(At).ToList();
            pts.Add(pts[0]);
            cells.Add(new PolylineCurve(pts));
        }

        var lines = new DataTree<Curve>();
        var paths = new List<Curve?>();
        for (int c = 0; c < hatch.Lines.Count; c++)
        {
            var branch = new GH_Path(c);
            lines.EnsurePath(branch);
            foreach ((Point2d a, Point2d b) in hatch.Lines[c])
                lines.Add(new LineCurve(At(a), At(b)), branch);
            Point2d[] walk = hatch.Walks[c][direction];
            paths.Add(walk.Length > 1 ? new PolylineCurve(walk.Select(At)) : null);
        }

        da.SetDataList(0, cells);
        da.SetDataTree(1, lines);
        da.SetDataList(2, paths);
        da.SetData(3, OptionalInputs.Frame(plane, RhinoMath.ToRadians(angle), phase));
    }
}
