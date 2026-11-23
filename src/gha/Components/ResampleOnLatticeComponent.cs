using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Rhino.Geometry;

namespace GeomSeq.Components;

/// <summary>
/// Resamples a path, given as Plan Coverage's pieces and kinds, onto a staggered lattice.
/// Fill pieces follow the lattice; travel pieces are split evenly by length.
/// </summary>
public sealed class ResampleOnLatticeComponent : GH_Component, IGH_VariableParameterComponent
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("c30bf690-7bcf-4ae3-8d9f-e65071bef49b");

    private const int Travel = 1;
    private const string StaggerName = "Stagger";

    // Hidden until added with ⊕, after the seven always shown.
    private static readonly OptionalInputs Extra = new(7,
        () => OptionalInputs.Optional<Grasshopper.Kernel.Parameters.Param_Number>(StaggerName, "Sg",
            "Rows before the lattice repeats; each row shifts by Step / Stagger. 4 when absent."));

    public ResampleOnLatticeComponent()
        : base("Resample On Lattice", "ResampleLat",
               "Resamples Plan Coverage's path onto a staggered lattice: rows take the lattice's " +
               "points, joins between rows its columns, travels are split evenly by length.",
               "GeomSeq", "Division")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.secondary;
    protected override Bitmap Icon => Icons.ResampleOnLattice;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Pieces", "Pc", "Plan Coverage's pieces, in order.", GH_ParamAccess.list);
        p.AddIntegerParameter("Kinds", "K", "Plan Coverage's kinds, one per piece: 0 fill (lattice), 1 travel (even).",
            GH_ParamAccess.list);
        p.AddPlaneParameter("Frame", "Fr",
            "Plan Coverage's frame: x along the lines, origin on the line grid.", GH_ParamAccess.item, Plane.WorldXY);
        p.AddNumberParameter("Spacing", "S", "Line spacing, as Plan Coverage's. Greater than 0.", GH_ParamAccess.item);
        p.AddNumberParameter("Step", "St", "Point distance along a row. Greater than 0.", GH_ParamAccess.item);
        p.AddNumberParameter("Free Step", "F", "Longest step along a travel. 0 takes Step.", GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Min Step", "M",
            "Shortest step kept; a closer point is dropped. 0 takes Spacing (drops a half-spacing climb, " +
            "keeps a zig-zag turn); negative keeps every point.", GH_ParamAccess.item, 0.0);
        // Stagger follows, hidden until added: see Extra.

        p[0].Optional = true;
        p[1].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddPointParameter("Points", "P", "The resampled points, in order.", GH_ParamAccess.list);
        p.AddCurveParameter("Polyline", "L", "One polyline through them.", GH_ParamAccess.item);
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
        var kinds = new List<int>();
        if (!da.GetDataList(0, curves) || curves.Count == 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No pieces to resample.");
            return;
        }
        da.GetDataList(1, kinds);
        if (kinds.Count != curves.Count)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                $"{curves.Count} pieces but {kinds.Count} kinds: connect Plan Coverage's Pieces and Kinds together.");
            return;
        }

        // The lattice is measured in the frame: lines along its x axis, line k at y = k * spacing.
        Plane plane = Plane.WorldXY;
        double spacing = 0.0, step = 0.0, freeStep = 0.0, minStep = 0.0;
        da.GetData(2, ref plane);
        if (!da.GetData(3, ref spacing) || !da.GetData(4, ref step))
            return;
        da.GetData(5, ref freeStep);
        da.GetData(6, ref minStep);
        double stagger = OptionalInputs.Read(this, da, StaggerName, 4.0);

        var runs = new List<(Point2d[] points, bool free)>(curves.Count);
        var bad = new List<int>();
        for (int i = 0; i < curves.Count; i++)
        {
            Curve? c = curves[i];
            if (c == null || !c.TryGetPolyline(out Polyline pl))
            {
                bad.Add(i);
                continue;
            }
            var pts = new Point2d[pl.Count];
            for (int k = 0; k < pl.Count; k++)
            {
                Point3d q = plane.RemapToPlaneSpace(pl[k], out Point3d local) ? local : pl[k];
                pts[k] = new Point2d(q.X, q.Y);
            }
            runs.Add((pts, kinds[i] == Travel));
        }
        if (bad.Count > 0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                $"Piece(s) at input index {string.Join(", ", bad.Take(20))} are missing or not polylines; " +
                "nothing was resampled.");
            return;
        }

        Point2d[] resampled;
        try
        {
            resampled = CoverageCore.ResampleOnLattice(
                runs, spacing, step, 0.0, 0.0, stagger,
                freeStep == 0.0 ? step : freeStep, false, minStep == 0.0 ? spacing : Math.Max(minStep, 0.0));
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

        var points = resampled.Select(q => plane.PointAt(q.X, q.Y)).ToList();
        da.SetDataList(0, points);
        da.SetData(1, points.Count > 1 ? new PolylineCurve(points) : null);
    }
}
