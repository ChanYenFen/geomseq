using System;
using System.Collections.Generic;
using System.Drawing;
using GeomSeq.Native;
using Grasshopper.Kernel;
using Grasshopper.Kernel.Data;
using Grasshopper.Kernel.Types;
using Rhino.Geometry;

namespace GeomSeq.Components;

public sealed class ShatterCrossingsComponent : GH_Component
{
    // Contract: never change. Saved .gh files find this component by it.
    private static readonly Guid Id = new("3805baf9-7e11-4aff-b8bb-2a1c63881800");

    // Pieces are rebuilt from the vertices they came from, so an uncut segment
    // reproduces its ends exactly and a shared vertex is bit-identical. This only
    // has to absorb the plane round-trip, which both sides of a joint go through
    // together -- it is not a modelling tolerance, and Tolerance is not it.
    private const double JoinTol = 1e-9;

    public ShatterCrossingsComponent()
        : base("Shatter Crossings", "ShatterX",
               "Cuts polylines where they meet and opens a gap there, so crossing paths no longer touch. " +
               "Covers a T-junction as well as a crossing. For a meeting pair the earlier input is left whole " +
               "and the later one yields, so reorder the input to choose which path survives intact.",
               "GeomSeq", "Division")
    {
    }

    public override Guid ComponentGuid => Id;
    public override GH_Exposure Exposure => GH_Exposure.primary;
    protected override Bitmap Icon => Icons.ShatterCrossings;

    // Contract: Grasshopper saves wires by port index. New ports go at the end, never in between.
    //
    // Curves is a tree rather than a list for the same reason Redistribute Lookups takes
    // one: under list access Grasshopper solves once per branch and appends the iteration
    // index to every output path, which would land the results one level deeper than the
    // input index this component exists to preserve.
    protected override void RegisterInputParams(GH_InputParamManager p)
    {
        p.AddCurveParameter("Curves", "C",
            "Polylines to shatter. Each branch is handled on its own, so curves in different branches never cut each other.",
            GH_ParamAccess.tree);
        p.AddNumberParameter("Distance", "D",
            "The whole gap: how far apart the two cut ends end up. Half of it comes off either side of each contact.",
            GH_ParamAccess.item, 0.0);
        p.AddNumberParameter("Tolerance", "T",
            "How close counts as touching rather than missing, in model units. A T-junction is rarely drawn exact, " +
            "so the document tolerance is usually right. 0 means exact only.",
            GH_ParamAccess.item, 0.0);
        p.AddBooleanParameter("Self", "X",
            "Also cut a polyline where it crosses itself. Its own corners are never cut either way.",
            GH_ParamAccess.item, false);
        p.AddPlaneParameter("Plane", "P",
            "Plane the contacts are measured in. Height above it rides along, so a cut point keeps the height it would have had.",
            GH_ParamAccess.item, Plane.WorldXY);

        // Optional so empty input reaches SolveInstance and gets a Remark rather than
        // Grasshopper's own missing-input warning.
        p[0].Optional = true;
    }

    protected override void RegisterOutputParams(GH_OutputParamManager p)
    {
        p.AddCurveParameter("Segments", "S",
            "Surviving pieces, on the input path plus the index of the curve they came from. " +
            "A curve the gaps swallowed whole keeps its path and holds nothing, so later indices do not shift.",
            GH_ParamAccess.tree);
        p.AddIntegerParameter("Count", "N",
            "How many pieces each input curve produced, on the path it came in on.",
            GH_ParamAccess.tree);
    }

    protected override void SolveInstance(IGH_DataAccess da)
    {
        if (!NativeLibraryLoader.TryEnsureLoaded(out string loadError))
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, loadError);
            return;
        }

        if (!da.GetDataTree(0, out GH_Structure<GH_Curve> curveTree) || curveTree.IsEmpty)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark, "No curves to shatter.");
            return;
        }

        double gapD = 0.0, touchTol = 0.0;
        bool testSelf = false;
        var plane = Plane.WorldXY;
        da.GetData(1, ref gapD);
        da.GetData(2, ref touchTol);
        da.GetData(3, ref testSelf);
        da.GetData(4, ref plane);

        if (gapD < 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Distance cannot be negative.");
            return;
        }

        if (touchTol < 0.0)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Tolerance cannot be negative.");
            return;
        }

        if (!plane.IsValid)
        {
            AddRuntimeMessage(GH_RuntimeMessageLevel.Error, "Plane is not valid.");
            return;
        }

        if (gapD == 0.0)
            AddRuntimeMessage(GH_RuntimeMessageLevel.Remark,
                "Distance is 0, so curves are cut at their contacts but no gap is opened.");

        var segmentsOut = new GH_Structure<GH_Curve>();
        var countsOut = new GH_Structure<GH_Integer>();

        for (int b = 0; b < curveTree.PathCount; b++)
        {
            GH_Path path = curveTree.Paths[b];
            IList<GH_Curve> branch = curveTree.Branches[b];

            // Refusing rather than approximating: a curve that is not a polyline has no
            // straight segments to meet, and reducing it to the line between its ends
            // would move every contact with nothing in the output to say so.
            var polylines = new List<Polyline>(branch.Count);
            var rejected = new List<int>();
            for (int i = 0; i < branch.Count; i++)
            {
                Curve? c = branch[i]?.Value;
                if (c == null || !c.TryGetPolyline(out Polyline pl) || pl.Count < 2)
                {
                    rejected.Add(i);
                    polylines.Add(new Polyline());
                }
                else
                {
                    polylines.Add(pl);
                }
            }

            if (rejected.Count > 0)
            {
                // Messages.Skipped is not reusable here: it ends "The rest were sorted",
                // and nothing is sorted or partially kept -- the branch is refused whole.
                string listed = string.Join(", ", rejected.Count > 20 ? rejected.GetRange(0, 20) : rejected);
                if (rejected.Count > 20)
                    listed += $", … ({rejected.Count - 20} more)";
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error,
                    $"Branch {path}: {rejected.Count} of {branch.Count} curves are not polylines (index {listed}). " +
                    "Shatter Crossings handles polylines only, so nothing in this branch was computed.");
                segmentsOut.EnsurePath(path);
                countsOut.EnsurePath(path);
                continue;
            }

            // Explode into plane space. The contact test is 2D; working in plane
            // coordinates puts it on the plane the caller chose and leaves the height
            // above it as z, which the native side interpolates down each segment.
            int segCount = 0;
            foreach (Polyline pl in polylines)
                segCount += pl.Count - 1;

            if (segCount == 0)
            {
                segmentsOut.EnsurePath(path);
                countsOut.EnsurePath(path);
                continue;
            }

            var buffer = new double[segCount * 6];
            var owners = new int[segCount];
            var spans = new (int Start, int Stop)[polylines.Count];

            int s = 0;
            for (int owner = 0; owner < polylines.Count; owner++)
            {
                Polyline pl = polylines[owner];
                int start = s;
                for (int k = 0; k < pl.Count - 1; k++)
                {
                    Point3d a = ToPlane(plane, pl[k]);
                    Point3d z = ToPlane(plane, pl[k + 1]);
                    int o = s * 6;
                    buffer[o] = a.X; buffer[o + 1] = a.Y; buffer[o + 2] = a.Z;
                    buffer[o + 3] = z.X; buffer[o + 4] = z.Y; buffer[o + 5] = z.Z;
                    owners[s] = owner;
                    s++;
                }
                spans[owner] = (start, s);
            }

            GeomSeqCore.ShatterResult result;
            try
            {
                result = GeomSeqCore.ShatterAtCrossings(buffer, segCount, gapD, touchTol, owners, testSelf);
            }
            catch (Exception e) when (e is DllNotFoundException or EntryPointNotFoundException or BadImageFormatException)
            {
                AddRuntimeMessage(GH_RuntimeMessageLevel.Error, Messages.LoadFailedPrefix + e.Message);
                return;
            }

            // Where each input segment's pieces begin in the flat result.
            var offsets = new int[segCount + 1];
            for (int k = 0; k < segCount; k++)
                offsets[k + 1] = offsets[k] + result.PieceCounts[k];

            for (int owner = 0; owner < polylines.Count; owner++)
            {
                (int start, int stop) = spans[owner];
                GH_Path outPath = path.AppendElement(owner);

                // Pieces of one polyline, in order, stay contiguous wherever nothing was
                // cut away, so a run of pieces that touch end-to-start is one surviving
                // polyline. A gap inside a segment and a gap at a corner both break a run
                // by the same test, so neither needs a case of its own.
                var rebuilt = new List<GH_Curve>();
                var run = new List<Point3d>();

                for (int k = offsets[start]; k < offsets[stop]; k++)
                {
                    Line piece = result.Piece(k);
                    Point3d from = plane.PointAt(piece.FromX, piece.FromY, piece.FromZ);
                    Point3d to = plane.PointAt(piece.ToX, piece.ToY, piece.ToZ);

                    if (run.Count > 0 && run[run.Count - 1].DistanceTo(from) <= JoinTol)
                    {
                        run.Add(to);
                    }
                    else
                    {
                        if (run.Count > 1)
                            rebuilt.Add(new GH_Curve(new PolylineCurve(run)));
                        run = new List<Point3d> { from, to };
                    }
                }
                if (run.Count > 1)
                    rebuilt.Add(new GH_Curve(new PolylineCurve(run)));

                segmentsOut.AppendRange(rebuilt, outPath);
                countsOut.Append(new GH_Integer(rebuilt.Count), path);

                // An empty path rather than a missing one: dropping it would shift every
                // later index, which is exactly what this component promises not to do.
                if (rebuilt.Count == 0)
                    segmentsOut.EnsurePath(outPath);
            }
        }

        da.SetDataTree(0, segmentsOut);
        da.SetDataTree(1, countsOut);
    }

    /// <summary>World point to plane coordinates; x/y along the plane, z its height above it.</summary>
    private static Point3d ToPlane(Plane plane, Point3d world)
    {
        return plane.RemapToPlaneSpace(world, out Point3d local) ? local : world;
    }
}
