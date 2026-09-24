using System.Drawing;
using System.IO;

namespace GeomSeq;

/// <summary>24x24 icons embedded from Resources/, decoded once.</summary>
internal static class Icons
{
    public static readonly Bitmap Library     = Load("geomseq.png");
    public static readonly Bitmap SortCurves  = Load("sort_curves.png");
    public static readonly Bitmap SortPoints  = Load("sort_points.png");

    // No artwork for these yet: Load returns null and Grasshopper draws its default
    // icon, so the wiring can land before the drawing does. Dropping the PNG into
    // Resources/ later is enough -- the csproj embeds that folder by wildcard.
    public static readonly Bitmap RedistributeArcLengths = Load("redistribute_arc_lengths.png");
    public static readonly Bitmap SampleCurvePoints   = Load("sample_curve_points.png");
    public static readonly Bitmap DivideCurves        = Load("divide_curves.png");
    public static readonly Bitmap BuildTurnWaypoints  = Load("build_turn_waypoints.png");
    public static readonly Bitmap ShatterCrossings    = Load("shatter_crossings.png");

    private static Bitmap Load(string file)
    {
        using Stream? stream = typeof(Icons).Assembly.GetManifestResourceStream("GeomSeq.Resources." + file);
        if (stream == null)
            return null!;  // Grasshopper draws its default icon for null

        // GDI+ may keep reading the source stream for the Bitmap's lifetime; copy so the stream can close.
        using var decoded = new Bitmap(stream);
        return new Bitmap(decoded);
    }
}
