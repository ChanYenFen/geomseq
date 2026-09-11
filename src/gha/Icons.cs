using System.Drawing;
using System.IO;

namespace GeomSeq;

/// <summary>24x24 icons embedded from Resources/, decoded once.</summary>
internal static class Icons
{
    public static readonly Bitmap Library     = Load("geomseq.png");
    public static readonly Bitmap SortCurves  = Load("sort_curves.png");
    public static readonly Bitmap SortPoints  = Load("sort_points.png");

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
