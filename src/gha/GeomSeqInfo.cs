using System;
using System.Drawing;
using Grasshopper.Kernel;

namespace GeomSeq;

public sealed class GeomSeqInfo : GH_AssemblyInfo
{
    // Never change: Grasshopper identifies the library by this id.
    internal static readonly Guid LibraryId = new("a52a3125-dc70-4b44-9a04-a20bf99b36ec");

    public override Guid Id => LibraryId;
    public override string Name => "GeomSeq";
    public override string Version => typeof(GeomSeqInfo).Assembly.GetName().Version?.ToString(3) ?? "0.0.0";
    public override Bitmap Icon => Icons.Library;
    public override string Description =>
        "Orders curves and points to minimise travel between them (greedy k-NN + 2-opt, native C++ core).";
    public override string AuthorName => "Yen-Fen Chan";
    public override string AuthorContact => "https://github.com/ChanYenFen/geomseq";
}
