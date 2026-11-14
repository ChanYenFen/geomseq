using System.Collections.Generic;
using System.Linq;

namespace GeomSeq.Components;

internal static class Messages
{
    private const int MaxListed = 20;

    /// <param name="what">Singular noun phrase, e.g. "null or degenerate curve".</param>
    /// <param name="rest">
    /// Past participle for what happened to the others, e.g. "sorted", "classified".
    /// </param>
    public static string Skipped(IReadOnlyList<int> indices, string what, string rest = "sorted")
    {
        string listed = string.Join(", ", indices.Take(MaxListed));
        if (indices.Count > MaxListed)
            listed += $", … ({indices.Count - MaxListed} more)";
        return $"Skipped {indices.Count} {what}(s) at input index {listed}. The rest were {rest}.";
    }

    /// <summary>
    /// Text for an input larger than any measured one. Post it as a Remark, not a
    /// Warning: nothing is wrong.
    /// </summary>
    public static string AboveTestedLimit(int n, string items, int limit)
        => $"{n:N0} {items} is above the largest measured input ({limit:N0}); timing beyond it is untested.";

    public const string LoadFailedPrefix = "geomseq_core native call failed: ";
}
