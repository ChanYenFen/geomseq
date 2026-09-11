using System.Collections.Generic;
using System.Linq;

namespace GeomSeq.Components;

internal static class Messages
{
    private const int MaxListed = 20;

    /// <param name="what">Singular noun phrase, e.g. "null or degenerate curve".</param>
    public static string Skipped(IReadOnlyList<int> indices, string what)
    {
        string listed = string.Join(", ", indices.Take(MaxListed));
        if (indices.Count > MaxListed)
            listed += $", … ({indices.Count - MaxListed} more)";
        return $"Skipped {indices.Count} {what}(s) at input index {listed}. The rest were sorted.";
    }

    public static string AboveTestedLimit(int n, string items, int limit)
        => $"{n:N0} {items} is more than the largest tested input ({limit:N0}). Sorting may take tens of seconds.";

    public const string LoadFailedPrefix = "geomseq_core native call failed: ";
}
