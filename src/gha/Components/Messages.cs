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

    /// <summary>
    /// Both components prune their 2-opt now, so neither has a cost story worth
    /// a paragraph: they are quick at every size measured and simply unmeasured
    /// above it. This says that and stops. It is posted as a Remark, not a
    /// Warning -- nothing is wrong, the component has just left the range anyone
    /// has numbers for, and an orange bubble on a correct result teaches people
    /// to ignore the bubbles that matter.
    /// </summary>
    public static string AboveTestedLimit(int n, string items, int limit)
        => $"{n:N0} {items} is above the largest measured input ({limit:N0}); timing beyond it is untested.";

    public const string LoadFailedPrefix = "geomseq_core native call failed: ";
}
