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
    /// The cost sentence is the caller's to supply. The two components no longer
    /// share a 2-opt implementation, so a single hardcoded warning was true for
    /// one of them and false for the other -- it told Sort Curves users to expect
    /// minutes for work that now finishes in well under a second.
    /// </summary>
    public static string AboveTestedLimit(int n, string items, int limit, string cost)
        => $"{n:N0} {items} is above the largest measured input ({limit:N0}). {cost}";

    /// <summary>For the exhaustive O(n²) pass, which is what sort_points still runs.</summary>
    public const string ExhaustiveCost =
        "The 2-opt pass is O(n²) from here and has no cheaper fallback, so this can take "
        + "tens of seconds to minutes.";

    /// <summary>For the pruned search: fast where it has been measured, simply unmeasured above.</summary>
    public const string PrunedCost =
        "The 2-opt pass only tests pairs that could shorten the tour, so it stays quick at every "
        + "size measured -- 2.6 s at 50,000 curves. Past that it is untested rather than known to be slow.";

    public const string LoadFailedPrefix = "geomseq_core native call failed: ";
}
