// sort_curves.cpp
// Greedy k-NN (kd-tree) + 2-opt ordering of curve endpoints. Called from
// Python via ctypes -- only endpoint coordinates cross the boundary, never geometry.

#include "nanoflann.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

// ---------------------------------------------------------------------------
// Step 2: data source that nanoflann queries to build the kd-tree.
// It does NOT own or copy the points; it just borrows the endpoints buffer.
// Point layout: 2*n points total, each point = 3 doubles.
//   point pt -> endpoints[pt*3 + 0..2]
//   pt even  -> a curve's start; pt odd -> that curve's end
//   curve index = pt / 2
// ---------------------------------------------------------------------------
struct EndpointCloud {
    const double* pts;  // borrowed pointer to the flat endpoints buffer
    int           num;  // number of points = 2 * n

    // Q1: how many points do you have?
    inline size_t kdtree_get_point_count() const {
        return num;
    }

    // Q2: give me coordinate `dim` (0,1,2) of point `idx`.
    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx * 3 + dim];
    }

    // Q3: bounding box hint. Return false to let nanoflann compute it.
    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
        return false;
    }
};

// Convenience: the concrete kd-tree type over our 3D endpoint cloud.
// L2_Simple_Adaptor = squared Euclidean distance; last arg 3 = dimensions.
typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, EndpointCloud>,
    EndpointCloud,
    3
> KDTree;

// ---------------------------------------------------------------------------
// Small internal helpers (used by greedy + 2-opt). Not exported.
// ---------------------------------------------------------------------------

// Euclidean distance between two endpoint points, given their point indices.
static double dist_pts(const double* ep, int a, int b) {
    double dx = ep[a * 3 + 0] - ep[b * 3 + 0];
    double dy = ep[a * 3 + 1] - ep[b * 3 + 1];
    double dz = ep[a * 3 + 2] - ep[b * 3 + 2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Point index of a curve's entry / exit, given its reversal flag.
//   rev == 0: enter at start (even), exit at end (odd)
//   rev == 1: enter at end   (odd),  exit at start (even)
static inline int entry_point(int curve, int rev) { return rev ? curve * 2 + 1 : curve * 2;     }
static inline int exit_point (int curve, int rev) { return rev ? curve * 2     : curve * 2 + 1; }


// Export macro: makes this function callable from outside the shared library
// (MSVC's __declspec(dllexport) on Windows, GCC/Clang's visibility attribute elsewhere).
#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

// extern "C" disables C++ name mangling so ctypes can find the symbol by name.
extern "C" {

// The single entry point GH/ctypes will call.
//
// Inputs (read-only):
//   endpoints : flat array, 6 doubles per curve -> sx,sy,sz, ex,ey,ez
//   n         : number of curves
//   start_pt  : 3 doubles -> reference point to pick the first curve
//   use_two_opt        : 0/1 flag
//   two_opt_max_passes : cap on 2-opt passes
//   knn_k     : how many nearest neighbors to query per greedy step
//   if_flip   : 1 = curves may be reversed to shorten travel (default);
//               0 = direction is fixed, connect head->tail only. In this
//               mode reversal is always 0, and 2-opt is skipped because
//               reversing a sub-sequence would flip curve directions.
//
// Outputs (caller pre-allocates, we fill):
//   out_order         : n ints -> original curve indices in sorted order
//   out_reversal      : n ints -> 0/1, whether each curve is reversed
//   out_travel_points : n*6 doubles -> n travel segments, 6 values each
//                       (start_x,y,z, end_x,y,z). Segment 0 is start_pt to
//                       the first curve's entry; segment k (1..n-1) is
//                       curve[k-1]'s exit to curve[k]'s entry. Computed as a
//                       post-pass after out_order/out_reversal are final --
//                       doesn't affect greedy/2-opt.
DLL_EXPORT void sort_curves(
    const double* endpoints,
    int           n,
    const double* start_pt,
    int           use_two_opt,
    int           two_opt_max_passes,
    int           knn_k,
    int           if_flip,
    int*          out_order,
    int*          out_reversal,
    double*       out_travel_points)
{
    if (n <= 0) {
        return;
    }

    // --- Build the kd-tree over all 2*n endpoints ---
    EndpointCloud cloud;
    cloud.pts = endpoints;
    cloud.num = 2 * n;

    KDTree tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // Tracks which curves are already placed in the chain.
    std::vector<bool> used(n, false);

    // The current exit point of the path (where the next curve should connect).
    double cur[3];

    // --- Pick the first curve: nearest entry point to start_pt ---
    // When if_flip == 0 we may only enter at a head (even point), so we search
    // a growing neighborhood and skip tails until a head is found.
    {
        double q[3] = { start_pt[0], start_pt[1], start_pt[2] };

        int first_curve = -1;
        int first_rev   = 0;
        int sc = knn_k;

        while (first_curve == -1) {
            if (sc > 2 * n) {
                sc = 2 * n;
            }

            std::vector<uint32_t> idxs(sc);
            std::vector<double>   dists(sc);
            size_t got = tree.knnSearch(&q[0], sc, idxs.data(), dists.data());

            for (size_t r = 0; r < got; ++r) {
                int pt = (int)idxs[r];
                if (!if_flip && (pt % 2 == 1)) {
                    continue;  // direction fixed: heads (even) only
                }
                first_curve = pt / 2;
                first_rev   = (pt % 2 == 1) ? 1 : 0;
                break;
            }

            if (first_curve == -1) {
                if (sc == 2 * n) {
                    break;
                }
                sc *= 2;
            }
        }

        if (first_curve == -1) {  // safety fallback
            first_curve = 0;
            first_rev   = 0;
        }

        out_order[0]    = first_curve;
        out_reversal[0] = first_rev;
        used[first_curve] = true;

        // Exit point = the OTHER end of this curve.
        int ep = exit_point(first_curve, first_rev);
        cur[0] = endpoints[ep * 3 + 0];
        cur[1] = endpoints[ep * 3 + 1];
        cur[2] = endpoints[ep * 3 + 2];
    }

    // --- Greedy: fill positions 1 .. n-1 ---
    for (int placed = 1; placed < n; ++placed) {
        int best_curve = -1;
        int best_rev   = 0;

        // Query k neighbors; if all are used, grow the search and retry.
        int sc = knn_k;
        while (best_curve == -1) {
            if (sc > 2 * n) {
                sc = 2 * n;
            }

            std::vector<uint32_t> idxs(sc);
            std::vector<double>   dists(sc);
            size_t got = tree.knnSearch(&cur[0], sc, idxs.data(), dists.data());

            for (size_t r = 0; r < got; ++r) {
                int pt    = (int)idxs[r];
                if (!if_flip && (pt % 2 == 1)) {
                    continue;  // direction fixed: heads (even) only
                }
                int curve = pt / 2;
                if (used[curve]) {
                    continue;
                }
                best_curve = curve;
                best_rev   = (pt % 2 == 1) ? 1 : 0;
                break;
            }

            if (best_curve == -1) {
                if (sc == 2 * n) {
                    break;  // nothing unused left (safety; shouldn't happen)
                }
                sc *= 2;
            }
        }

        if (best_curve == -1) {
            break;  // safety break
        }

        out_order[placed]    = best_curve;
        out_reversal[placed] = best_rev;
        used[best_curve]     = true;

        int ep = exit_point(best_curve, best_rev);
        cur[0] = endpoints[ep * 3 + 0];
        cur[1] = endpoints[ep * 3 + 1];
        cur[2] = endpoints[ep * 3 + 2];
    }

    // --- Step 4: 2-opt post-processing ---
    // Neighbour-pruned, not exhaustive. A 2-opt move removes the two travel
    // gaps after positions i and j and reconnects them:
    //
    //     remove: exit_i -> entry_i+1   (length gap_i)
    //             exit_j -> entry_j+1   (length gap_j)
    //     add:    exit_i -> exit_j
    //             entry_i+1 -> entry_j+1
    //
    // It pays only when the new pair is shorter than the old pair, and that
    // cannot happen unless at least one *new* edge is shorter than the old
    // edge it is measured against -- if both new edges were longer, so would
    // be their sum. So every improving move satisfies
    //
    //     d(exit_i, exit_j) < gap_i     OR     d(entry_i+1, entry_j+1) < gap_j
    //
    // and each half is a ball query the existing endpoint kd-tree can answer:
    // scan A anchors on exit_i with radius gap_i, scan B anchors on entry_j+1
    // with radius gap_j. The two radii belong to different ends of the move,
    // which is why both scans are needed -- neither alone is complete.
    //
    // This discards no improving move, so the result is still a true 2-opt
    // local optimum. It is NOT the same tour the old exhaustive loop produced:
    // both take the first improving move they meet, and the kd-tree meets them
    // in a different order, so the two walk to different local optima of
    // comparable quality. Do not expect a recorded travel figure to reproduce
    // to the digit across this change.
    //
    // The radius is derived from the tour itself rather than configured, which
    // is the difference from the windowed 2-opt this replaces: that one kept a
    // fixed WINDOW_K = 500 candidates whose coverage fell from 25% to 1% as n
    // grew, and lost 6-13% of tour quality for it. Here the search shrinks only
    // because the gaps do -- and shorter gaps are the goal, not a compromise.
    // See CLAUDE.md before reviving anything from
    // archive/sort_curves_v2_with_windowed_2opt.cpp.
    //
    // Skipped entirely when if_flip == 0: reversing a sub-sequence flips
    // curve directions, which the direction-fixed mode forbids.
    if (use_two_opt && if_flip && n > 3) {
        // Where each curve currently sits in the tour, so an endpoint the
        // kd-tree hands back can be turned into a tour position. Kept in step
        // with every reversal below.
        std::vector<int> pos(n);
        for (int k = 0; k < n; ++k) {
            pos[out_order[k]] = k;
        }

        std::vector<nanoflann::ResultItem<uint32_t, double>> hits;
        nanoflann::SearchParameters search_params;  // sorted: nearest candidates first

        // Applies the (i, j) move if it shortens the tour. Same arithmetic and
        // same 1e-6 acceptance margin as the exhaustive version it replaces.
        auto try_move = [&](int i, int j) -> bool {
            int exit_i   = exit_point (out_order[i],     out_reversal[i]);
            int entry_i1 = entry_point(out_order[i + 1], out_reversal[i + 1]);
            int exit_j   = exit_point (out_order[j],     out_reversal[j]);

            double cost_before = dist_pts(endpoints, exit_i, entry_i1);
            double cost_after  = dist_pts(endpoints, exit_i, exit_j);

            if (j + 1 < n) {
                int entry_j1 = entry_point(out_order[j + 1], out_reversal[j + 1]);
                cost_before += dist_pts(endpoints, exit_j,   entry_j1);
                cost_after  += dist_pts(endpoints, entry_i1, entry_j1);
            }

            if (cost_after >= cost_before - 1e-6) {
                return false;
            }

            int lo = i + 1;
            int hi = j;
            while (lo < hi) {
                std::swap(out_order[lo], out_order[hi]);
                int neg_lo = out_reversal[lo] ? 0 : 1;
                int neg_hi = out_reversal[hi] ? 0 : 1;
                out_reversal[lo] = neg_hi;
                out_reversal[hi] = neg_lo;
                pos[out_order[lo]] = lo;
                pos[out_order[hi]] = hi;
                ++lo;
                --hi;
            }
            if (lo == hi) {
                out_reversal[lo] = out_reversal[lo] ? 0 : 1;
            }
            return true;
        };

        bool improved = true;
        int  passes   = 0;

        while (improved && passes < two_opt_max_passes) {
            improved = false;
            ++passes;

            for (int k = 0; k + 1 < n; ++k) {
                // The gap after position k, which is both scans' radius --
                // recomputed each time because a move may have just changed it.
                int exit_k   = exit_point (out_order[k],     out_reversal[k]);
                int entry_k1 = entry_point(out_order[k + 1], out_reversal[k + 1]);
                double gap   = dist_pts(endpoints, exit_k, entry_k1);

                // Scan A: k is the move's i, so look for a j whose exit lies
                // within gap of exit_k. radiusSearch works in squared distance.
                hits.clear();
                (void)tree.radiusSearch(&endpoints[exit_k * 3], gap * gap, hits, search_params);

                for (size_t h = 0; h < hits.size(); ++h) {
                    int p = (int)hits[h].first;
                    int j = pos[p / 2];
                    // j must sit far enough along to leave a segment to reverse,
                    // and p must be that curve's exit under its current reversal
                    // (the other endpoint is its entry, a different move).
                    if (j < k + 2) {
                        continue;
                    }
                    if (exit_point(out_order[j], out_reversal[j]) != p) {
                        continue;
                    }
                    if (try_move(k, j)) {
                        improved = true;
                        break;  // tour changed under us; carry on at the next k
                    }
                }

                // Scan B: k is the move's j, so look for an i+1 whose entry lies
                // within gap of entry_k1. Needs k >= 2 to leave room for i >= 0.
                if (k >= 2) {
                    exit_k   = exit_point (out_order[k],     out_reversal[k]);
                    entry_k1 = entry_point(out_order[k + 1], out_reversal[k + 1]);
                    gap      = dist_pts(endpoints, exit_k, entry_k1);

                    hits.clear();
                    (void)tree.radiusSearch(&endpoints[entry_k1 * 3], gap * gap, hits, search_params);

                    for (size_t h = 0; h < hits.size(); ++h) {
                        int q = (int)hits[h].first;
                        int m = pos[q / 2];  // the candidate is position i+1
                        if (m < 1 || m > k - 1) {
                            continue;
                        }
                        if (entry_point(out_order[m], out_reversal[m]) != q) {
                            continue;
                        }
                        if (try_move(m - 1, k)) {
                            improved = true;
                            break;
                        }
                    }
                }
            }
        }
    }

    // --- Step 5: travel-segment start/end points (post-pass, doesn't touch
    // out_order/out_reversal) ---
    // Segment 0: start_pt -> first curve's entry.
    {
        int ep0 = entry_point(out_order[0], out_reversal[0]);
        out_travel_points[0] = start_pt[0];
        out_travel_points[1] = start_pt[1];
        out_travel_points[2] = start_pt[2];
        out_travel_points[3] = endpoints[ep0 * 3 + 0];
        out_travel_points[4] = endpoints[ep0 * 3 + 1];
        out_travel_points[5] = endpoints[ep0 * 3 + 2];
    }

    // Segments 1..n-1: curve[k-1]'s exit -> curve[k]'s entry. (Plugging
    // k = n-1 into this same formula gives exactly the "last segment" case,
    // so no separate boundary handling is needed.)
    for (int k = 1; k < n; ++k) {
        int exit_k1 = exit_point (out_order[k - 1], out_reversal[k - 1]);
        int entry_k = entry_point(out_order[k],     out_reversal[k]);

        out_travel_points[k * 6 + 0] = endpoints[exit_k1 * 3 + 0];
        out_travel_points[k * 6 + 1] = endpoints[exit_k1 * 3 + 1];
        out_travel_points[k * 6 + 2] = endpoints[exit_k1 * 3 + 2];
        out_travel_points[k * 6 + 3] = endpoints[entry_k * 3 + 0];
        out_travel_points[k * 6 + 4] = endpoints[entry_k * 3 + 1];
        out_travel_points[k * 6 + 5] = endpoints[entry_k * 3 + 2];
    }
}

} // extern "C"
