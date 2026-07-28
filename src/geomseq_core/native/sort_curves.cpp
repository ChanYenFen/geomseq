// sort_curves.cpp
// Native curve sorting for the Embroidery pipeline.
// Algorithm: greedy nearest-neighbor (k-NN via kd-tree) + 2-opt.
// Called from Rhino/GH Python via ctypes. Geometry never crosses the
// boundary: only endpoint coordinates in, index permutation out.

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
// Second kd-tree, built only for 2-opt (after greedy finishes -- the
// EndpointCloud/KDTree above are done being used by then). Stores one point
// per *current tour edge* rather than per curve endpoint, so 2-opt can query
// "which edges are spatially near edge i" instead of scanning all O(n) pairs.
//
// n-1 real edges (out_order[k] -> out_order[k+1]) plus one "phantom" point
// for the open path's terminal (out_order[n-1]'s own exit point, not a
// midpoint of two things) -- without it, the j == n-1 "reconnect to the
// tour's end" case the exhaustive loop tests would never be reachable by a
// spatial query, which isn't the same kind of miss as skipping a distant
// real edge. n points total.
// ---------------------------------------------------------------------------
struct EdgeMidpointCloud {
    const double* pts;  // borrowed pointer to a flat buffer, 3 doubles per edge point
    int           num;  // number of edge points = n (n-1 real edges + 1 terminal)

    inline size_t kdtree_get_point_count() const {
        return num;
    }

    inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
        return pts[idx * 3 + dim];
    }

    template <class BBOX>
    bool kdtree_get_bbox(BBOX&) const {
        return false;
    }
};

typedef nanoflann::KDTreeSingleIndexAdaptor<
    nanoflann::L2_Simple_Adaptor<double, EdgeMidpointCloud>,
    EdgeMidpointCloud,
    3
> MidpointKDTree;

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
    // Two paths sharing one function, not two files: real-DLL timing showed
    // the windowed kd-tree approach (below) is only worth its overhead once
    // n is large enough -- below TWO_OPT_WINDOW_THRESHOLD the plain O(n^2)
    // exhaustive loop (same logic as archive/sort_curves_v1_windowed2opt_
    // backup.cpp, copied in-line rather than called out to that file, which
    // isn't part of the build) is both simpler and faster in practice.
    // Threshold picked from measured crossover: at n=8000 windowed was still
    // slightly slower than exhaustive, at n=10000 about even -- 10000 favors
    // exhaustive slightly at the boundary rather than risking the windowed
    // path's quality trade-off where it isn't needed for speed anyway.
    // Skipped entirely when if_flip == 0: reversing a sub-sequence flips
    // curve directions, which the direction-fixed mode forbids.
    if (use_two_opt && if_flip && n > 3) {
        const int TWO_OPT_WINDOW_THRESHOLD = 10000;

        if (n <= TWO_OPT_WINDOW_THRESHOLD) {
            // --- Exhaustive: test every (i, j) pair ---
            bool improved = true;
            int  passes   = 0;

            while (improved && passes < two_opt_max_passes) {
                improved = false;
                ++passes;

                for (int i = 0; i < n - 1; ++i) {
                    for (int j = i + 2; j < n; ++j) {
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

                        if (cost_after < cost_before - 1e-6) {
                            int lo = i + 1;
                            int hi = j;
                            while (lo < hi) {
                                std::swap(out_order[lo], out_order[hi]);
                                int neg_lo = out_reversal[lo] ? 0 : 1;
                                int neg_hi = out_reversal[hi] ? 0 : 1;
                                out_reversal[lo] = neg_hi;
                                out_reversal[hi] = neg_lo;
                                ++lo;
                                --hi;
                            }
                            if (lo == hi) {
                                out_reversal[lo] = out_reversal[lo] ? 0 : 1;
                            }
                            improved = true;
                        }
                    }
                }
            }
        } else {
        // --- Windowed: only test the WINDOW_K spatially-nearest candidate
        // edges per i, found via a second kd-tree built over current edge
        // midpoints, instead of every (i, j) pair. Can miss a few distant
        // pairs that would've improved the tour; an accepted trade-off, not
        // a bug -- see the discussion this shipped with (K=24 gave ~11-15%
        // worse total travel than exhaustive; K=500 got that to ~2%, still
        // an order of magnitude fewer comparisons than O(n^2)).
        const int WINDOW_K = 500;

        bool improved = true;
        int  passes   = 0;

        // n-1 real edges + 1 phantom point for the open path's terminal
        // (out_order[n-1]'s own exit, not a midpoint of two things) so the
        // "reconnect to the tour's end" case the exhaustive version tests
        // via j == n-1 stays reachable by the spatial query too.
        std::vector<double> midpoints(3 * n);

        while (improved && passes < two_opt_max_passes) {
            improved = false;
            ++passes;

            // Rebuild fresh at the start of every pass: a reversal changes
            // which physical edge lives at which index (only the two
            // boundary edges of a reversed range actually change length;
            // the internal ones keep their geometry but shift index), so a
            // tree left over from a previous pass would hand back stale
            // candidates. Rebuild is O(n log n) -- negligible next to the
            // O(n*WINDOW_K) scan below, and far cheaper than the O(n^2) loop
            // it replaces. Swaps applied *within* this pass still use the
            // as-of-pass-start tree/query points for the remaining i's, which
            // can be mildly stale too -- not a correctness issue (the cost
            // comparison below always reads the live out_order/out_reversal
            // state, so whatever pair gets tested is evaluated correctly; a
            // stale candidate just means we occasionally query a spatially
            // suboptimal point), and any opportunity missed this way gets a
            // fresh look at the start of the next pass.
            for (int k = 0; k < n - 1; ++k) {
                int ep_exit  = exit_point (out_order[k],     out_reversal[k]);
                int ep_entry = entry_point(out_order[k + 1], out_reversal[k + 1]);
                midpoints[k * 3 + 0] = (endpoints[ep_exit * 3 + 0] + endpoints[ep_entry * 3 + 0]) / 2.0;
                midpoints[k * 3 + 1] = (endpoints[ep_exit * 3 + 1] + endpoints[ep_entry * 3 + 1]) / 2.0;
                midpoints[k * 3 + 2] = (endpoints[ep_exit * 3 + 2] + endpoints[ep_entry * 3 + 2]) / 2.0;
            }
            {
                int ep_last = exit_point(out_order[n - 1], out_reversal[n - 1]);
                midpoints[(n - 1) * 3 + 0] = endpoints[ep_last * 3 + 0];
                midpoints[(n - 1) * 3 + 1] = endpoints[ep_last * 3 + 1];
                midpoints[(n - 1) * 3 + 2] = endpoints[ep_last * 3 + 2];
            }

            EdgeMidpointCloud mid_cloud;
            mid_cloud.pts = midpoints.data();
            mid_cloud.num = n;

            MidpointKDTree mid_tree(3, mid_cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
            mid_tree.buildIndex();

            int k_query = WINDOW_K < n ? WINDOW_K : n;

            for (int i = 0; i < n - 1; ++i) {
                double q[3] = { midpoints[i * 3 + 0], midpoints[i * 3 + 1], midpoints[i * 3 + 2] };

                std::vector<uint32_t> idxs(k_query);
                std::vector<double>   dists(k_query);
                size_t got = mid_tree.knnSearch(&q[0], k_query, idxs.data(), dists.data());

                for (size_t r = 0; r < got; ++r) {
                    int cand = (int)idxs[r];
                    int lo = i < cand ? i : cand;
                    int hi = i < cand ? cand : i;
                    if (hi < lo + 2) {
                        continue;  // adjacent/self -- not a valid 2-opt pair
                    }

                    int exit_i   = exit_point (out_order[lo],     out_reversal[lo]);
                    int entry_i1 = entry_point(out_order[lo + 1], out_reversal[lo + 1]);
                    int exit_j   = exit_point (out_order[hi],     out_reversal[hi]);

                    double cost_before = dist_pts(endpoints, exit_i, entry_i1);
                    double cost_after  = dist_pts(endpoints, exit_i, exit_j);

                    if (hi + 1 < n) {
                        int entry_j1 = entry_point(out_order[hi + 1], out_reversal[hi + 1]);
                        cost_before += dist_pts(endpoints, exit_j,   entry_j1);
                        cost_after  += dist_pts(endpoints, entry_i1, entry_j1);
                    }

                    if (cost_after < cost_before - 1e-6) {
                        // Reverse sub-sequence [lo+1 .. hi]: flip order AND
                        // negate each reversal flag (a flipped curve is
                        // traversed the other way) -- same mechanics as the
                        // exhaustive version.
                        int p     = lo + 1;
                        int q_idx = hi;
                        while (p < q_idx) {
                            std::swap(out_order[p], out_order[q_idx]);
                            int neg_p = out_reversal[p]     ? 0 : 1;
                            int neg_q = out_reversal[q_idx] ? 0 : 1;
                            out_reversal[p]     = neg_q;
                            out_reversal[q_idx] = neg_p;
                            ++p;
                            --q_idx;
                        }
                        if (p == q_idx) {
                            out_reversal[p] = out_reversal[p] ? 0 : 1;
                        }
                        improved = true;
                    }
                }
            }
        }
        } // else (windowed)
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