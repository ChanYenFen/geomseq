// sort_points_no_crossing.cpp
// Native point sorting for the Embroidery pipeline, for stitch paths where
// self-crossing is not acceptable. Greedy nearest-neighbor fill is identical
// to sort_points.cpp (no crossing awareness -- a hard reject during greedy
// can dead-end with no guaranteed zero-crossing continuation, a math
// limitation, not an implementation gap). Crossing avoidance instead happens
// as a forced-reversal 2-opt pass after the path is already complete, which
// can't get stuck the way greedy can.
//
// Deliberately duplicates the small kd-tree plumbing (point cloud adaptor,
// KDTree typedef, dist_pts) instead of sharing a header with sort_points.cpp,
// same principle as that file's own header comment.

#include "geometry2d.h"
#include "nanoflann.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

// ---------------------------------------------------------------------------
// Data source that nanoflann queries to build the kd-tree: n plain 3D points.
// Same as sort_points.cpp's PointCloud -- the kd-tree point index IS the
// point index, no decoding needed.
// ---------------------------------------------------------------------------
struct PointCloud {
    const double* pts;  // borrowed pointer to the flat points buffer (n*3)
    int           num;  // number of points = n

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
    nanoflann::L2_Simple_Adaptor<double, PointCloud>,
    PointCloud,
    3
> KDTree;

// Euclidean distance between two points, given their point indices.
static double dist_pts(const double* pts, int a, int b) {
    double dx = pts[a * 3 + 0] - pts[b * 3 + 0];
    double dy = pts[a * 3 + 1] - pts[b * 3 + 1];
    double dz = pts[a * 3 + 2] - pts[b * 3 + 2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

// Currently unused. Was the greedy-phase hard-reject check; superseded by
// the crossing-removal 2-opt phase below (greedy hitting a dead end with no
// valid candidate has no guaranteed zero-crossing solution -- a math
// limitation, not an implementation gap). Kept as reference / for future
// strategy comparisons.
static bool candidate_is_safe(int pt, const double* points, const int* out_order, int placed) {
    // The new edge we're considering: from the path's current end
    // (out_order[placed-1]) to the candidate point pt.
    int cur_idx = out_order[placed - 1];

    double curx = points[cur_idx * 3 + 0];
    double cury = points[cur_idx * 3 + 1];
    double newx = points[pt * 3 + 0];
    double newy = points[pt * 3 + 1];

    // Check the new edge against every already-committed edge except the
    // immediately preceding one -- that edge shares the `cur` endpoint with
    // the new edge by construction, which segments_intersect's collinear-
    // touch case (on_segment with r == p) always reports as a "crossing".
    // Skipping it is the standard adjacent-edge exclusion for polyline
    // self-intersection checks.
    for (int i = 0; i < placed - 2; ++i) {
        int e1_idx = out_order[i];
        int e2_idx = out_order[i + 1];

        double e1x = points[e1_idx * 3 + 0];
        double e1y = points[e1_idx * 3 + 1];
        double e2x = points[e2_idx * 3 + 0];
        double e2y = points[e2_idx * 3 + 1];

        if (segments_intersect(curx, cury, newx, newy, e1x, e1y, e2x, e2y)) {
            return false;  // crosses an already-committed edge -- reject
        }
    }

    return true;  // no crossing found -- safe to use
}

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

// The single entry point GH/ctypes will call.
//
// Inputs (read-only):
//   points    : flat array, 3 doubles per point -> x,y,z
//   n         : number of points
//   start_pt  : 3 doubles -> reference point to pick the first point
//   use_two_opt        : 0/1 flag
//   two_opt_max_passes : cap on 2-opt passes
//   knn_k     : how many nearest neighbors to query per greedy step
//   is_closed : 0 = open path, 1 = closed loop (path also has an implicit
//               edge from the last point back to the first)
//
// Outputs (caller pre-allocates, we fill):
//   out_order : n ints -> original point indices in sorted order
DLL_EXPORT void sort_points_no_crossing(
    const double* points,
    int           n,
    const double* start_pt,
    int           use_two_opt,
    int           two_opt_max_passes,
    int           knn_k,
    int           is_closed,
    int*          out_order)
{
    // TODO: is_closed is currently a dead parameter -- the implicit closing
    // edge (last point -> first point) is never checked for crossings, even
    // when is_closed == 1. Don't assume a closed loop is crossing-free.

    if (n <= 0) {
        return;
    }

    // --- Build the kd-tree over all n points ---
    PointCloud cloud;
    cloud.pts = points;
    cloud.num = n;

    KDTree tree(3, cloud, nanoflann::KDTreeSingleIndexAdaptorParams(10));
    tree.buildIndex();

    // Tracks which points are already placed in the chain.
    std::vector<bool> used(n, false);

    // The current position of the path (where the next point should connect).
    double cur[3];

    // --- Pick the first point: nearest to start_pt ---
    {
        double q[3] = { start_pt[0], start_pt[1], start_pt[2] };

        std::vector<uint32_t> idxs(1);
        std::vector<double>   dists(1);
        size_t got = tree.knnSearch(&q[0], 1, idxs.data(), dists.data());
        (void)got;  // always 1 here: n > 0 already checked above

        int first_point = (int)idxs[0];

        out_order[0]      = first_point;
        used[first_point] = true;

        cur[0] = points[first_point * 3 + 0];
        cur[1] = points[first_point * 3 + 1];
        cur[2] = points[first_point * 3 + 2];
    }

    // --- Greedy: fill positions 1 .. n-1 ---
    // Same as sort_points.cpp -- no crossing check here. A hard reject at
    // this stage can dead-end with no guaranteed zero-crossing continuation
    // (a math limitation, not an implementation gap); crossings are instead
    // force-resolved by the 2-opt phase below, which can't get stuck since
    // it operates on an already-complete path.
    for (int placed = 1; placed < n; ++placed) {
        int best_point = -1;

        // Query k neighbors; if all are used, grow the search and retry.
        int sc = knn_k;
        while (best_point == -1) {
            if (sc > n) {
                sc = n;
            }

            std::vector<uint32_t> idxs(sc);
            std::vector<double>   dists(sc);
            size_t got = tree.knnSearch(&cur[0], sc, idxs.data(), dists.data());

            for (size_t r = 0; r < got; ++r) {
                int pt = (int)idxs[r];
                if (used[pt]) {
                    continue;
                }
                best_point = pt;
                break;
            }

            if (best_point == -1) {
                if (sc == n) {
                    break;  // nothing unused left (safety; shouldn't happen)
                }
                sc *= 2;
            }
        }

        if (best_point == -1) {
            break;  // safety break
        }

        out_order[placed] = best_point;
        used[best_point]  = true;

        cur[0] = points[best_point * 3 + 0];
        cur[1] = points[best_point * 3 + 1];
        cur[2] = points[best_point * 3 + 2];
    }

    // --- Phase 1: crossing removal (forced, not cost-based) -- ALWAYS runs,
    // regardless of use_two_opt. This is the function's actual no-crossing
    // guarantee now that greedy no longer checks crossings (see above).
    //
    // For each pair (i, j) with j >= i+2, test edge i=(out_order[i],
    // out_order[i+1]) against edge j=(out_order[j], out_order[j+1]); if they
    // cross, force-reverse the sub-sequence [i+1..j] regardless of the
    // resulting distance. j >= i+2 keeps edge i and edge j from sharing an
    // endpoint (adjacent edges sharing a vertex would otherwise always
    // false-positive as "crossing" -- same degenerate case candidate_is_safe
    // hit above). j < n-1 ensures out_order[j+1] is a real committed edge;
    // an open path has no edge past the last point, so there's nothing to
    // test j against when j == n-1.
    //
    // TODO: doesn't consider the is_closed wraparound edge (see TODO above).
    // Not guaranteed to reach zero crossings within two_opt_max_passes --
    // that cap is a safety valve against runaway iteration, not a proof of
    // convergence.
    if (n > 3) {
        bool improved = true;
        int  passes   = 0;

        while (improved && passes < two_opt_max_passes) {
            improved = false;
            ++passes;

            for (int i = 0; i < n - 1; ++i) {
                for (int j = i + 2; j < n - 1; ++j) {
                    double a1x = points[out_order[i]     * 3 + 0], a1y = points[out_order[i]     * 3 + 1];
                    double a2x = points[out_order[i + 1] * 3 + 0], a2y = points[out_order[i + 1] * 3 + 1];
                    double b1x = points[out_order[j]     * 3 + 0], b1y = points[out_order[j]     * 3 + 1];
                    double b2x = points[out_order[j + 1] * 3 + 0], b2y = points[out_order[j + 1] * 3 + 1];

                    if (segments_intersect(a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y)) {
                        std::reverse(out_order + i + 1, out_order + j + 1);
                        improved = true;
                    }
                }
            }
        }
    }

    // --- Phase 2: optional pure-distance optimization (point-only: no
    // reversal flags to maintain), gated by use_two_opt. For each pair
    // (i, j), compare current edges against the edges obtained by reversing
    // the sub-sequence i+1 .. j. Accept the reversal if strictly cheaper.
    // KNOWN LIMITATION: this has no crossing check, so it can reintroduce
    // crossings that phase 1 just removed. The no-crossing guarantee only
    // holds when use_two_opt is off.
    if (use_two_opt && n > 3) {
        bool improved = true;
        int  passes   = 0;

        while (improved && passes < two_opt_max_passes) {
            improved = false;
            ++passes;

            for (int i = 0; i < n - 1; ++i) {
                for (int j = i + 2; j < n; ++j) {
                    double cost_before = dist_pts(points, out_order[i], out_order[i + 1]);
                    double cost_after  = dist_pts(points, out_order[i], out_order[j]);

                    if (j + 1 < n) {
                        cost_before += dist_pts(points, out_order[j],     out_order[j + 1]);
                        cost_after  += dist_pts(points, out_order[i + 1], out_order[j + 1]);
                    }

                    if (cost_after < cost_before - 1e-6) {
                        // Reverse sub-sequence [i+1 .. j]. No reversal flags to
                        // negate here -- points have no direction, so a plain
                        // index reversal is enough.
                        std::reverse(out_order + i + 1, out_order + j + 1);
                        improved = true;
                    }
                }
            }
        }
    }
}

} // extern "C"
