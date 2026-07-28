// sort_points.cpp
// Native point sorting for the Embroidery pipeline. Single-point sibling of
// sort_curves.cpp: same greedy nearest-neighbor (k-NN via kd-tree) + 2-opt
// algorithm, but over plain points instead of curve endpoints -- so there is
// no direction/reversal concept here.
//
// Deliberately duplicates the small kd-tree plumbing (point cloud adaptor,
// KDTree typedef, dist_pts) instead of sharing a header with sort_curves.cpp,
// to avoid touching that file's already-verified logic.

#include "nanoflann.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

// ---------------------------------------------------------------------------
// Data source that nanoflann queries to build the kd-tree: n plain 3D points.
// Contrast with sort_curves's EndpointCloud, which holds 2*n endpoints and
// decodes a curve index via pt/2. Here the kd-tree point index IS the point
// index -- no decoding needed.
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
//
// Outputs (caller pre-allocates, we fill):
//   out_order : n ints -> original point indices in sorted order
DLL_EXPORT void sort_points(
    const double* points,
    int           n,
    const double* start_pt,
    int           use_two_opt,
    int           two_opt_max_passes,
    int           knn_k,
    int*          out_order)
{
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
    // No direction to worry about, so a single 1-NN query suffices (unlike
    // sort_curves's growing-window search, which exists only to skip tails
    // when if_flip == 0).
    {
        double q[3] = { start_pt[0], start_pt[1], start_pt[2] };

        std::vector<uint32_t> idxs(1);
        std::vector<double>   dists(1);
        size_t got = tree.knnSearch(&q[0], 1, idxs.data(), dists.data());
        (void)got;  // always 1 here: n > 0 already checked above

        int first_point = (int)idxs[0];

        out_order[0]     = first_point;
        used[first_point] = true;

        cur[0] = points[first_point * 3 + 0];
        cur[1] = points[first_point * 3 + 1];
        cur[2] = points[first_point * 3 + 2];
    }

    // --- Greedy: fill positions 1 .. n-1 ---
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

        out_order[placed]  = best_point;
        used[best_point]   = true;

        cur[0] = points[best_point * 3 + 0];
        cur[1] = points[best_point * 3 + 1];
        cur[2] = points[best_point * 3 + 2];
    }

    // --- 2-opt post-processing (point-only: no reversal flags to maintain) ---
    // For each pair (i, j), compare current edges against the edges obtained by
    // reversing the sub-sequence i+1 .. j. Accept the reversal if strictly cheaper.
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
                        // negate here (unlike sort_curves) -- points have no
                        // direction, so a plain index reversal is enough.
                        std::reverse(out_order + i + 1, out_order + j + 1);
                        improved = true;
                    }
                }
            }
        }
    }
}

} // extern "C"
