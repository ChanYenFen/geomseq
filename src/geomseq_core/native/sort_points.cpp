// sort_points.cpp
// Orders points to shorten travel between them: greedy nearest-neighbour
// (kd-tree), then 2-opt. The sibling of sort_curves.cpp, without direction.

#include "nanoflann.hpp"
#include "nearest_unused.hpp"
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

// nanoflann's view of the points buffer (borrowed, not copied): n points,
// 3 doubles each. A kd-tree index is the point index.
struct PointCloud {
    const double* pts;
    int           num;  // n

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

// Euclidean distance between two points, by point index.
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

// Inputs (read-only):
//   points    : flat array, 3 doubles per point -> x,y,z
//   n         : number of points
//   start_pt  : 3 doubles -> where the path starts; picks the first point
//   use_two_opt        : 0/1 flag
//   two_opt_max_passes : cap on 2-opt passes
//   knn_k     : unused; kept so the signature does not change
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

    // Every point starts allowed and is removed once placed.
    NearestUnused<KDTree> unused(tree, std::vector<char>(n, 1));

    // The current end of the path: start_pt, then each placed point.
    double cur[3] = { start_pt[0], start_pt[1], start_pt[2] };

    // --- Greedy: fill positions 0 .. n-1 ---
    (void)knn_k;
    for (int placed = 0; placed < n; ++placed) {
        int best_point = unused.nearest(cur);
        if (best_point == -1) {
            break;  // nothing unused left (shouldn't happen)
        }

        out_order[placed] = best_point;
        unused.remove((uint32_t)best_point);

        cur[0] = points[best_point * 3 + 0];
        cur[1] = points[best_point * 3 + 1];
        cur[2] = points[best_point * 3 + 2];
    }

    // --- 2-opt ---
    // A move drops the edges after positions i and j and reconnects them:
    //
    //     remove: order[i] -> order[i+1]   (length gap_i)
    //             order[j] -> order[j+1]   (length gap_j)
    //     add:    order[i] -> order[j]
    //             order[i+1] -> order[j+1]
    //
    // The search is pruned, and provably loses nothing: a move can only shorten
    // the tour if at least one new edge is shorter than the old edge it replaces,
    //
    //     d(order[i], order[j]) < gap_i   OR   d(order[i+1], order[j+1]) < gap_j
    //
    // so each half is a radius query on the kd-tree. Both scans are needed;
    // neither alone is complete. See docs/design-notes.md, "Why the 2-opt
    // search is pruned".
    if (use_two_opt && n > 3) {
        // Tour position of each point, kept in step with every reversal below.
        std::vector<int> pos(n);
        for (int k = 0; k < n; ++k) {
            pos[out_order[k]] = k;
        }

        std::vector<nanoflann::ResultItem<uint32_t, double>> hits;
        nanoflann::SearchParameters search_params;  // sorted: nearest candidates first

        // Applies the (i, j) move if it shortens the tour by more than 1e-6.
        auto try_move = [&](int i, int j) -> bool {
            double cost_before = dist_pts(points, out_order[i], out_order[i + 1]);
            double cost_after  = dist_pts(points, out_order[i], out_order[j]);

            if (j + 1 < n) {
                cost_before += dist_pts(points, out_order[j],     out_order[j + 1]);
                cost_after  += dist_pts(points, out_order[i + 1], out_order[j + 1]);
            }

            if (cost_after >= cost_before - 1e-6) {
                return false;
            }

            std::reverse(out_order + i + 1, out_order + j + 1);
            for (int k = i + 1; k <= j; ++k) {
                pos[out_order[k]] = k;
            }
            return true;
        };

        bool improved = true;
        int  passes   = 0;

        while (improved && passes < two_opt_max_passes) {
            improved = false;
            ++passes;

            for (int k = 0; k + 1 < n; ++k) {
                // Scan A: k is the move's i, so look for a j whose point lies
                // within gap of order[k]. radiusSearch works in squared distance.
                double gap = dist_pts(points, out_order[k], out_order[k + 1]);

                hits.clear();
                (void)tree.radiusSearch(&points[out_order[k] * 3], gap * gap,
                                        hits, search_params);

                for (size_t h = 0; h < hits.size(); ++h) {
                    int j = pos[(int)hits[h].first];
                    // j must leave a segment to reverse.
                    if (j < k + 2) {
                        continue;
                    }
                    if (try_move(k, j)) {
                        improved = true;
                        break;  // tour changed; carry on at the next k
                    }
                }

                // Scan B: k is the move's j, so look for an i+1 whose point lies
                // within gap of order[k+1]. Needs k >= 2 to leave room for i >= 0.
                // The gap is recomputed because scan A may have just changed it.
                if (k >= 2) {
                    gap = dist_pts(points, out_order[k], out_order[k + 1]);

                    hits.clear();
                    (void)tree.radiusSearch(&points[out_order[k + 1] * 3], gap * gap,
                                            hits, search_params);

                    for (size_t h = 0; h < hits.size(); ++h) {
                        int m = pos[(int)hits[h].first];  // the candidate is position i+1
                        if (m < 1 || m > k - 1) {
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
}

} // extern "C"
