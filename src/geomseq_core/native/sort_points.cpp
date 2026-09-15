// sort_points.cpp
// Single-point sibling of sort_curves.cpp -- same greedy k-NN + 2-opt, but
// over plain points (no direction/reversal); kd-tree plumbing is duplicated rather than shared.

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
    // Neighbour-pruned rather than every pair, the same rule sort_curves.cpp
    // uses and for the same reason. A move drops the edges after positions i and
    // j and reconnects them:
    //
    //     remove: order[i] -> order[i+1]   (length gap_i)
    //             order[j] -> order[j+1]   (length gap_j)
    //     add:    order[i] -> order[j]
    //             order[i+1] -> order[j+1]
    //
    // It can only pay if at least one new edge is shorter than the old edge it
    // is measured against -- if both were longer, so would be their sum. So
    // every improving move satisfies
    //
    //     d(order[i], order[j]) < gap_i   OR   d(order[i+1], order[j+1]) < gap_j
    //
    // and each half is a ball query the kd-tree built for the greedy phase can
    // answer. Both scans are needed: the two radii belong to opposite ends of
    // the move, so neither alone is complete.
    //
    // Simpler here than in sort_curves: a kd-tree index *is* a point index, with
    // no curve to decode and no entry/exit to pick, and the reversal is a plain
    // std::reverse because points have no direction. Only `pos` has to be kept
    // in step, so a candidate point can be turned back into a tour position.
    //
    // Nothing improving is discarded, so this still finishes at a true 2-opt
    // local optimum -- but not the *same* one the exhaustive loop found. Both
    // take the first improving move they meet and the kd-tree meets them in a
    // different order, so recorded travel figures do not reproduce to the digit
    // across this change. See CLAUDE.md.
    if (use_two_opt && n > 3) {
        // Where each point currently sits in the tour.
        std::vector<int> pos(n);
        for (int k = 0; k < n; ++k) {
            pos[out_order[k]] = k;
        }

        std::vector<nanoflann::ResultItem<uint32_t, double>> hits;
        nanoflann::SearchParameters search_params;  // sorted: nearest candidates first

        // Applies the (i, j) move if it shortens the tour. Same arithmetic and
        // same 1e-6 margin as the exhaustive version it replaces.
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
                    // Needs to sit far enough along to leave a segment to
                    // reverse; this also drops order[k] and order[k+1] themselves.
                    if (j < k + 2) {
                        continue;
                    }
                    if (try_move(k, j)) {
                        improved = true;
                        break;  // tour changed under us; carry on at the next k
                    }
                }

                // Scan B: k is the move's j, so look for an i+1 whose point lies
                // within gap of order[k+1]. Needs k >= 2 to leave room for i >= 0.
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
