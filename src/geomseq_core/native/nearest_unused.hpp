// nearest_unused.hpp
// Nearest-*unused*-point queries over a built nanoflann static kd-tree, for
// the greedy phase of sort_points and sort_curves.
//
// The greedy phase used to ask the tree for k neighbours, skip the ones already
// placed, and double k until an unused one appeared. Placed points never leave
// the tree, so late in the walk a query could fetch thousands of them only to
// throw them away -- the reason greedy grew towards O(n^2).
//
// This keeps, for every node, a count of the points beneath it that are still
// allowed, and a search that returns as soon as it reaches a subtree whose
// count is zero. That skip is provable: a subtree with nothing allowed in it
// cannot hold the answer. Nothing else is skipped that the k-NN search would
// not also have skipped, so the answer is the same nearest unused point.
//
// The answer is also the same *one* when several tie. Both searches walk the
// tree in the same depth-first order (nearer child first, a leaf's points in
// vAcc_ order) and both replace the current best only on a strictly smaller
// distance, so among equal distances the first point visited wins -- which is
// where nanoflann's sorted k-NN result puts it too. The distance arithmetic is
// nanoflann's own (evalMetric / accum_dist), so ties are ties in both.
//
// The tree is copied into a flat array in depth-first order, so a node is
// found by index rather than by chasing nanoflann's pointers, and the per-node
// counts sit next to the split data the search reads anyway.

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

template <class Tree>
class NearestUnused {
public:
    // `allowed[pt]` says whether point pt may ever be returned. A point that is
    // never allowed (a curve's tail when direction is fixed) is simply never
    // counted, so it costs nothing to skip.
    NearestUnused(const Tree& tree, const std::vector<char>& allowed)
        : tree_(tree), alive_(allowed), leaf_of_(allowed.size(), -1)
    {
        nodes_.reserve(64);
        flatten(tree.root_node_, -1);
    }

    // Stop returning pt. A no-op if pt is not currently allowed.
    void remove(uint32_t pt)
    {
        if (!alive_[pt]) {
            return;
        }
        alive_[pt] = 0;
        for (int id = leaf_of_[pt]; id != -1; id = nodes_[id].parent) {
            --nodes_[id].live;
        }
    }

    // Nearest allowed point to q, or -1 if none is left.
    int nearest(const double* q) const
    {
        if (nodes_.empty() || nodes_[0].live == 0) {
            return -1;
        }
        double dists[3] = { 0.0, 0.0, 0.0 };
        double mindist = initial_distance(q, dists);
        double best_d  = std::numeric_limits<double>::max();
        int    best_pt = -1;
        search(0, q, mindist, dists, best_d, best_pt);
        return best_pt;
    }

private:
    struct FlatNode {
        int    child1, child2;    // -1 on a leaf
        int    parent;            // -1 at the root
        int    divfeat;
        double divlow, divhigh;
        size_t left, right;       // leaf only: range in vAcc_
        int    live;              // allowed points in this subtree
    };

    const Tree&          tree_;
    std::vector<char>    alive_;
    std::vector<int>     leaf_of_;
    std::vector<FlatNode> nodes_;

    // Depth-first copy of nanoflann's tree; returns the node's flat id.
    int flatten(const typename Tree::Node* node, int parent)
    {
        int id = (int)nodes_.size();
        nodes_.push_back(FlatNode());
        nodes_[id].parent = parent;
        nodes_[id].live   = 0;

        if (!node->child1) {
            nodes_[id].child1 = nodes_[id].child2 = -1;
            nodes_[id].left   = node->node_type.lr.left;
            nodes_[id].right  = node->node_type.lr.right;
            int live = 0;
            for (size_t i = nodes_[id].left; i < nodes_[id].right; ++i) {
                uint32_t pt = tree_.vAcc_[i];
                leaf_of_[pt] = id;
                live += alive_[pt] ? 1 : 0;
            }
            nodes_[id].live = live;
            return id;
        }

        nodes_[id].divfeat = node->node_type.sub.divfeat;
        nodes_[id].divlow  = node->node_type.sub.divlow;
        nodes_[id].divhigh = node->node_type.sub.divhigh;
        // push_back may move nodes_, so hold ids, never references, across it.
        int c1 = flatten(node->child1, id);
        int c2 = flatten(node->child2, id);
        nodes_[id].child1 = c1;
        nodes_[id].child2 = c2;
        nodes_[id].live   = nodes_[c1].live + nodes_[c2].live;
        return id;
    }

    // nanoflann's computeInitialDistances: distance from q to the root box.
    double initial_distance(const double* q, double* dists) const
    {
        double dist = 0.0;
        for (int i = 0; i < 3; ++i) {
            if (q[i] < tree_.root_bbox_[i].low) {
                dists[i] = tree_.distance_.accum_dist(q[i], tree_.root_bbox_[i].low, i);
                dist += dists[i];
            } else if (q[i] > tree_.root_bbox_[i].high) {
                dists[i] = tree_.distance_.accum_dist(q[i], tree_.root_bbox_[i].high, i);
                dist += dists[i];
            }
        }
        return dist;
    }

    // nanoflann's searchLevel with a one-slot result, plus the live-count skip.
    void search(int id, const double* q, double mindist, double* dists,
                double& best_d, int& best_pt) const
    {
        const FlatNode& node = nodes_[id];
        if (node.live == 0) {
            return;  // nothing allowed below: cannot hold the answer
        }

        if (node.child1 == -1) {
            for (size_t i = node.left; i < node.right; ++i) {
                uint32_t pt = tree_.vAcc_[i];
                if (!alive_[pt]) {
                    continue;
                }
                double d = tree_.distance_.evalMetric(q, pt, 3);
                if (d < best_d) {
                    best_d  = d;
                    best_pt = (int)pt;
                }
            }
            return;
        }

        int    idx   = node.divfeat;
        double val   = q[idx];
        double diff1 = val - node.divlow;
        double diff2 = val - node.divhigh;

        int    best_child, other_child;
        double cut_dist;
        if ((diff1 + diff2) < 0) {
            best_child  = node.child1;
            other_child = node.child2;
            cut_dist    = tree_.distance_.accum_dist(val, node.divhigh, idx);
        } else {
            best_child  = node.child2;
            other_child = node.child1;
            cut_dist    = tree_.distance_.accum_dist(val, node.divlow, idx);
        }

        search(best_child, q, mindist, dists, best_d, best_pt);

        double dst = dists[idx];
        mindist    = mindist + cut_dist - dst;
        dists[idx] = cut_dist;
        if (mindist <= best_d) {
            search(other_child, q, mindist, dists, best_d, best_pt);
        }
        dists[idx] = dst;
    }
};
