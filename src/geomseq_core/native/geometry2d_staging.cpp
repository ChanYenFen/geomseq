// geometry2d_staging.cpp
// Experimental 2D geometric primitives -- staging, see README Modules.

#include <cmath>
#include <algorithm>
#include <vector>

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

// Floor for every tolerance below, and the tolerance used where none is given.
// Note this is a DISTANCE: the side tests divide their cross product by the
// segment length before comparing, so the threshold means the same thing at any
// coordinate magnitude. A raw cross product would not -- it scales with
// coordinate size times segment length, so an absolute epsilon on it silently
// tightens as a drawing grows.
static const double EPS = 1e-9;

struct Point {
    double x, y;
};

static double sgn_tol(double v, double tol) {
    double t = (tol > EPS) ? tol : EPS;
    if (v >  t) return  1;
    if (v < -t) return -1;
    return 0;
}

static double cross(Point p, Point q, Point r) {
    return (q.x - p.x) * (r.y - p.y) - (q.y - p.y) * (r.x - p.x);
}

static double pt_dist(Point p, Point q) {
    double dx = q.x - p.x, dy = q.y - p.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Precondition: r is already known to lie on the infinite line through p-q.
static bool on_segment(Point p, Point q, Point r, double tol) {
    return std::min(p.x, q.x) - tol <= r.x && r.x <= std::max(p.x, q.x) + tol &&
           std::min(p.y, q.y) - tol <= r.y && r.y <= std::max(p.y, q.y) + tol;
}

// Where r falls along p->q, as a parameter. Precondition: p != q.
static double project_t(Point p, Point q, Point r) {
    double dx = q.x - p.x, dy = q.y - p.y;
    return ((r.x - p.x) * dx + (r.y - p.y) * dy) / (dx * dx + dy * dy);
}

// General contact predicate: true for a transversal crossing, a collinear
// overlap, or a bare endpoint touch. Nothing calls it -- shatter_at_crossings
// needs the position of the contact, which segment_contact_param reports -- but
// it is the only thing here that answers for collinear overlap, which is where
// this would be picked up again.
static bool segment_intersect(Point A, Point B, Point C, Point D) {
    double d1 = cross(A, B, C);
    double d2 = cross(A, B, D);
    double d3 = cross(C, D, A);
    double d4 = cross(C, D, B);

    if (sgn_tol(d1, 0.0) * sgn_tol(d2, 0.0) < 0 &&
        sgn_tol(d3, 0.0) * sgn_tol(d4, 0.0) < 0) return true;

    if (sgn_tol(d1, 0.0) == 0 && on_segment(A, B, C, EPS)) return true;
    if (sgn_tol(d2, 0.0) == 0 && on_segment(A, B, D, EPS)) return true;
    if (sgn_tol(d3, 0.0) == 0 && on_segment(C, D, A, EPS)) return true;
    if (sgn_tol(d4, 0.0) == 0 && on_segment(C, D, B, EPS)) return true;

    return false;
}

// Where AB meets CD, as a parameter along A->B. Covers both a transversal
// crossing and a touch -- an endpoint of one landing on the other, which is
// what a T-junction is, and what a toolpath is full of.
//
// The side tests are PERPENDICULAR DISTANCES, not raw cross products: each is
// divided by the length of the segment it is measured against, so touch_tol is
// a distance in model units and can be set from the document tolerance. A
// junction drawn 1e-6 out -- which is most of them -- reads as a touch rather
// than as a crossing a hair further along.
//
// Collinear overlap is still refused: it has no single point to centre a gap
// on. segment_intersect above is the one that answers for it.
static bool segment_contact_param(Point A, Point B, Point C, Point D,
                                  double touch_tol, double& out_t) {
    double ab_len = pt_dist(A, B);
    double cd_len = pt_dist(C, D);
    if (ab_len < EPS || cd_len < EPS) return false;

    double g1 = sgn_tol(cross(A, B, C) / ab_len, touch_tol);
    double g2 = sgn_tol(cross(A, B, D) / ab_len, touch_tol);
    double g3 = sgn_tol(cross(C, D, A) / cd_len, touch_tol);
    double g4 = sgn_tol(cross(C, D, B) / cd_len, touch_tol);

    // Both endpoints of one on the line of the other: collinear, no single point.
    if (g1 == 0 && g2 == 0) return false;
    if (g3 == 0 && g4 == 0) return false;

    // Transversal crossing -- the parameter comes from the same cross products,
    // since d3 and d4 are proportional to how far A and B sit from line CD and
    // t is where that distance reaches zero.
    if (g1 * g2 < 0 && g3 * g4 < 0) {
        double d3 = cross(C, D, A);
        double d4 = cross(C, D, B);
        double denom = d3 - d4;
        if (std::fabs(denom) < EPS) return false;
        out_t = d3 / denom;
        return true;
    }

    double span = (touch_tol > EPS) ? touch_tol : EPS;

    // An endpoint of AB resting on CD: the contact is that endpoint itself.
    if (g3 == 0 && on_segment(C, D, A, span)) { out_t = 0.0; return true; }
    if (g4 == 0 && on_segment(C, D, B, span)) { out_t = 1.0; return true; }

    // An endpoint of CD resting on AB: the contact is that point, projected
    // back onto AB to get its parameter.
    if (g1 == 0 && on_segment(A, B, C, span)) { out_t = project_t(A, B, C); return true; }
    if (g2 == 0 && on_segment(A, B, D, span)) { out_t = project_t(A, B, D); return true; }

    return false;
}

// True when the two segments meet end to end. Used to tell a polyline's own
// joints apart from a place where it genuinely crosses itself -- without this,
// admitting touches would cut every polyline at every vertex. Comparing the
// endpoints rather than the indices also covers a closed polyline, whose first
// and last segments are joined but not adjacent in the buffer.
static bool share_endpoint(const double* si, const double* sj, double tol) {
    double t = (tol > EPS) ? tol : EPS;
    const int ends[2] = { 0, 3 };
    for (int a = 0; a < 2; a++) {
        for (int b = 0; b < 2; b++) {
            if (std::fabs(si[ends[a]]     - sj[ends[b]])     <= t &&
                std::fabs(si[ends[a] + 1] - sj[ends[b] + 1]) <= t) return true;
        }
    }
    return false;
}

// ---------------------------------------------------------------------------
// Exported entry point
// ---------------------------------------------------------------------------

extern "C" {

// Cuts segments where they meet, removing a gap of gap_d centred on each
// contact so the two paths no longer touch there.
//
// Which segment yields is decided by input order: for a meeting pair the lower
// index is left intact and the higher one is cut, so segment i is only ever cut
// by the segments before it. Sort the input first if some other priority is
// wanted -- this function does not reorder anything. That rule covers a
// T-junction too: the branch ends at the contact, so cutting it retracts its
// tip by gap_d/2, which is the same clearance a crossing leaves.
//
// Inputs (read-only):
//   segments      : flat array, 6 doubles per segment -> x0,y0,z0, x1,y1,z1
//   n             : number of segments
//   gap_d         : the full gap -- how far apart the two cut ends end up.
//                   Each contact removes gap_d/2 to either side of itself.
//   touch_tol     : how close, in model units, counts as touching rather than
//                   missing. 0 means exact only. Set it from the document
//                   tolerance: a T-junction is rarely drawn to the last bit,
//                   and below this it would otherwise read as a near miss.
//   segment_owner : n ints naming which source curve each segment came from.
//                   Exploding a polyline gives many segments with one owner.
//                   May be nullptr, which means every segment owns itself --
//                   but then a polyline's own joints cannot be recognised, so
//                   pass real owners for anything exploded.
//   test_self     : 0 skips pairs that share an owner, so a polyline is not cut
//                   where it crosses itself; 1 tests them. Either way, two
//                   segments of one curve that meet end to end are skipped:
//                   that is a joint, not a crossing.
//
// Only x and y take part in the geometry -- crossing is a 2D question, same as
// build_turn_waypoints. z is carried through by interpolating it along the
// segment, so a flat path keeps its height and a sloped one stays on its line.
//
// Outputs (caller pre-allocates, we fill):
//   out_segments     : surviving pieces, 6 doubles each, grouped in input order
//                      and ordered along the direction of their own segment
//   out_capacity     : how many pieces out_segments has room for
//   out_piece_counts : n ints, how many pieces each input segment produced. A
//                      segment swallowed whole by the gaps reports 0 rather
//                      than vanishing, so the index mapping of the caller
//                      survives.
//   out_total        : total pieces REQUIRED. Coming back greater than
//                      out_capacity means the buffer was too small: nothing
//                      past the capacity was written, and the call should be
//                      repeated with out_total as the new capacity. The worst
//                      case here is quadratic (every pair crossing), which is
//                      why the caller is told the requirement instead of being
//                      asked to allocate for a bound almost no input reaches.
DLL_EXPORT void shatter_at_crossings(
    const double* segments,
    int           n,
    double        gap_d,
    double        touch_tol,
    const int*    segment_owner,
    int           test_self,
    double*       out_segments,
    int           out_capacity,
    int*          out_piece_counts,
    int*          out_total)
{
    int needed = 0;

    std::vector<std::pair<double, double> > removed;

    for (int i = 0; i < n; i++) {
        const double* si = segments + i * 6;
        Point  A  = { si[0], si[1] };
        Point  B  = { si[3], si[4] };
        double az = si[2];
        double bz = si[5];

        double dx  = B.x - A.x;
        double dy  = B.y - A.y;
        double len = std::sqrt(dx * dx + dy * dy);

        int  pieces   = 0;
        bool zero_len = (len < EPS);

        // Emits the sub-segment spanning [t0, t1] of this input segment.
        // Always counts, but only writes while the buffer of the caller has
        // room -- that is what lets out_total report the true requirement
        // after an undersized call.
        auto emit = [&](double t0, double t1) {
            if (!zero_len && (t1 - t0) * len < EPS) return;  // numerical crumb, not a piece
            pieces++;
            int slot = needed++;
            if (slot >= out_capacity) return;
            double* o = out_segments + slot * 6;
            o[0] = A.x + t0 * dx;
            o[1] = A.y + t0 * dy;
            o[2] = az  + t0 * (bz - az);
            o[3] = A.x + t1 * dx;
            o[4] = A.y + t1 * dy;
            o[5] = az  + t1 * (bz - az);
        };

        if (zero_len) {
            // Nothing to cut, and no direction to measure gap_d along. Passed
            // through unchanged rather than dropped, so it still occupies its
            // slot in the index mapping of the caller.
            emit(0.0, 1.0);
            out_piece_counts[i] = pieces;
            continue;
        }

        double dt = (gap_d * 0.5) / len;

        removed.clear();
        for (int j = 0; j < i; j++) {
            const double* sj = segments + j * 6;

            if (segment_owner && segment_owner[j] == segment_owner[i]) {
                if (!test_self) continue;
                if (share_endpoint(si, sj, touch_tol)) continue;  // a joint, not a crossing
            }

            Point C = { sj[0], sj[1] };
            Point D = { sj[3], sj[4] };

            double t;
            if (segment_contact_param(A, B, C, D, touch_tol, t)) {
                removed.push_back(std::make_pair(t - dt, t + dt));
            }
        }

        // What survives is the complement of the merged removal intervals
        // within [0, 1]. Cutting at every contact, and dropping a piece a gap
        // swallows whole, both fall out of this rather than needing a branch of
        // their own: intervals covering [0, 1] simply leave nothing behind, and
        // overlapping gaps merge instead of cutting each other into crumbs.
        std::sort(removed.begin(), removed.end());

        double cursor = 0.0;
        for (size_t k = 0; k < removed.size(); k++) {
            double lo = std::min(removed[k].first, 1.0);
            double hi = removed[k].second;
            if (lo > cursor) emit(cursor, lo);
            if (hi > cursor) cursor = hi;
        }
        if (cursor < 1.0) emit(cursor, 1.0);

        out_piece_counts[i] = pieces;
    }

    *out_total = needed;
}

} // extern "C"
