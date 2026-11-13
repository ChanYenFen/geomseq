// shatter_at_crossings.cpp
// Cuts 2D segments where they meet and opens a gap there. Collinear overlap is
// refused by design: it has no single point to centre a gap on.

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

// Floor for every tolerance below, and the default where none is given.
// A distance in model units, not a raw cross-product threshold.
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

// True for a crossing, a collinear overlap, or an endpoint touch. Unused:
// kept as the only predicate here that answers for collinear overlap.
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

// Where AB meets CD, as a parameter along A->B. Covers a crossing and a touch
// (an endpoint of one resting on the other, i.e. a T-junction).
//
// The side tests are perpendicular distances -- cross product over segment
// length -- so touch_tol is in model units. Collinear overlap returns false.
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

    // Transversal crossing: t is where the distance from line CD reaches zero.
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

// True when the two segments meet end to end: a polyline's own joint, not a
// place where it crosses itself. Compares endpoints, not indices, so a closed
// polyline's first and last segments count too.
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
// Broad phase
// ---------------------------------------------------------------------------

// Every contact between two segments, as t along the one that yields (the
// higher index). ts[i] holds segment i's contacts, unsorted.
//
// Sweep and prune: segments are visited in order of their bounding box along
// one axis, and only pairs whose boxes overlap on both axes are tested. Boxes
// are widened by `pad`, which exceeds the tolerance segment_contact_param
// accepts, so no pair that could touch is skipped and the output is identical
// to testing every pair. Do not shrink `pad`: too little silently drops contacts.
//
// The sweep axis is the one whose boxes overlap least (hatching overlaps
// entirely in one axis and not at all in the other).
// See docs/design-notes.md, "Why shatter's broad phase is sweep and prune".
static void collect_contacts(const double* segments, int n, double touch_tol,
                             const int* segment_owner, int test_self,
                             std::vector<std::vector<double> >& ts)
{
    ts.assign(n, std::vector<double>());

    struct Box {
        double lo[2], hi[2];
        int    idx;
    };

    std::vector<Box> boxes;
    boxes.reserve(n);

    double sum_ext[2] = { 0.0, 0.0 };
    double all_lo[2]  = {  HUGE_VAL,  HUGE_VAL };
    double all_hi[2]  = { -HUGE_VAL, -HUGE_VAL };
    double mag        = 0.0;

    for (int i = 0; i < n; i++) {
        const double* s = segments + i * 6;
        Point A = { s[0], s[1] };
        Point B = { s[3], s[4] };

        // Too short for segment_contact_param to accept.
        if (pt_dist(A, B) < EPS) continue;

        Box b;
        b.idx = i;
        for (int a = 0; a < 2; a++) {
            b.lo[a] = std::min(s[a], s[a + 3]);
            b.hi[a] = std::max(s[a], s[a + 3]);
            sum_ext[a] += b.hi[a] - b.lo[a];
            all_lo[a]   = std::min(all_lo[a], b.lo[a]);
            all_hi[a]   = std::max(all_hi[a], b.hi[a]);
            mag = std::max(mag, std::max(std::fabs(b.lo[a]), std::fabs(b.hi[a])));
        }
        boxes.push_back(b);
    }
    if (boxes.size() < 2) return;

    double ratio[2];
    for (int a = 0; a < 2; a++) {
        double extent = all_hi[a] - all_lo[a];
        ratio[a] = (extent > EPS) ? sum_ext[a] / extent : HUGE_VAL;
    }
    const int ax = (ratio[1] < ratio[0]) ? 1 : 0;
    const int ox = 1 - ax;

    const double span = (touch_tol > EPS) ? touch_tol : EPS;
    const double pad  = 2.0 * span + 8.0 * std::numeric_limits<double>::epsilon() * mag;

    std::sort(boxes.begin(), boxes.end(),
              [ax](const Box& p, const Box& q) { return p.lo[ax] < q.lo[ax]; });

    std::vector<int> active;

    for (size_t k = 0; k < boxes.size(); k++) {
        const Box& bi = boxes[k];

        size_t kept = 0;
        for (size_t m = 0; m < active.size(); m++) {
            const Box& bj = boxes[active[m]];

            // Ended before this one starts, so before every later one too: drop it.
            if (bj.hi[ax] + pad < bi.lo[ax]) continue;
            active[kept++] = active[m];

            if (bj.hi[ox] + pad < bi.lo[ox] || bi.hi[ox] + pad < bj.lo[ox]) continue;

            const int hi = std::max(bi.idx, bj.idx);
            const int lo = std::min(bi.idx, bj.idx);
            const double* sh = segments + hi * 6;
            const double* sl = segments + lo * 6;

            if (segment_owner && segment_owner[lo] == segment_owner[hi]) {
                if (!test_self) continue;
                if (share_endpoint(sh, sl, touch_tol)) continue;  // a joint, not a crossing
            }

            Point A = { sh[0], sh[1] };
            Point B = { sh[3], sh[4] };
            Point C = { sl[0], sl[1] };
            Point D = { sl[3], sl[4] };

            double t;
            if (segment_contact_param(A, B, C, D, touch_tol, t)) {
                ts[hi].push_back(t);
            }
        }
        active.resize(kept);
        active.push_back((int)k);
    }
}

// ---------------------------------------------------------------------------
// Exported entry point
// ---------------------------------------------------------------------------

extern "C" {

// Cuts segments where they meet, removing a gap of gap_d centred on each
// contact so the two paths no longer touch there.
//
// Input order decides which segment yields: of a meeting pair the lower index
// is left intact and the higher one is cut. Sort the input first if another
// priority is wanted.
//
// Inputs (read-only):
//   segments      : flat array, 6 doubles per segment -> x0,y0,z0, x1,y1,z1
//   n             : number of segments
//   gap_d         : the full gap; each contact removes gap_d/2 to either side
//   touch_tol     : how close, in model units, counts as touching. 0 means
//                   exact only; set it from the document tolerance.
//   segment_owner : n ints naming the source curve of each segment (an
//                   exploded polyline gives many segments with one owner).
//                   May be nullptr: every segment then owns itself, and a
//                   polyline's own joints cannot be recognised.
//   test_self     : 0 skips pairs that share an owner; 1 tests them, except
//                   where they meet end to end (a joint, not a crossing).
//
// Only x and y take part in the geometry. z is interpolated along the segment.
//
// Outputs (caller pre-allocates, we fill):
//   out_segments     : surviving pieces, 6 doubles each, grouped in input order
//                      and ordered along the direction of their own segment
//   out_capacity     : how many pieces out_segments has room for
//   out_piece_counts : n ints, pieces produced per input segment; 0 means the
//                      gaps swallowed it whole
//   out_total        : total pieces REQUIRED. If greater than out_capacity,
//                      nothing past the capacity was written: call again with
//                      out_total as the new capacity.
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

    std::vector<std::vector<double> > ts;
    collect_contacts(segments, n, touch_tol, segment_owner, test_self, ts);

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

        // Emits the sub-segment spanning [t0, t1]. Always counts, but only
        // writes while the buffer has room, so out_total stays the true requirement.
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
            // Nothing to cut: passed through unchanged, so it keeps its slot.
            emit(0.0, 1.0);
            out_piece_counts[i] = pieces;
            continue;
        }

        double dt = (gap_d * 0.5) / len;

        removed.clear();
        for (size_t c = 0; c < ts[i].size(); c++) {
            removed.push_back(std::make_pair(ts[i][c] - dt, ts[i][c] + dt));
        }

        // What survives is the complement of the merged removal intervals
        // within [0, 1]; overlapping gaps merge.
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
