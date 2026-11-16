// point_in_curve.cpp
// Classifies 2D query points against closed polylines: outside, on the
// boundary, or inside. Nothing here knows about CAD objects, curves or
// planes -- the caller discretizes the curve and flattens both it and the
// query points into the working plane first (see design-notes.md, "Point in
// curve, and where it cannot match Rhino's").
//
// Two exports share the classification core, the way Grasshopper splits
// Point In Curve from Point In Curves: point_in_curve takes exactly one
// polyline and answers a relationship per point; point_in_curves takes any
// number and also answers which polyline, first in input order to contain the
// point, or -1.

#include <cmath>
#include <algorithm>
#include <limits>
#include <vector>

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

// Floor for every tolerance below. A distance in model units.
static const double EPS = 1e-9;

// A projected polygon counts as collapsed when its area falls below this
// fraction of its own bounding diagonal squared. Only ever raises a warning:
// the classification is still computed and returned, so this threshold cannot
// change an answer. Chosen to fire when the curve's plane is near enough to
// perpendicular to the working plane that "inside" has stopped meaning
// anything, not at the first loss of precision.
static const double AREA_WARN_RATIO = 1e-6;

// --- status codes, returned by point_in_curve -------------------------------
//
// Negative means nothing was written. Positive means the outputs are filled
// but suspect. The caller maps these onto its own warnings; see the C#
// component and geometry_utils.point_in_curve_native.
#define PIC_OK                 0
#define PIC_WARN_FLAT_AREA     1   // the projection collapsed; see AREA_WARN_RATIO
#define PIC_ERR_POLYLINE_COUNT (-1)
#define PIC_ERR_TOO_FEW_VERTS  (-2)
#define PIC_ERR_NOT_CLOSED     (-3)
#define PIC_ERR_BAD_ARGUMENT   (-4)
#define PIC_ERR_NEGATIVE_TOL   (-5)

// --- relationship codes, written into out_relationship ----------------------
//
// The integers Grasshopper's own Point In Curve reports in R, so the wrappers
// pass them straight through. NOT Rhino's PointContainment enum, which numbers
// the same three states differently (Unset 0, Inside 1, Outside 2,
// Coincident 3) -- do not substitute one for the other.
#define PIC_OUTSIDE    0
#define PIC_COINCIDENT 1
#define PIC_INSIDE     2

struct Point {
    double x, y;
};

// An axis-aligned box, per polyline.
struct Aabb {
    double lo[2], hi[2];
};

static Aabb aabb_of(const double* verts, int count) {
    Aabb b;
    b.lo[0] = b.lo[1] =  HUGE_VAL;
    b.hi[0] = b.hi[1] = -HUGE_VAL;
    for (int k = 0; k < count; k++) {
        for (int a = 0; a < 2; a++) {
            double v = verts[k * 2 + a];
            b.lo[a] = std::min(b.lo[a], v);
            b.hi[a] = std::max(b.hi[a], v);
        }
    }
    return b;
}

// Distance from q to the nearest point of the box; 0 when q is inside it.
//
// This is a lower bound on the distance from q to anything the box contains,
// which is what makes the rejection below provable rather than a guess.
static double dist_to_box(const Aabb& b, Point q) {
    double dx = std::max(0.0, std::max(b.lo[0] - q.x, q.x - b.hi[0]));
    double dy = std::max(0.0, std::max(b.lo[1] - q.y, q.y - b.hi[1]));
    return std::sqrt(dx * dx + dy * dy);
}

static double box_diagonal(const Aabb& b) {
    double w = b.hi[0] - b.lo[0];
    double h = b.hi[1] - b.lo[1];
    return std::sqrt(w * w + h * h);
}

// Twice the signed area of the closed polygon, by the shoelace sum. Sign is
// the winding direction; only the magnitude is used here.
static double double_signed_area(const double* verts, int count) {
    double sum = 0.0;
    for (int k = 0; k + 1 < count; k++) {
        double x0 = verts[k * 2],       y0 = verts[k * 2 + 1];
        double x1 = verts[(k + 1) * 2], y1 = verts[(k + 1) * 2 + 1];
        sum += x0 * y1 - x1 * y0;
    }
    return sum;
}

// First vertex repeated as the last, within tol. The caller's Curve.IsClosed
// is the authority; this catches a discretization or marshaling fault, where
// the polyline handed over is not the closed curve it came from.
static bool polyline_is_closed(const double* verts, int count, double tol) {
    double t = (tol > EPS) ? tol : EPS;
    double dx = verts[0] - verts[(count - 1) * 2];
    double dy = verts[1] - verts[(count - 1) * 2 + 1];
    return std::sqrt(dx * dx + dy * dy) <= t;
}

// ---------------------------------------------------------------------------
// The classification core
// ---------------------------------------------------------------------------

// Parity of the crossings a ray from q makes with the closed polyline:
// PIC_INSIDE for odd, PIC_OUTSIDE for even. The ray runs in +x, so only an
// edge spanning q's y matters, and the crossing is where that edge meets the
// horizontal line through q.
//
// verts holds `count` vertices, 2 doubles each, first repeated as last. The
// caller has already handled the boundary through nearest_on_polyline, so a
// point *on* an edge never reaches here and the strict comparisons below are
// not being asked to decide it.
//
// The span test is half open -- `(y0 > q.y) != (y1 > q.y)` -- which is what
// keeps a ray passing exactly through a vertex honest. At such a vertex two
// edges meet, and a test for "q.y lies between y0 and y1 inclusive" counts
// both, flipping the parity twice and reporting an interior point as outside.
// Written this way, the vertex belongs to whichever edge has it as its lower
// end, so exactly one of the two counts. Do not relax either comparison to
// >=.
static int crossing_number_parity(const double* verts, int count, Point q) {
    int crossings = 0;

    for (int k = 0; k + 1 < count; k++) {
        double x0 = verts[k * 2],       y0 = verts[k * 2 + 1];
        double x1 = verts[(k + 1) * 2], y1 = verts[(k + 1) * 2 + 1];

        if ((y0 > q.y) != (y1 > q.y)) {
            // y1 != y0 is guaranteed: the two differ in which side of q.y
            // they fall on, so the division is safe.
            double t  = (q.y - y0) / (y1 - y0);
            double xi = x0 + t * (x1 - x0);
            if (xi > q.x) {
                crossings++;
            }
        }
    }

    return (crossings & 1) ? PIC_INSIDE : PIC_OUTSIDE;
}

// Distance from q to the closed polyline: the smallest distance to any of its
// edges. Used for the coincident test, which is why it returns the distance
// rather than answering "within tol" -- the threshold belongs to the caller.
//
// Distances are compared squared and the root is taken once, at the end.
static double distance_to_polyline(const double* verts, int count, Point q) {
    double best = HUGE_VAL;

    for (int k = 0; k + 1 < count; k++) {
        Point a = { verts[k * 2],       verts[k * 2 + 1] };
        Point b = { verts[(k + 1) * 2], verts[(k + 1) * 2 + 1] };

        double dx   = b.x - a.x;
        double dy   = b.y - a.y;
        double len2 = dx * dx + dy * dy;

        // Where q projects onto the edge, clamped to it: past an end the
        // nearest point of the edge is that end. A repeated vertex leaves
        // len2 at zero and collapses to a.
        double t = 0.0;
        if (len2 > 0.0) {
            t = ((q.x - a.x) * dx + (q.y - a.y) * dy) / len2;
            t = (t < 0.0) ? 0.0 : (t > 1.0 ? 1.0 : t);
        }

        Point  c  = { a.x + t * dx, a.y + t * dy };
        double ex = q.x - c.x;
        double ey = q.y - c.y;
        double d2 = ex * ex + ey * ey;

        if (d2 < best) {
            best = d2;
        }
    }

    return std::sqrt(best);
}

// One query point against one closed polyline.
//
// The bounding-box rejection comes first. dist_to_box is a lower bound on the
// distance to every point of the polyline, so past span the point can be
// neither coincident with the boundary nor inside it -- the interior lies
// within the box too. Nothing is traded away: the answer is the one the full
// scan would give.
static int classify_one(const double* verts, int count, const Aabb& box,
                        double span, Point p) {
    if (dist_to_box(box, p) > span) {
        return PIC_OUTSIDE;
    }
    if (distance_to_polyline(verts, count, p) <= span) {
        return PIC_COINCIDENT;
    }
    return crossing_number_parity(verts, count, p);
}

// The polylines, validated and laid out for the query loop. Shared by both
// exports so that they refuse the same inputs for the same reasons.
struct Layout {
    std::vector<const double*> verts;
    std::vector<Aabb>          boxes;
    double                     span;
    int                        status;   // PIC_OK or PIC_WARN_FLAT_AREA
};

// Returns a negative PIC_* status when the input is refused, else fills `out`
// and returns PIC_OK. `min_polylines` is the one place the two exports differ.
static int prepare(const double* polylines, const int* vertex_counts,
                   int num_polylines, int min_polylines,
                   const double* query_pts, int num_query, double tol,
                   Layout& out) {
    if (num_polylines < min_polylines) {
        return PIC_ERR_POLYLINE_COUNT;
    }
    if (!polylines || !vertex_counts) {
        return PIC_ERR_BAD_ARGUMENT;
    }
    if (num_query < 0 || (num_query > 0 && !query_pts)) {
        return PIC_ERR_BAD_ARGUMENT;
    }
    if (tol < 0.0) {
        return PIC_ERR_NEGATIVE_TOL;
    }

    // A closed polygon needs three distinct corners plus the repeated first.
    for (int c = 0; c < num_polylines; c++) {
        if (vertex_counts[c] < 4) {
            return PIC_ERR_TOO_FEW_VERTS;
        }
    }

    out.verts.assign(num_polylines, nullptr);
    out.boxes.assign(num_polylines, Aabb());

    int cursor = 0;
    for (int c = 0; c < num_polylines; c++) {
        out.verts[c] = polylines + cursor * 2;
        cursor      += vertex_counts[c];

        if (!polyline_is_closed(out.verts[c], vertex_counts[c], tol)) {
            return PIC_ERR_NOT_CLOSED;
        }
        out.boxes[c] = aabb_of(out.verts[c], vertex_counts[c]);
    }

    // The projection has collapsed when a polygon encloses no area worth
    // speaking of: either flattened to a line (the curve's plane is near
    // perpendicular to the working plane) or to a point. Reported, not
    // refused -- the answers are still computed.
    out.status = PIC_OK;
    for (int c = 0; c < num_polylines; c++) {
        double diag = box_diagonal(out.boxes[c]);
        double area = std::fabs(double_signed_area(out.verts[c], vertex_counts[c])) * 0.5;
        if (diag < EPS || area < AREA_WARN_RATIO * diag * diag) {
            out.status = PIC_WARN_FLAT_AREA;
            break;
        }
    }

    out.span = (tol > EPS) ? tol : EPS;
    return PIC_OK;
}

// ---------------------------------------------------------------------------
// Exported entry points
// ---------------------------------------------------------------------------

extern "C" {

// Classifies each query point against one closed polyline.
//
// Inputs (read-only):
//   polylines      : flat array, 2 doubles per vertex -> x,y. Every polyline's
//                    vertices run consecutively, in the order vertex_counts
//                    names them, each with its first vertex repeated as its
//                    last. Length is 2 * sum(vertex_counts).
//   vertex_counts  : num_polylines ints, vertices in each polyline
//   num_polylines  : exactly 1; anything else is refused. Several curves is
//                    point_in_curves below, which also answers which one.
//   query_pts      : flat array, 2 doubles per query point -> x,y
//   num_query      : number of query points
//   tol            : how close to the boundary counts as PIC_COINCIDENT, in
//                    model units. 0 means exact only, floored at EPS. Set it
//                    from the document tolerance.
//
// Everything is already in the plane the containment is measured in: the
// caller does the projection, and no z appears in this signature.
//
// Outputs (caller pre-allocates, we fill):
//   out_relationship : num_query ints -> PIC_OUTSIDE / PIC_COINCIDENT / PIC_INSIDE
//
// Returns a PIC_* status. Below zero nothing was written.
DLL_EXPORT int point_in_curve(
    const double* polylines,
    const int*    vertex_counts,
    int           num_polylines,
    const double* query_pts,
    int           num_query,
    double        tol,
    int*          out_relationship)
{
    if (num_polylines != 1) {
        return PIC_ERR_POLYLINE_COUNT;
    }
    if (!out_relationship) {
        return PIC_ERR_BAD_ARGUMENT;
    }

    Layout L;
    int err = prepare(polylines, vertex_counts, num_polylines, 1,
                      query_pts, num_query, tol, L);
    if (err < 0) {
        return err;
    }

    for (int q = 0; q < num_query; q++) {
        Point p = { query_pts[q * 2], query_pts[q * 2 + 1] };
        out_relationship[q] = classify_one(L.verts[0], vertex_counts[0], L.boxes[0],
                                           L.span, p);
    }

    return L.status;
}

// Classifies each query point against several closed polylines, and says
// which one. Inputs are as for point_in_curve, except that num_polylines may
// be any count of one or more.
//
// Each point is tested against the polylines in input order and stops at the
// first that contains it, on the boundary or inside. That one's index is the
// answer, and its relationship. A point no polyline contains is PIC_OUTSIDE
// with index -1. Nested curves therefore answer for the outermost one listed
// first, not the innermost -- the rule Grasshopper's own Point In Curves is
// read to follow, and the cheapest one, since the loop can stop on a hit.
//
// Outputs (caller pre-allocates, we fill):
//   out_relationship : num_query ints -> PIC_OUTSIDE / PIC_COINCIDENT / PIC_INSIDE
//   out_index        : num_query ints -> the containing polyline, or -1
//
// Returns a PIC_* status. Below zero nothing was written.
DLL_EXPORT int point_in_curves(
    const double* polylines,
    const int*    vertex_counts,
    int           num_polylines,
    const double* query_pts,
    int           num_query,
    double        tol,
    int*          out_relationship,
    int*          out_index)
{
    if (!out_relationship || !out_index) {
        return PIC_ERR_BAD_ARGUMENT;
    }

    Layout L;
    int err = prepare(polylines, vertex_counts, num_polylines, 1,
                      query_pts, num_query, tol, L);
    if (err < 0) {
        return err;
    }

    for (int q = 0; q < num_query; q++) {
        Point p = { query_pts[q * 2], query_pts[q * 2 + 1] };

        out_relationship[q] = PIC_OUTSIDE;
        out_index[q]        = -1;

        for (int c = 0; c < num_polylines; c++) {
            int r = classify_one(L.verts[c], vertex_counts[c], L.boxes[c], L.span, p);
            if (r != PIC_OUTSIDE) {
                out_relationship[q] = r;
                out_index[q]        = c;
                break;
            }
        }
    }

    return L.status;
}

} // extern "C"
