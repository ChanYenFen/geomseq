// coverage_path.cpp
// Native port of src/geomseq_core/coverage_path.py: one continuous path over
// a closed region. The Python module is the reference: every function here
// mirrors one there, under the same name, and the tests compare the two.
// Read the Python docstrings for why each step is the way it is; the comments
// here only note where the port has to take care to match.
//
// Two exports: coverage_hatch_region (decompose + fill_cells) and
// coverage_plan (the whole path: travel graph, order, walks). Results come
// back through a handle, because their sizes are not known before the run:
// run, read each field, free.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <new>
#include <set>
#include <unordered_map>
#include <vector>

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

namespace {

// --- status codes ------------------------------------------------------------
// Negative: nothing was produced. The Python bridge maps each to the
// ValueError message the pure-Python module raises for the same input.
const int CP_OK = 0;
const int CP_ERR_BAD_ARGUMENT = -1;    // null pointer, no rings, bad count
const int CP_ERR_NEGATIVE_TOL = -2;    // "tol cannot be negative"
const int CP_ERR_RING_TOO_SHORT = -3;  // "a ring needs at least three distinct vertices"
const int CP_ERR_NO_HEIGHT = -4;       // "the region has no height along the sweep direction"
const int CP_ERR_ODD_CROSSINGS = -5;   // "a scanline crossed the boundary an odd number of times..."
const int CP_ERR_SPACING = -6;         // "spacing must be greater than 0"
const int CP_ERR_LINK = -7;            // "link must be ZIGZAG (0) or ONE_WAY (1)"
const int CP_ERR_INTERNAL = -99;       // out of memory or an unexpected failure

const int ZIGZAG = 0, ONE_WAY = 1;

struct Pt {
    double x, y;
};

inline bool operator==(const Pt& a, const Pt& b) { return a.x == b.x && a.y == b.y; }
inline bool operator!=(const Pt& a, const Pt& b) { return !(a == b); }

struct Row {
    double y, xl, xr;
};

struct Error {
    int code;
};

inline Pt rotate(const Pt& p, double c, double s) {
    return {c * p.x - s * p.y, s * p.x + c * p.y};
}

inline double dist(const Pt& a, const Pt& b) { return std::hypot(a.x - b.x, a.y - b.y); }

// --- the cut -----------------------------------------------------------------

struct Edge {
    Pt a, b;
};

struct Cell {
    std::vector<Pt> outline;  // input frame
    std::vector<Row> rows;    // sweep frame
};

struct Portal {
    int cell_a, cell_b;
    Pt p0, p1;
};

struct Decomposition {
    std::vector<Cell> cells;
    std::vector<Portal> portals;
    double angle;
    std::vector<std::vector<Pt>> rings;
};

struct Trap {
    double xl_lo, xr_lo, xl_hi, xr_hi;
};

struct PortalRow {
    int a, b;
    double y, x0, x1;
};

double x_at(const Edge& e, double y) {
    return e.a.x + (y - e.a.y) * (e.b.x - e.a.x) / (e.b.y - e.a.y);
}

std::vector<double> slab_levels(const std::vector<Edge>& edges, double tol) {
    std::set<double> ys;
    for (const Edge& e : edges) {
        ys.insert(e.a.y);
        ys.insert(e.b.y);
    }
    std::vector<double> levels;
    for (double y : ys) {
        if (levels.empty() || y - levels.back() > tol)
            levels.push_back(y);
    }
    if (levels.size() < 2)
        throw Error{CP_ERR_NO_HEIGHT};
    return levels;
}

std::vector<Trap> slab_spans(const std::vector<Edge>& edges, double y_lo, double y_hi, double tol) {
    double y_mid = 0.5 * (y_lo + y_hi);
    std::vector<std::pair<double, int>> crossing;
    for (int i = 0; i < (int)edges.size(); ++i) {
        const Edge& e = edges[i];
        if ((e.a.y < y_mid) != (e.b.y < y_mid))
            crossing.push_back({x_at(e, y_mid), i});
    }
    // Python's sort is stable; ties keep edge order.
    std::stable_sort(crossing.begin(), crossing.end(),
                     [](const std::pair<double, int>& p, const std::pair<double, int>& q) {
                         return p.first < q.first;
                     });
    if (crossing.size() % 2)
        throw Error{CP_ERR_ODD_CROSSINGS};

    struct Pair {
        double xm0, xm1;
        int e0, e1;
    };
    std::vector<Pair> pairs;
    for (size_t i = 0; i < crossing.size(); i += 2) {
        double xm0 = crossing[i].first, xm1 = crossing[i + 1].first;
        int e0 = crossing[i].second, e1 = crossing[i + 1].second;
        if (!pairs.empty() && xm0 - pairs.back().xm1 <= tol) {
            pairs.back().xm1 = xm1;
            pairs.back().e1 = e1;
        } else {
            pairs.push_back({xm0, xm1, e0, e1});
        }
    }
    std::vector<Trap> spans;
    for (const Pair& p : pairs) {
        if (p.xm1 - p.xm0 <= tol)
            continue;
        spans.push_back({x_at(edges[p.e0], y_lo), x_at(edges[p.e1], y_lo),
                         x_at(edges[p.e0], y_hi), x_at(edges[p.e1], y_hi)});
    }
    return spans;
}

void link_slabs(const std::vector<std::vector<Trap>>& trapezoids, const std::vector<double>& levels,
                double tol, std::vector<std::vector<Row>>& cells_rows, std::vector<PortalRow>& portals) {
    std::vector<std::vector<int>> cell_of(trapezoids.size());
    for (size_t k = 0; k < trapezoids.size(); ++k)
        cell_of[k].assign(trapezoids[k].size(), -1);

    static const std::vector<Trap> none;
    for (size_t k = 0; k < trapezoids.size(); ++k) {
        const std::vector<Trap>& here = trapezoids[k];
        const std::vector<Trap>& below = k > 0 ? trapezoids[k - 1] : none;
        double y_lo = levels[k], y_hi = levels[k + 1];

        struct Overlap {
            int i;
            double x0, x1;
        };
        std::vector<std::vector<Overlap>> overlaps(here.size());
        std::vector<std::vector<int>> above(below.size());
        for (size_t i = 0; i < below.size(); ++i) {
            double bl = below[i].xl_hi, br = below[i].xr_hi;
            for (size_t j = 0; j < here.size(); ++j) {
                double hl = here[j].xl_lo, hr = here[j].xr_lo;
                double x0 = std::max(bl, hl), x1 = std::min(br, hr);
                if (x1 - x0 > tol) {
                    overlaps[j].push_back({(int)i, x0, x1});
                    above[i].push_back((int)j);
                }
            }
        }

        for (size_t j = 0; j < here.size(); ++j) {
            const Trap& t = here[j];
            bool one_to_one = overlaps[j].size() == 1 && above[overlaps[j][0].i].size() == 1;
            if (one_to_one) {
                int cell = cell_of[k - 1][overlaps[j][0].i];
                cell_of[k][j] = cell;
                std::vector<Row>& rows = cells_rows[cell];
                const Row& last = rows.back();
                if (std::fabs(last.xl - t.xl_lo) > tol || std::fabs(last.xr - t.xr_lo) > tol)
                    rows.push_back({y_lo, t.xl_lo, t.xr_lo});
                rows.push_back({y_hi, t.xl_hi, t.xr_hi});
            } else {
                cell_of[k][j] = (int)cells_rows.size();
                cells_rows.push_back({{y_lo, t.xl_lo, t.xr_lo}, {y_hi, t.xl_hi, t.xr_hi}});
                for (const Overlap& o : overlaps[j])
                    portals.push_back({cell_of[k - 1][o.i], cell_of[k][j], y_lo, o.x0, o.x1});
            }
        }
    }
}

std::vector<Pt> rows_to_outline(const std::vector<Row>& rows) {
    std::vector<Pt> pts;
    for (const Row& r : rows)
        pts.push_back({r.xr, r.y});
    for (auto it = rows.rbegin(); it != rows.rend(); ++it)
        pts.push_back({it->xl, it->y});
    std::vector<Pt> out;
    for (const Pt& p : pts) {
        if (out.empty() || out.back() != p)
            out.push_back(p);
    }
    if (out.size() > 1 && out.front() == out.back())
        out.pop_back();
    return out;
}

std::vector<Pt> strip_closing(const std::vector<Pt>& ring) {
    std::vector<Pt> pts = ring;
    if (pts.size() > 1 && pts.front() == pts.back())
        pts.pop_back();
    return pts;
}

Decomposition decompose(const std::vector<std::vector<Pt>>& rings, double angle, double tol) {
    if (tol < 0.0)
        throw Error{CP_ERR_NEGATIVE_TOL};
    double c = std::cos(-angle), s = std::sin(-angle);
    std::vector<Edge> edges;
    Decomposition dec;
    for (const auto& ring : rings) {
        std::vector<Pt> clean = strip_closing(ring);
        if (clean.size() < 3)
            throw Error{CP_ERR_RING_TOO_SHORT};
        dec.rings.push_back(clean);
        std::vector<Pt> pts;
        for (const Pt& p : clean)
            pts.push_back(rotate(p, c, s));
        for (size_t i = 0; i < pts.size(); ++i)
            edges.push_back({pts[i], pts[(i + 1) % pts.size()]});
    }

    std::vector<double> levels = slab_levels(edges, tol);
    std::vector<std::vector<Trap>> trapezoids;
    for (size_t k = 0; k + 1 < levels.size(); ++k)
        trapezoids.push_back(slab_spans(edges, levels[k], levels[k + 1], tol));

    std::vector<std::vector<Row>> cells_rows;
    std::vector<PortalRow> portal_rows;
    link_slabs(trapezoids, levels, tol, cells_rows, portal_rows);

    double cb = std::cos(angle), sb = std::sin(angle);
    for (auto& rows : cells_rows) {
        Cell cell;
        for (const Pt& p : rows_to_outline(rows))
            cell.outline.push_back(rotate(p, cb, sb));
        cell.rows = rows;
        dec.cells.push_back(cell);
    }
    for (const PortalRow& p : portal_rows)
        dec.portals.push_back({p.a, p.b, rotate({p.x0, p.y}, cb, sb), rotate({p.x1, p.y}, cb, sb)});
    dec.angle = angle;
    return dec;
}

// --- the fill ----------------------------------------------------------------

struct Line {
    double y, xl, xr;  // sweep frame
};

struct Fill {
    int cell;
    std::vector<Pt> walks[4];                // input frame
    std::vector<std::pair<Pt, Pt>> lines;    // input frame, (left end, right end)
};

// The cell's x-interval on scanline y; false where the cell does not reach.
bool interval_at(const std::vector<Row>& rows, double y, double& xl, double& xr) {
    bool found = false;
    for (size_t k = 0; k + 1 < rows.size(); ++k) {
        double y0 = rows[k].y, l0 = rows[k].xl, r0 = rows[k].xr;
        double y1 = rows[k + 1].y, l1 = rows[k + 1].xl, r1 = rows[k + 1].xr;
        if (y < y0 || y > y1)
            continue;
        double a, b;
        if (y1 == y0) {
            a = std::min(l0, l1);
            b = std::max(r0, r1);
        } else {
            double t = (y - y0) / (y1 - y0);
            a = l0 + t * (l1 - l0);
            b = r0 + t * (r1 - r0);
        }
        xl = found ? std::min(xl, a) : a;
        xr = found ? std::max(xr, b) : b;
        found = true;
    }
    return found;
}

// Points and lines of one cell's zig-zag, in the sweep frame.
void zigzag(const std::vector<Row>& rows, double spacing, double phase, double tol, bool start_left,
            bool drop_top, std::vector<Pt>& pts, std::vector<Line>& lines) {
    pts.clear();
    lines.clear();
    double y_bot = rows.front().y, y_top = rows.back().y;
    // int(math.ceil(...)) and int(math.floor(...)) in the Python.
    long long k0 = (long long)std::ceil((y_bot - phase) / spacing - 1e-9);
    long long k1;
    if (drop_top)
        k1 = (long long)std::ceil((y_top - phase) / spacing - 1e-9) - 1;
    else
        k1 = (long long)std::floor((y_top - phase) / spacing + 1e-9);
    for (long long k = k0; k <= k1; ++k) {
        double y = phase + (double)k * spacing;
        y = std::min(std::max(y, y_bot), y_top);
        double xl, xr;
        if (interval_at(rows, y, xl, xr) && xr - xl > tol)
            lines.push_back({y, xl, xr});
    }
    if (lines.empty()) {
        double y = 0.5 * (y_bot + y_top);
        double xl, xr;
        if (!interval_at(rows, y, xl, xr) || xr - xl <= tol)
            return;
        lines.push_back({y, xl, xr});
    }
    for (size_t n = 0; n < lines.size(); ++n) {
        const Line& ln = lines[n];
        bool left_to_right = ((n % 2) == 0) == start_left;
        if (n > 0) {
            double y_prev = lines[n - 1].y;
            for (const Row& r : rows) {
                if (y_prev < r.y && r.y < ln.y)
                    pts.push_back({left_to_right ? r.xl : r.xr, r.y});
            }
        }
        if (left_to_right) {
            pts.push_back({ln.xl, ln.y});
            pts.push_back({ln.xr, ln.y});
        } else {
            pts.push_back({ln.xr, ln.y});
            pts.push_back({ln.xl, ln.y});
        }
    }
}

// (y, lo, hi) at each row height strictly between y_lo and y_hi, the
// narrower interval where the cell steps at that height, ascending in y.
std::vector<Row> narrowest(const std::vector<Row>& rows, double y_lo, double y_hi) {
    std::map<double, std::pair<double, double>> out;
    for (const Row& r : rows) {
        if (y_lo < r.y && r.y < y_hi) {
            auto it = out.find(r.y);
            if (it != out.end())
                it->second = {std::max(it->second.first, r.xl), std::min(it->second.second, r.xr)};
            else
                out[r.y] = {r.xl, r.xr};
        }
    }
    std::vector<Row> levels;
    for (const auto& kv : out)
        levels.push_back({kv.first, kv.second.first, kv.second.second});
    return levels;
}

std::vector<Pt> shortest_in_cell(const std::vector<Row>& rows, const Pt& a, const Pt& b, double tol) {
    std::vector<Row> levels = narrowest(rows, a.y, b.y);
    auto clear = [&](const Pt& p, const Pt& q) {
        for (const Row& l : levels) {
            if (p.y < l.y && l.y < q.y) {
                double x = p.x + (q.x - p.x) * (l.y - p.y) / (q.y - p.y);
                if (x < l.xl - tol || x > l.xr + tol)
                    return false;
            }
        }
        return true;
    };
    if (clear(a, b))
        return {a, b};
    std::vector<Pt> nodes;
    nodes.push_back(a);
    for (const Row& l : levels) {
        nodes.push_back({l.xl, l.y});
        nodes.push_back({l.xr, l.y});
    }
    nodes.push_back(b);
    const double inf = std::numeric_limits<double>::infinity();
    std::vector<double> d(nodes.size(), inf);
    std::vector<int> prev(nodes.size(), -1);
    d[0] = 0.0;
    for (size_t j = 1; j < nodes.size(); ++j) {
        for (size_t i = 0; i < j; ++i) {
            if (d[i] < inf && nodes[i].y < nodes[j].y && clear(nodes[i], nodes[j])) {
                double nd = d[i] + dist(nodes[i], nodes[j]);
                if (nd < d[j]) {
                    d[j] = nd;
                    prev[j] = (int)i;
                }
            }
        }
    }
    std::vector<Pt> path;
    for (int j = (int)nodes.size() - 1; j != -1; j = prev[j])
        path.push_back(nodes[j]);
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Pt> one_way(const std::vector<Row>& rows, const std::vector<Line>& lines, bool start_left,
                        double tol) {
    std::vector<Pt> pts;
    for (size_t n = 0; n < lines.size(); ++n) {
        const Line& ln = lines[n];
        Pt a = start_left ? Pt{ln.xl, ln.y} : Pt{ln.xr, ln.y};
        Pt b = start_left ? Pt{ln.xr, ln.y} : Pt{ln.xl, ln.y};
        if (n > 0) {
            std::vector<Pt> join = shortest_in_cell(rows, pts.back(), a, tol);
            for (size_t i = 1; i + 1 < join.size(); ++i)
                pts.push_back(join[i]);
        }
        pts.push_back(a);
        pts.push_back(b);
    }
    return pts;
}

std::vector<Fill> fill_cells(const Decomposition& dec, double spacing, double phase, int link,
                             double tol) {
    if (!(spacing > 0.0))
        throw Error{CP_ERR_SPACING};
    if (link != ZIGZAG && link != ONE_WAY)
        throw Error{CP_ERR_LINK};
    double cb = std::cos(dec.angle), sb = std::sin(dec.angle);
    std::set<int> has_above;
    for (const Portal& p : dec.portals)
        has_above.insert(p.cell_a);
    std::vector<Fill> fills;
    for (size_t i = 0; i < dec.cells.size(); ++i) {
        const std::vector<Row>& rows = dec.cells[i].rows;
        bool drop_top = has_above.count((int)i) > 0;
        std::vector<Pt> pts, mirrored;
        std::vector<Line> lines, unused;
        zigzag(rows, spacing, phase, tol, true, drop_top, pts, lines);
        zigzag(rows, spacing, phase, tol, false, drop_top, mirrored, unused);
        if (link == ONE_WAY) {
            pts = one_way(rows, lines, true, tol);
            mirrored = one_way(rows, lines, false, tol);
        }
        Fill f;
        f.cell = (int)i;
        for (const Pt& p : pts)
            f.walks[0].push_back(rotate(p, cb, sb));
        for (const Pt& p : mirrored)
            f.walks[2].push_back(rotate(p, cb, sb));
        f.walks[1].assign(f.walks[0].rbegin(), f.walks[0].rend());
        f.walks[3].assign(f.walks[2].rbegin(), f.walks[2].rend());
        for (const Line& ln : lines)
            f.lines.push_back({rotate({ln.xl, ln.y}, cb, sb), rotate({ln.xr, ln.y}, cb, sb)});
        fills.push_back(f);
    }
    return fills;
}

// --- the travel --------------------------------------------------------------

const int CP_ERR_KICKS = -8;           // "kicks cannot be negative"
const int CP_ERR_PIECES = -9;          // "the cells left lie in another piece of the region"

inline bool same(const Pt& a, const Pt& b, double tol) { return std::hypot(a.x - b.x, a.y - b.y) <= tol; }

// Python compares points as tuples: x first, then y.
inline bool lex_less(const Pt& a, const Pt& b) { return a.x < b.x || (a.x == b.x && a.y < b.y); }

struct PtLess {
    bool operator()(const Pt& a, const Pt& b) const { return lex_less(a, b); }
};

struct PairLess {
    bool operator()(const std::pair<Pt, Pt>& a, const std::pair<Pt, Pt>& b) const {
        if (lex_less(a.first, b.first)) return true;
        if (lex_less(b.first, a.first)) return false;
        return lex_less(a.second, b.second);
    }
};

bool on_segment(const Pt& p, const Pt& a, const Pt& b, double tol) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double L2 = dx * dx + dy * dy;
    if (L2 == 0.0)
        return same(p, a, tol);
    double t = std::min(std::max(((p.x - a.x) * dx + (p.y - a.y) * dy) / L2, 0.0), 1.0);
    return same(p, {a.x + t * dx, a.y + t * dy}, tol);
}

bool point_in_ring(const Pt& p, const std::vector<Pt>& ring) {
    bool inside = false;
    size_t n = ring.size();
    for (size_t i = 0; i < n; ++i) {
        const Pt& a = ring[i];
        const Pt& b = ring[(i + 1) % n];
        if ((a.y > p.y) != (b.y > p.y)) {
            double xi = a.x + (p.y - a.y) * (b.x - a.x) / (b.y - a.y);
            if (p.x < xi)
                inside = !inside;
        }
    }
    return inside;
}

std::vector<double> cut_params(const Pt& a, const Pt& b, const std::vector<std::pair<Pt, Pt>>& edges,
                               double tol) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double L = std::hypot(dx, dy);
    std::vector<double> ts = {0.0, 1.0};
    for (const auto& e : edges) {
        const Pt& p = e.first;
        const Pt& q = e.second;
        double ex = q.x - p.x, ey = q.y - p.y;
        double apx = p.x - a.x, apy = p.y - a.y;
        double denom = dx * ey - dy * ex;
        if (std::fabs(denom) > tol * L * std::hypot(ex, ey)) {
            double t = (apx * ey - apy * ex) / denom;
            double u = (apx * dy - apy * dx) / denom;
            if (0.0 <= t && t <= 1.0 && 0.0 <= u && u <= 1.0)
                ts.push_back(t);
        } else if (std::fabs(apx * dy - apy * dx) <= tol * L) {
            const Pt* ends[2] = {&p, &q};
            for (const Pt* r : ends) {
                double t = ((r->x - a.x) * dx + (r->y - a.y) * dy) / (L * L);
                if (0.0 <= t && t <= 1.0)
                    ts.push_back(t);
            }
        }
    }
    std::sort(ts.begin(), ts.end());
    return ts;
}

inline Pt mid(const Pt& a, const Pt& b) { return {0.5 * (a.x + b.x), 0.5 * (a.y + b.y)}; }

bool foot_on_segment(const Pt& p, const Pt& a, const Pt& b, double tol, Pt& foot) {
    double dx = b.x - a.x, dy = b.y - a.y;
    double L2 = dx * dx + dy * dy;
    if (L2 == 0.0)
        return false;
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / L2;
    double slack = tol / std::sqrt(L2);
    if (t < -slack || t > 1.0 + slack)
        return false;
    t = std::min(std::max(t, 0.0), 1.0);
    foot = {a.x + t * dx, a.y + t * dy};
    return true;
}

std::vector<Pt> step_targets(const Pt& e, const std::pair<Pt, Pt>& line, double tol) {
    Pt foot;
    if (!foot_on_segment(e, line.first, line.second, tol, foot))
        return {};
    double gap = dist(e, foot);
    Pt m = mid(line.first, line.second);
    double toward = dist(foot, m);
    if (gap <= tol || toward <= tol)
        return {foot};
    double s = std::min(gap, toward) / toward;
    return {foot, {foot.x + s * (m.x - foot.x), foot.y + s * (m.y - foot.y)}};
}

// A set of cells, as the Python's frozensets: bit c for cell c.
struct Bits {
    std::vector<uint64_t> w;
    Bits() {}
    explicit Bits(size_t n) : w((n + 63) / 64, 0) {}
    void set(int c) { w[c >> 6] |= uint64_t(1) << (c & 63); }
    bool disjoint(const Bits& o) const {
        for (size_t i = 0; i < w.size(); ++i)
            if (w[i] & o.w[i])
                return false;
        return true;
    }
    bool operator==(const Bits& o) const { return w == o.w; }
    bool any() const {
        for (uint64_t x : w)
            if (x)
                return true;
        return false;
    }
};

struct Piece {
    double length;
    Bits cells;
};
typedef std::vector<Piece> Pieces;

inline double shown(const Pieces& pieces, const Bits& hidden) {
    double s = 0.0;
    for (const Piece& p : pieces)
        if (p.cells.disjoint(hidden))
            s += p.length;
    return s;
}

Pieces merge_pieces(const std::vector<const Pieces*>& runs) {
    Pieces out;
    for (const Pieces* pieces : runs)
        for (const Piece& p : *pieces) {
            if (!out.empty() && out.back().cells == p.cells)
                out.back().length += p.length;
            else
                out.push_back(p);
        }
    return out;
}

struct Cost {
    double vis, len;
    bool operator<(const Cost& o) const { return vis < o.vis || (vis == o.vis && len < o.len); }
    Cost operator+(const Cost& o) const { return {vis + o.vis, len + o.len}; }
};

// A cost or nothing, as the Python's None.
struct OptCost {
    bool ok = false;
    Cost c{0.0, 0.0};
};

// A link of the search graph. via: the points a folded run passes through,
// in the link's own direction; rev reads them backwards.
struct Link {
    int v;
    double len;
    int pieces;
    int via;   // -1 for none
    bool rev;
};

struct SearchOut {
    std::vector<OptCost> dist;
    std::vector<Link> prev;
    std::vector<int> prev_u;
};

// _search: Dijkstra over a graph read through nbrs(u, f), f called for every
// link out of u in the Python's adjacency order; cost (visible, length).
// Stops once every target is settled, if any are given. Heap entries compare
// as the Python's (visible, length, node) tuples, so ties break the same way.
template <class Nbrs>
void search(int n, const Nbrs& nbrs, const std::vector<Pieces>& pool, int src, const Bits& hidden,
            const std::vector<int>& targets, SearchOut& out) {
    out.dist.assign(n, OptCost());
    out.prev.assign(n, Link{-1, 0.0, -1, -1, false});
    out.prev_u.assign(n, -1);
    out.dist[src].ok = true;
    struct Entry {
        double vis, len;
        int node;
    };
    auto worse = [](const Entry& a, const Entry& b) {
        if (a.vis != b.vis) return a.vis > b.vis;
        if (a.len != b.len) return a.len > b.len;
        return a.node > b.node;
    };
    std::vector<Entry> heap;
    heap.push_back({0.0, 0.0, src});
    std::vector<char> is_target(n, 0);
    int left = 0;
    for (int t : targets)
        if (!is_target[t]) {
            is_target[t] = 1;
            ++left;
        }
    while (!heap.empty()) {
        std::pop_heap(heap.begin(), heap.end(), worse);
        Entry top = heap.back();
        heap.pop_back();
        int u = top.node;
        Cost here{top.vis, top.len};
        if (out.dist[u].c < here)
            continue;
        if (left && is_target[u]) {
            is_target[u] = 0;
            if (--left == 0)
                break;
        }
        nbrs(u, [&](const Link& l) {
            Cost nd{here.vis + shown(pool[l.pieces], hidden), here.len + l.len};
            if (!out.dist[l.v].ok || nd < out.dist[l.v].c) {
                out.dist[l.v].ok = true;
                out.dist[l.v].c = nd;
                out.prev[l.v] = l;
                out.prev_u[l.v] = u;
                heap.push_back({nd.vis, nd.len, l.v});
                std::push_heap(heap.begin(), heap.end(), worse);
            }
        });
    }
}

std::vector<Pt> walk_back(const std::vector<Pt>& nodes, const std::vector<std::vector<Pt>>& vias,
                          const SearchOut& s, int dst) {
    std::vector<Pt> path = {nodes[dst]};
    int u = dst;
    while (s.prev_u[u] != -1) {
        const Link& l = s.prev[u];
        if (l.via >= 0) {
            const std::vector<Pt>& via = vias[l.via];
            // The Python extends by reversed(via), via in the link's direction.
            if (!l.rev)
                path.insert(path.end(), via.rbegin(), via.rend());
            else
                path.insert(path.end(), via.begin(), via.end());
        }
        u = s.prev_u[u];
        path.push_back(nodes[u]);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

struct Attachment {
    int node;
    double len;
    int pieces;
};

struct CostKey {
    double x, y;
    int cell;
    std::vector<uint64_t> hidden;
    bool operator==(const CostKey& o) const {
        return x == o.x && y == o.y && cell == o.cell && hidden == o.hidden;
    }
};

struct CostKeyHash {
    size_t operator()(const CostKey& k) const {
        std::hash<double> hd;
        size_t h = hd(k.x) * 1000003u ^ hd(k.y);
        h = h * 1000003u ^ (size_t)k.cell;
        for (uint64_t w : k.hidden)
            h = h * 1000003u ^ std::hash<uint64_t>()(w);
        return h;
    }
};

typedef std::array<OptCost, 4> Row4;

struct Router {
    double tol;
    const std::vector<Fill>& fills;
    size_t ncells;
    std::vector<std::vector<Pt>> region;
    std::vector<Pt> nodes;
    std::vector<std::vector<Link>> adj;     // before contraction
    std::vector<std::vector<Link>> cadj;    // after
    std::vector<std::vector<Link>> folded;  // node -> links toward both ends
    std::vector<char> is_folded;
    std::vector<std::vector<Pt>> vias;
    std::vector<Pieces> pool;
    std::map<std::pair<Pt, Pt>, int, PairLess> piece_ids;
    std::map<Pt, std::vector<Attachment>, PtLess> attached;
    std::map<Pt, std::vector<int>, PtLess> hops;
    std::map<int, std::vector<int>> ways_up, ways_down;
    struct Mid {
        int node, cell, line;
    };
    std::vector<Mid> mids;
    std::vector<Pt> outline_corners;
    struct Box {
        double x0, y0, x1, y1;
    };
    std::vector<Box> boxes;
    struct BoxEdge {
        Pt p, q;
        Box b;
    };
    std::vector<std::vector<BoxEdge>> edges;
    std::set<int> anchors;
    std::unordered_map<CostKey, Row4, CostKeyHash> cost_memo;
    std::map<std::pair<int, int>, double> gaps;
    bool corners_built = false;
    std::vector<Pt> corner_verts;
    std::vector<std::vector<Link>> corner_adj;

    Router(const Decomposition& dec, const std::vector<Fill>& f, double t)
        : tol(t), fills(f), ncells(dec.cells.size()) {
        for (const Cell& c : dec.cells)
            region.push_back(c.outline);
        for (const auto& r : dec.rings)
            for (const Pt& v : r)
                outline_corners.push_back(v);
        for (const auto& ring : region) {
            double x0 = ring[0].x, y0 = ring[0].y, x1 = ring[0].x, y1 = ring[0].y;
            for (const Pt& v : ring) {
                x0 = std::min(x0, v.x); y0 = std::min(y0, v.y);
                x1 = std::max(x1, v.x); y1 = std::max(y1, v.y);
            }
            boxes.push_back({x0 - tol, y0 - tol, x1 + tol, y1 + tol});
            std::vector<BoxEdge> es;
            for (size_t i = 0; i < ring.size(); ++i) {
                const Pt& p = ring[i];
                const Pt& q = ring[(i + 1) % ring.size()];
                es.push_back({p, q, {std::min(p.x, q.x) - tol, std::min(p.y, q.y) - tol,
                                     std::max(p.x, q.x) + tol, std::max(p.y, q.y) + tol}});
            }
            edges.push_back(es);
        }
        std::vector<std::pair<Pt, Pt>> ring_edges;
        for (const auto& r : dec.rings)
            for (size_t i = 0; i < r.size(); ++i)
                ring_edges.push_back({r[i], r[(i + 1) % r.size()]});
        auto on_boundary = [&](const Pt& p) {
            for (const auto& e : ring_edges)
                if (on_segment(p, e.first, e.second, tol))
                    return true;
            return false;
        };

        // Midline of each cell: its line midpoints, chained.
        std::map<int, int> first, last;
        for (size_t c = 0; c < dec.cells.size(); ++c) {
            int prev_node = -1;
            for (size_t k = 0; k < fills[c].lines.size(); ++k) {
                Pt m = mid(fills[c].lines[k].first, fills[c].lines[k].second);
                if (on_boundary(m))
                    continue;
                int node = add_node(m);
                mids.push_back({node, (int)c, (int)k});
                if (prev_node != -1)
                    link(prev_node, node);
                else
                    first[(int)c] = node;
                prev_node = node;
            }
            if (prev_node != -1) {
                last[(int)c] = prev_node;
                anchors.insert(first[(int)c]);
                anchors.insert(last[(int)c]);
            }
        }
        // Portals, and each cell's ways out up and down.
        for (const Portal& p : dec.portals) {
            if (last.count(p.cell_a) && first.count(p.cell_b)) {
                int node = add_node(mid(p.p0, p.p1));
                anchors.insert(node);
                link(last[p.cell_a], node);
                link(node, first[p.cell_b]);
                ways_up[p.cell_a].push_back(node);
                ways_up[p.cell_a].push_back(first[p.cell_b]);
                ways_down[p.cell_b].push_back(node);
                ways_down[p.cell_b].push_back(last[p.cell_a]);
            }
        }
        // Hops: the fill's own step across a portal, hidden by either cell.
        for (const Portal& p : dec.portals) {
            int below = p.cell_a, above = p.cell_b;
            if (!last.count(below) || !first.count(above))
                continue;
            struct Side {
                std::pair<Pt, Pt> ends, line;
                int target;
            };
            Side sides[2] = {{fills[below].lines.back(), fills[above].lines.front(), first[above]},
                             {fills[above].lines.front(), fills[below].lines.back(), last[below]}};
            for (const Side& s : sides) {
                const Pt* ends[2] = {&s.ends.first, &s.ends.second};
                for (const Pt* e : ends) {
                    for (const Pt& q : step_targets(*e, s.line, tol)) {
                        int step = pieces(*e, q);
                        if (step < 0)
                            continue;
                        Bits both(ncells);
                        both.set(below);
                        both.set(above);
                        double total = 0.0;
                        for (const Piece& pc : pool[step]) {
                            total += pc.length;
                            for (size_t i = 0; i < both.w.size(); ++i)
                                both.w[i] |= pc.cells.w[i];
                        }
                        std::pair<Pt, Pt> key = lex_less(q, *e) ? std::make_pair(q, *e) : std::make_pair(*e, q);
                        Pieces one;
                        one.push_back(Piece{total, both});
                        pool.push_back(one);
                        piece_ids[key] = (int)pool.size() - 1;
                        int node = add_node(q);
                        link(node, s.target);
                        hops[*e].push_back(node);
                        anchors.insert(node);
                        break;
                    }
                }
            }
        }
        contract();
    }

    int add_node(const Pt& p) {
        nodes.push_back(p);
        adj.emplace_back();
        return (int)nodes.size() - 1;
    }

    void link(int i, int j) {
        Pt a = nodes[i], b = nodes[j];
        int pid = pieces(a, b);
        if (pid < 0)
            return;
        double len = dist(a, b);
        adj[i].push_back({j, len, pid, -1, false});
        adj[j].push_back({i, len, pid, -1, false});
    }

    void contract() {
        int n = (int)nodes.size();
        std::vector<char> keep(n);
        for (int i = 0; i < n; ++i)
            keep[i] = adj[i].size() != 2 || anchors.count(i);
        cadj.assign(n, {});
        folded.assign(n, {});
        is_folded.assign(n, 0);
        for (int u = 0; u < n; ++u) {
            if (!keep[u])
                continue;
            for (size_t li = 0; li < adj[u].size(); ++li) {
                Link first_link = adj[u][li];
                std::vector<int> run;
                std::vector<double> lengths = {first_link.len};
                std::vector<int> runs = {first_link.pieces};
                int prev = u, cur = first_link.v;
                while (!keep[cur] && std::find(run.begin(), run.end(), cur) == run.end()) {
                    run.push_back(cur);
                    const Link* nxt = nullptr;
                    for (const Link& e : adj[cur])
                        if (e.v != prev) {
                            nxt = &e;
                            break;
                        }
                    if (!nxt)
                        break;
                    prev = cur;
                    cur = nxt->v;
                    lengths.push_back(nxt->len);
                    runs.push_back(nxt->pieces);
                }
                if (!keep[cur])
                    continue;
                std::vector<Pt> via;
                for (int x : run)
                    via.push_back(nodes[x]);
                double total = 0.0;
                for (double l : lengths)
                    total += l;
                int pid = merged(runs, runs.size());
                int vid = -1;
                if (!via.empty()) {
                    vias.push_back(via);
                    vid = (int)vias.size() - 1;
                }
                cadj[u].push_back({cur, total, pid, vid, false});
                for (size_t i = 0; i < run.size(); ++i) {
                    double part = 0.0;
                    for (size_t k = 0; k <= i; ++k)
                        part += lengths[k];
                    int spid = merged(runs, i + 1);
                    std::vector<Pt> back;
                    for (size_t k = i; k-- > 0;)
                        back.push_back(nodes[run[k]]);
                    int bvid = -1;
                    if (!back.empty()) {
                        vias.push_back(back);
                        bvid = (int)vias.size() - 1;
                    }
                    folded[run[i]].push_back({u, part, spid, bvid, false});
                    is_folded[run[i]] = 1;
                }
            }
        }
    }

    int merged(const std::vector<int>& runs, size_t count) {
        std::vector<const Pieces*> ptrs;
        for (size_t i = 0; i < count; ++i)
            ptrs.push_back(&pool[runs[i]]);
        Pieces m = merge_pieces(ptrs);
        pool.push_back(m);
        return (int)pool.size() - 1;
    }

    // Pieces of segment ab, as an index into pool, or -1 if it leaves the
    // region or runs through a corner; remembered either way round.
    int pieces(const Pt& a, const Pt& b) {
        std::pair<Pt, Pt> key = lex_less(b, a) ? std::make_pair(b, a) : std::make_pair(a, b);
        auto it = piece_ids.find(key);
        if (it != piece_ids.end())
            return it->second;
        int id = -1;
        Pieces out;
        if (cut_into_pieces(key.first, key.second, out)) {
            pool.push_back(out);
            id = (int)pool.size() - 1;
        }
        piece_ids[key] = id;
        return id;
    }

    bool contains(size_t c, const Pt& p) const {
        if (point_in_ring(p, region[c]))
            return true;
        for (const BoxEdge& e : edges[c])
            if (e.b.x0 <= p.x && p.x <= e.b.x1 && e.b.y0 <= p.y && p.y <= e.b.y1 &&
                on_segment(p, e.p, e.q, tol))
                return true;
        return false;
    }

    Bits cells_at(const Pt& p) const {
        Bits out(ncells);
        for (size_t c = 0; c < boxes.size(); ++c) {
            const Box& b = boxes[c];
            if (b.x0 <= p.x && p.x <= b.x1 && b.y0 <= p.y && p.y <= b.y1 && contains(c, p))
                out.set((int)c);
        }
        return out;
    }

    bool cut_into_pieces(const Pt& a, const Pt& b, Pieces& out) const {
        if (!cells_at(a).any() || !cells_at(b).any())
            return false;
        double bx0 = std::min(a.x, b.x) - tol, by0 = std::min(a.y, b.y) - tol;
        double bx1 = std::max(a.x, b.x) + tol, by1 = std::max(a.y, b.y) + tol;
        for (const Pt& v : outline_corners)
            if (bx0 <= v.x && v.x <= bx1 && by0 <= v.y && v.y <= by1 && !same(v, a, tol) &&
                !same(v, b, tol) && on_segment(v, a, b, tol))
                return false;
        double dx = b.x - a.x, dy = b.y - a.y;
        double L = std::hypot(dx, dy);
        if (L <= tol)
            return true;  // no pieces
        std::vector<std::pair<Pt, Pt>> near;
        for (size_t c = 0; c < boxes.size(); ++c) {
            const Box& bb = boxes[c];
            if (bb.x0 <= bx1 && bx0 <= bb.x1 && bb.y0 <= by1 && by0 <= bb.y1)
                for (const BoxEdge& e : edges[c])
                    if (e.b.x0 <= bx1 && bx0 <= e.b.x1 && e.b.y0 <= by1 && by0 <= e.b.y1)
                        near.push_back({e.p, e.q});
        }
        std::vector<double> ts = cut_params(a, b, near, tol);
        for (size_t i = 0; i + 1 < ts.size(); ++i) {
            double piece = (ts[i + 1] - ts[i]) * L;
            if (piece <= tol)
                continue;
            double tm = 0.5 * (ts[i] + ts[i + 1]);
            Bits cells = cells_at({a.x + tm * dx, a.y + tm * dy});
            if (!cells.any())
                return false;
            out.push_back({piece, cells});
        }
        return true;
    }

    double visible(const std::vector<Pt>& path, const Bits& hidden) {
        double s = 0.0;
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            int pid = pieces(path[i], path[i + 1]);
            if (pid >= 0)
                s += shown(pool[pid], hidden);
        }
        return s;
    }

    const std::vector<Attachment>& attach(const Pt& p) {
        auto it = attached.find(p);
        if (it != attached.end())
            return it->second;
        std::vector<Attachment> found = find_attachments(p);
        return attached[p] = found;
    }

    std::vector<Attachment> find_attachments(const Pt& p) {
        std::vector<int> targets;
        auto h = hops.find(p);
        if (h != hops.end())
            targets = h->second;
        for (size_t c = 0; c < fills.size(); ++c) {
            const auto& lines = fills[c].lines;
            for (size_t k = 0; k < lines.size(); ++k) {
                if (!on_segment(p, lines[k].first, lines[k].second, tol))
                    continue;
                if (k == lines.size() - 1) {
                    auto w = ways_up.find((int)c);
                    if (w != ways_up.end())
                        targets.insert(targets.end(), w->second.begin(), w->second.end());
                }
                if (k == 0) {
                    auto w = ways_down.find((int)c);
                    if (w != ways_down.end())
                        targets.insert(targets.end(), w->second.begin(), w->second.end());
                }
            }
        }
        for (const Mid& m : mids) {
            const auto& line = fills[m.cell].lines[m.line];
            if (on_segment(p, line.first, line.second, tol))
                targets.push_back(m.node);
        }
        std::vector<Attachment> out;
        std::set<int> seen;
        for (int node : targets) {
            if (!seen.insert(node).second)
                continue;
            int pid = pieces(p, nodes[node]);
            if (pid >= 0)
                out.push_back({node, dist(p, nodes[node]), pid});
        }
        if (!out.empty())
            return out;
        std::vector<Mid> order = mids;
        std::stable_sort(order.begin(), order.end(), [&](const Mid& a, const Mid& b) {
            return dist(p, nodes[a.node]) < dist(p, nodes[b.node]);
        });
        for (const Mid& m : order) {
            int pid = pieces(p, nodes[m.node]);
            if (pid >= 0) {
                out.push_back({m.node, dist(p, nodes[m.node]), pid});
                return out;
            }
        }
        return out;
    }

    // route: the cheapest travel from start to each goal. out_cost[g].ok is
    // false where the goal cannot be reached; paths are filled only when
    // out_paths is given.
    void route(const Pt& start, const std::vector<Pt>& goals, const Bits& hidden,
               std::vector<OptCost>& out_cost, std::vector<std::vector<Pt>>* out_paths) {
        int n0 = (int)nodes.size();
        int n = n0 + 1 + (int)goals.size();
        std::vector<std::vector<Link>> extra(n);
        std::vector<char> unfolded(n0, 0);
        auto unfold = [&](int x) {
            if (x < n0 && is_folded[x] && !unfolded[x]) {
                unfolded[x] = 1;
                for (const Link& l : folded[x]) {
                    extra[x].push_back(l);
                    extra[l.v].push_back({x, l.len, l.pieces, l.via, !l.rev});
                }
            }
        };
        std::vector<Attachment> at_start = attach(start);
        for (const Attachment& a : at_start) {
            unfold(a.node);
            extra[n0].push_back({a.node, a.len, a.pieces, -1, false});
            extra[a.node].push_back({n0, a.len, a.pieces, -1, false});
        }
        for (size_t g = 0; g < goals.size(); ++g) {
            std::vector<Attachment> at_goal = attach(goals[g]);
            for (const Attachment& a : at_goal) {
                unfold(a.node);
                extra[a.node].push_back({n0 + 1 + (int)g, a.len, a.pieces, -1, false});
            }
        }
        std::vector<int> targets;
        for (size_t g = 0; g < goals.size(); ++g)
            targets.push_back(n0 + 1 + (int)g);
        SearchOut s;
        auto nbrs = [&](int u, const std::function<void(const Link&)>& f) {
            if (u < n0)
                for (const Link& l : cadj[u])
                    f(l);
            for (const Link& l : extra[u])
                f(l);
        };
        search(n, nbrs, pool, n0, hidden, targets, s);
        out_cost.assign(goals.size(), OptCost());
        if (out_paths)
            out_paths->assign(goals.size(), {});
        std::vector<Pt> all;
        if (out_paths) {
            all = nodes;
            all.push_back(start);
            all.insert(all.end(), goals.begin(), goals.end());
        }
        for (size_t g = 0; g < goals.size(); ++g) {
            int t = n0 + 1 + (int)g;
            if (same(start, goals[g], tol)) {
                out_cost[g].ok = true;
                out_cost[g].c = {0.0, 0.0};
                if (out_paths)
                    (*out_paths)[g] = {start, goals[g]};
            } else if (s.dist[t].ok) {
                out_cost[g] = s.dist[t];
                if (out_paths)
                    (*out_paths)[g] = walk_back(all, vias, s, t);
            } else {
                std::vector<Pt> path;
                out_cost[g] = corner_route(start, goals[g], hidden, path);
                if (out_paths)
                    (*out_paths)[g] = path;
            }
        }
    }

    // costs from start to the four entries of cell, remembered.
    const Row4& costs(const Pt& start, int cell, const Bits& hidden) {
        CostKey key{start.x + 0.0, start.y + 0.0, cell, hidden.w};  // -0.0 hashes apart from 0.0
        auto it = cost_memo.find(key);
        if (it != cost_memo.end())
            return it->second;
        std::vector<Pt> goals;
        for (int d = 0; d < 4; ++d)
            goals.push_back(fills[cell].walks[d].front());
        std::vector<OptCost> out;
        route(start, goals, hidden, out, nullptr);
        Row4 arr = {out[0], out[1], out[2], out[3]};
        return cost_memo[key] = arr;
    }

    OptCost corner_route(const Pt& start, const Pt& goal, const Bits& hidden, std::vector<Pt>& path) {
        if (!corners_built) {
            corners_built = true;
            for (const auto& ring : region)
                for (const Pt& v : ring) {
                    bool dup = false;
                    for (const Pt& w : corner_verts)
                        if (same(v, w, tol)) {
                            dup = true;
                            break;
                        }
                    if (!dup)
                        corner_verts.push_back(v);
                }
            corner_adj.assign(corner_verts.size(), {});
            for (size_t i = 0; i < corner_verts.size(); ++i)
                for (size_t j = i + 1; j < corner_verts.size(); ++j) {
                    int pid = pieces(corner_verts[i], corner_verts[j]);
                    if (pid >= 0) {
                        double len = dist(corner_verts[i], corner_verts[j]);
                        corner_adj[i].push_back({(int)j, len, pid, -1, false});
                        corner_adj[j].push_back({(int)i, len, pid, -1, false});
                    }
                }
        }
        int nv = (int)corner_verts.size();
        std::vector<Pt> all = corner_verts;
        all.push_back(start);
        all.push_back(goal);
        int s = nv, g = nv + 1;
        std::vector<std::vector<Link>> extra(nv + 2);
        const int ends[2] = {s, g};
        for (int end : ends)
            for (int i = 0; i < nv; ++i) {
                int pid = pieces(all[end], corner_verts[i]);
                if (pid >= 0) {
                    double len = dist(all[end], corner_verts[i]);
                    extra[end].push_back({i, len, pid, -1, false});
                    extra[i].push_back({end, len, pid, -1, false});
                }
            }
        int direct = pieces(start, goal);
        if (direct >= 0)
            extra[s].push_back({g, dist(start, goal), direct, -1, false});
        SearchOut so;
        auto nbrs = [&](int u, const std::function<void(const Link&)>& f) {
            if (u < nv)
                for (const Link& l : corner_adj[u])
                    f(l);
            for (const Link& l : extra[u])
                f(l);
        };
        search(nv + 2, nbrs, pool, s, hidden, std::vector<int>(), so);
        if (!so.dist[g].ok)
            return OptCost();
        path = walk_back(all, vias, so, g);
        return so.dist[g];
    }

    std::vector<Pt> shortcut(const std::vector<Pt>& path, const Bits& hidden) {
        std::vector<double> sh = {0.0};
        for (size_t i = 0; i + 1 < path.size(); ++i) {
            int pid = pieces(path[i], path[i + 1]);
            sh.push_back(sh.back() + (pid >= 0 ? shown(pool[pid], hidden) : 0.0));
        }
        std::vector<Pt> out = {path[0]};
        size_t i = 0;
        while (i + 1 < path.size()) {
            size_t nxt = i + 1;
            double hop = dist(path[i], path[nxt]);
            for (size_t j = path.size() - 1; j > i + 1; --j) {
                if (dist(path[i], path[j]) >= hop)
                    continue;
                int pid = pieces(path[i], path[j]);
                if (pid >= 0 && shown(pool[pid], hidden) <= sh[j] - sh[i] + tol) {
                    nxt = j;
                    break;
                }
            }
            if (!same(path[nxt], out.back(), tol) || nxt == path.size() - 1)
                out.push_back(path[nxt]);
            i = nxt;
        }
        return out;
    }
};

// --- the order ---------------------------------------------------------------

// Python's random.Random(0) and its sample(): the kicks must shake the order
// exactly as the reference does, or the two could not be compared.
struct PyRandom {
    uint32_t mt[624];
    int mti;
    explicit PyRandom(uint32_t seed) {
        // init_by_array with key [seed], as random.seed(int) does.
        init_genrand(19650218u);
        uint32_t key[1] = {seed};
        int i = 1, j = 0, k = 624;
        for (; k; --k) {
            mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1664525u)) + key[j] + (uint32_t)j;
            ++i;
            ++j;
            if (i >= 624) {
                mt[0] = mt[623];
                i = 1;
            }
            if (j >= 1)
                j = 0;
        }
        for (k = 623; k; --k) {
            mt[i] = (mt[i] ^ ((mt[i - 1] ^ (mt[i - 1] >> 30)) * 1566083941u)) - (uint32_t)i;
            ++i;
            if (i >= 624) {
                mt[0] = mt[623];
                i = 1;
            }
        }
        mt[0] = 0x80000000u;
        mti = 624;
    }
    void init_genrand(uint32_t s) {
        mt[0] = s;
        for (mti = 1; mti < 624; ++mti)
            mt[mti] = 1812433253u * (mt[mti - 1] ^ (mt[mti - 1] >> 30)) + (uint32_t)mti;
    }
    uint32_t genrand() {
        static const uint32_t mag01[2] = {0u, 0x9908b0dfu};
        uint32_t y;
        if (mti >= 624) {
            int kk;
            for (kk = 0; kk < 624 - 397; ++kk) {
                y = (mt[kk] & 0x80000000u) | (mt[kk + 1] & 0x7fffffffu);
                mt[kk] = mt[kk + 397] ^ (y >> 1) ^ mag01[y & 1u];
            }
            for (; kk < 623; ++kk) {
                y = (mt[kk] & 0x80000000u) | (mt[kk + 1] & 0x7fffffffu);
                mt[kk] = mt[kk + (397 - 624)] ^ (y >> 1) ^ mag01[y & 1u];
            }
            y = (mt[623] & 0x80000000u) | (mt[0] & 0x7fffffffu);
            mt[623] = mt[396] ^ (y >> 1) ^ mag01[y & 1u];
            mti = 0;
        }
        y = mt[mti++];
        y ^= (y >> 11);
        y ^= (y << 7) & 0x9d2c5680u;
        y ^= (y << 15) & 0xefc60000u;
        y ^= (y >> 18);
        return y;
    }
    int randbelow(int n) {  // _randbelow_with_getrandbits, n < 2**31
        int k = 0;
        while ((1LL << k) <= n)
            ++k;  // n.bit_length()
        uint32_t r = genrand() >> (32 - k);
        while ((long long)r >= n)
            r = genrand() >> (32 - k);
        return (int)r;
    }
    // sample(range(lo, lo + n), 3)
    std::vector<int> sample3(int lo, int n) {
        std::vector<int> result(3);
        if (n <= 21) {
            std::vector<int> pool(n);
            for (int i = 0; i < n; ++i)
                pool[i] = lo + i;
            for (int i = 0; i < 3; ++i) {
                int j = randbelow(n - i);
                result[i] = pool[j];
                pool[j] = pool[n - i - 1];
            }
        } else {
            std::set<int> selected;
            for (int i = 0; i < 3; ++i) {
                int j = randbelow(n);
                while (selected.count(j))
                    j = randbelow(n);
                selected.insert(j);
                result[i] = lo + j;
            }
        }
        return result;
    }
};

struct TableRow {
    Row4 cost;
    std::array<int, 4> came_from;
};

inline bool better(const Cost& a, const Cost& b, double tol) {
    return a.vis < b.vis - tol || (a.vis <= b.vis + tol && a.len < b.len - tol);
}

struct Planner {
    Router& router;
    const std::vector<Fill>& fills;
    const Pt* start;
    double tol;

    Bits bits_of(const std::vector<int>& cells, size_t from) const {
        Bits b(router.ncells);
        for (size_t i = from; i < cells.size(); ++i)
            b.set(cells[i]);
        return b;
    }

    // _row; cut_best/cut_rest as the Python's cut.
    TableRow row(const std::vector<int>& cells, size_t k, const Row4* prev, const Cost* cut_best,
                 const Cost* cut_rest) {
        TableRow out;
        out.came_from = {0, 0, 0, 0};
        if (k == 0) {
            if (!start) {
                for (int d = 0; d < 4; ++d) {
                    out.cost[d].ok = true;
                    out.cost[d].c = {0.0, 0.0};
                }
            } else {
                out.cost = router.costs(*start, cells[0], bits_of(cells, 0));
            }
            return out;
        }
        Bits hidden = bits_of(cells, k);
        for (int dp = 0; dp < 4; ++dp) {
            if (!(*prev)[dp].ok)
                continue;
            if (cut_best) {
                Cost bound = (*prev)[dp].c + *cut_rest;
                if (!better(bound, *cut_best, tol))
                    continue;
            }
            Row4 step = router.costs(fills[cells[k - 1]].walks[dp].back(), cells[k], hidden);
            for (int d = 0; d < 4; ++d) {
                if (!step[d].ok)
                    continue;
                Cost total = (*prev)[dp].c + step[d].c;
                if (!out.cost[d].ok || total < out.cost[d].c) {
                    out.cost[d].ok = true;
                    out.cost[d].c = total;
                    out.came_from[d] = dp;
                }
            }
        }
        return out;
    }

    std::vector<TableRow> forward(const std::vector<int>& cells, const std::vector<TableRow>* table = nullptr,
                                  size_t lo = 0) {
        std::vector<TableRow> out;
        if (table)
            out.assign(table->begin(), table->begin() + std::min(lo, table->size()));
        for (size_t k = out.size(); k < cells.size(); ++k) {
            Row4 prev;
            if (k)
                prev = out[k - 1].cost;
            out.push_back(row(cells, k, k ? &prev : nullptr, nullptr, nullptr));
        }
        return out;
    }

    std::vector<Row4> backward(const std::vector<int>& cells) {
        size_t n = cells.size();
        std::vector<Row4> back(n);
        for (int d = 0; d < 4; ++d) {
            back[n - 1][d].ok = true;
            back[n - 1][d].c = {0.0, 0.0};
        }
        for (size_t k = n - 1; k-- > 0;) {
            Bits hidden = bits_of(cells, k + 1);
            for (int d = 0; d < 4; ++d) {
                Row4 step = router.costs(fills[cells[k]].walks[d].back(), cells[k + 1], hidden);
                OptCost best;
                for (int dn = 0; dn < 4; ++dn) {
                    if (!step[dn].ok || !back[k + 1][dn].ok)
                        continue;
                    Cost total = step[dn].c + back[k + 1][dn].c;
                    if (!best.ok || total < best.c) {
                        best.ok = true;
                        best.c = total;
                    }
                }
                back[k][d] = best;
            }
        }
        return back;
    }

    static OptCost lowest(const Row4& costs) {
        OptCost out;
        for (const OptCost& c : costs)
            if (c.ok && (!out.ok || c.c < out.c))
                out = c;
        return out;
    }

    static OptCost floor_of(const Row4& costs) {
        OptCost out;
        for (const OptCost& c : costs) {
            if (!c.ok)
                continue;
            if (!out.ok) {
                out = c;
            } else {
                out.c.vis = std::min(out.c.vis, c.c.vis);
                out.c.len = std::min(out.c.len, c.c.len);
            }
        }
        return out;
    }

    double gap(int a, int b) {
        auto key = std::make_pair(a, b);
        auto it = router.gaps.find(key);
        if (it != router.gaps.end())
            return it->second;
        Bits everything(router.ncells);
        for (size_t c = 0; c < router.ncells; ++c)
            everything.set((int)c);
        double found = std::numeric_limits<double>::infinity();
        for (int d = 0; d < 4; ++d) {
            Pt exit_ = fills[a].walks[d].back();
            Row4 cs = router.costs(exit_, b, everything);
            for (int e = 0; e < 4; ++e) {
                double v = cs[e].ok ? cs[e].c.len : dist(exit_, fills[b].walks[e].front());
                found = std::min(found, v);
            }
        }
        router.gaps[key] = found;
        return found;
    }

    // _best_walks: min over (cost, walk), back through came_from.
    bool best_walks(const std::vector<TableRow>& table, Cost& best, std::vector<int>& walks) {
        int d = -1;
        for (int e = 0; e < 4; ++e) {
            const OptCost& c = table.back().cost[e];
            if (c.ok && (d < 0 || c.c < best)) {
                best = c.c;
                d = e;
            }
        }
        if (d < 0)
            return false;
        walks.assign(table.size(), 0);
        walks.back() = d;
        for (size_t k = table.size() - 1; k > 0; --k) {
            d = table[k].came_from[d];
            walks[k - 1] = d;
        }
        return true;
    }

    OptCost spliced_cost(const std::vector<int>& cells, const std::vector<int>& cand,
                         const std::vector<TableRow>& table, const std::vector<Row4>& back, size_t lo,
                         const OptCost& best) {
        int n = (int)cand.size();
        int hi = n - 1;
        while (hi >= (int)lo && cand[hi] == cells[hi])
            --hi;
        if (hi < (int)lo)
            return OptCost();
        int last = std::min(hi + 1, n - 1);
        OptCost finish;
        if (hi + 1 < n) {
            finish = floor_of(back[hi + 1]);
        } else {
            finish.ok = true;
            finish.c = {0.0, 0.0};
        }
        if (!finish.ok)
            return OptCost();
        double run = 0.0;
        std::vector<double> floors(last + 1, 0.0);
        for (int k = last; k >= (int)lo; --k) {
            floors[k] = run;
            if (k >= 1)
                run += gap(cand[k - 1], cand[k]);
        }
        if (best.ok) {
            OptCost head;
            if (lo == 0) {
                head.ok = true;
                head.c = {0.0, 0.0};
                if (start) {
                    double m = std::numeric_limits<double>::infinity();
                    for (int d = 0; d < 4; ++d)
                        m = std::min(m, dist(*start, fills[cand[0]].walks[d].front()));
                    head.c = {0.0, m};
                }
            } else {
                head = floor_of(table[lo - 1].cost);
            }
            if (head.ok && !better({head.c.vis + finish.c.vis, head.c.len + run + finish.c.len}, best.c, tol))
                return OptCost();
        }
        Row4 prev;
        bool have_prev = lo > 0;
        if (have_prev)
            prev = table[lo - 1].cost;
        for (int k = (int)lo; k <= last; ++k) {
            Cost rest{0.0, 0.0};
            const Cost* cb = nullptr;
            const Cost* cr = nullptr;
            if (best.ok && k >= 1) {
                rest = {finish.c.vis, gap(cand[k - 1], cand[k]) + floors[k] + finish.c.len};
                cb = &best.c;
                cr = &rest;
            }
            TableRow r = row(cand, (size_t)k, have_prev ? &prev : nullptr, cb, cr);
            OptCost low = floor_of(r.cost);
            if (!low.ok)
                return OptCost();
            if (best.ok && k < last &&
                !better({low.c.vis + finish.c.vis, low.c.len + floors[k] + finish.c.len}, best.c, tol))
                return OptCost();
            prev = r.cost;
            have_prev = true;
        }
        if (hi + 1 >= n)
            return lowest(prev);
        OptCost out;
        for (int d = 0; d < 4; ++d) {
            if (!prev[d].ok || !back[hi + 1][d].ok)
                continue;
            Cost e = prev[d].c + back[hi + 1][d].c;
            if (!out.ok || e < out.c) {
                out.ok = true;
                out.c = e;
            }
        }
        return out;
    }

    static std::vector<std::pair<size_t, std::vector<int>>> rearrangements(const std::vector<int>& cells) {
        std::vector<std::pair<size_t, std::vector<int>>> out;
        size_t n = cells.size();
        for (size_t size = 1; size <= 3; ++size) {
            for (size_t i = 0; i + size <= n; ++i) {
                std::vector<int> block(cells.begin() + i, cells.begin() + i + size);
                std::vector<int> rest(cells.begin(), cells.begin() + i);
                rest.insert(rest.end(), cells.begin() + i + size, cells.end());
                for (int flip = 0; flip < (size > 1 ? 2 : 1); ++flip) {
                    std::vector<int> run = block;
                    if (flip)
                        std::reverse(run.begin(), run.end());
                    for (size_t j = 0; j <= rest.size(); ++j) {
                        if (j == i && !flip)
                            continue;
                        std::vector<int> cand(rest.begin(), rest.begin() + j);
                        cand.insert(cand.end(), run.begin(), run.end());
                        cand.insert(cand.end(), rest.begin() + j, rest.end());
                        out.push_back({std::min(i, j), cand});
                    }
                }
            }
        }
        for (size_t i = 0; i < n; ++i)
            for (size_t j = i + 2; j < n; ++j) {
                std::vector<int> cand = cells;
                std::swap(cand[i], cand[j]);
                out.push_back({i, cand});
            }
        for (size_t i = 0; i < n; ++i)
            for (size_t j = i + 4; j <= n; ++j) {
                std::vector<int> cand = cells;
                std::reverse(cand.begin() + i, cand.begin() + j);
                out.push_back({i, cand});
            }
        return out;
    }

    OptCost descend(std::vector<int>& cells, std::vector<int>& walks) {
        std::vector<TableRow> table = forward(cells);
        OptCost best;
        best.ok = best_walks(table, best.c, walks);
        std::vector<Row4> back = backward(cells);
        auto moves = rearrangements(cells);
        size_t k = 0, quiet = 0;
        while (!moves.empty() && quiet < moves.size()) {
            size_t lo = moves[k % moves.size()].first;
            const std::vector<int>& cand = moves[k % moves.size()].second;
            OptCost cost = spliced_cost(cells, cand, table, back, lo, best);
            if (cost.ok && (!best.ok || better(cost.c, best.c, tol))) {
                cells = cand;
                table = forward(cells, &table, lo);
                best.ok = best_walks(table, best.c, walks);
                back = backward(cells);
                moves = rearrangements(cells);
                quiet = 0;
            } else {
                ++quiet;
            }
            ++k;
        }
        return best;
    }

    void improve(std::vector<int>& cells, std::vector<int>& walks, int kicks) {
        OptCost best = descend(cells, walks);
        if (cells.size() < 4)
            return;
        PyRandom rng(0);
        for (int t = 0; t < kicks; ++t) {
            std::vector<int> pqr = rng.sample3(1, (int)cells.size() - 1);
            std::sort(pqr.begin(), pqr.end());
            size_t p = pqr[0], q = pqr[1], r = pqr[2];
            std::vector<int> cand(cells.begin(), cells.begin() + p);
            cand.insert(cand.end(), cells.begin() + q, cells.begin() + r);
            cand.insert(cand.end(), cells.begin() + p, cells.begin() + q);
            cand.insert(cand.end(), cells.begin() + r, cells.end());
            std::vector<int> cand_walks;
            OptCost cost = descend(cand, cand_walks);
            if (cost.ok && (!best.ok || better(cost.c, best.c, tol))) {
                cells = cand;
                walks = cand_walks;
                best = cost;
            }
        }
    }

    // _greedy's order: from where the needle is, the cheapest next walk.
    std::vector<int> greedy(const std::vector<int>& todo, int first_cell) {
        std::vector<int> order;
        std::set<int> remaining(todo.begin(), todo.end());
        Pt pos = start ? *start : Pt{0.0, 0.0};
        if (first_cell >= 0) {
            order.push_back(first_cell);
            remaining.erase(first_cell);
            pos = fills[first_cell].walks[0].back();
        }
        while (!remaining.empty()) {
            Bits hidden(router.ncells);
            for (int c : remaining)
                hidden.set(c);
            std::vector<std::pair<int, int>> cands;
            std::vector<Pt> goals;
            for (int c : remaining)
                for (int d = 0; d < 4; ++d) {
                    cands.push_back({c, d});
                    goals.push_back(fills[c].walks[d].front());
                }
            std::vector<OptCost> cs;
            router.route(pos, goals, hidden, cs, nullptr);
            int pick = -1;
            for (size_t k = 0; k < cs.size(); ++k)
                if (cs[k].ok && (pick < 0 || cs[k].c < cs[pick].c))
                    pick = (int)k;
            if (pick < 0)
                throw Error{CP_ERR_PIECES};
            int c = cands[pick].first, d = cands[pick].second;
            order.push_back(c);
            remaining.erase(c);
            pos = fills[c].walks[d].back();
        }
        return order;
    }
};

struct PlanPiece {
    int kind;  // 0 fill, 1 travel
    std::vector<Pt> points;
    int cell, direction;
    double visible;
};

struct PlanOut {
    std::vector<std::pair<int, int>> order;
    std::vector<PlanPiece> pieces;
};

PlanOut plan(const Decomposition& dec, const std::vector<Fill>& fills, const Pt* start, int kicks, double tol) {
    if (kicks < 0)
        throw Error{CP_ERR_KICKS};
    if (tol < 0.0)
        throw Error{CP_ERR_NEGATIVE_TOL};
    Router router(dec, fills, tol);
    std::vector<int> todo;
    for (size_t c = 0; c < dec.cells.size(); ++c)
        if (!fills[c].lines.empty())
            todo.push_back((int)c);
    PlanOut out;
    if (todo.empty())
        return out;
    Planner pl{router, fills, start, tol};
    std::vector<int> cells = pl.greedy(todo, start ? -1 : todo[0]);
    std::vector<int> walks;
    pl.improve(cells, walks, kicks);

    // _assemble
    bool have_pos = start != nullptr;
    Pt pos = start ? *start : Pt{0.0, 0.0};
    for (size_t i = 0; i < cells.size(); ++i) {
        int c = cells[i], d = walks[i];
        const std::vector<Pt>& walk = fills[c].walks[d];
        if (have_pos) {
            Bits hidden = pl.bits_of(cells, i);
            std::vector<OptCost> cs;
            std::vector<std::vector<Pt>> paths;
            std::vector<Pt> goal = {walk.front()};
            router.route(pos, goal, hidden, cs, &paths);
            if (!paths[0].empty()) {
                std::vector<Pt> path = router.shortcut(paths[0], hidden);
                if (!same(path.front(), path.back(), router.tol))
                    out.pieces.push_back({1, path, -1, -1, router.visible(path, hidden)});
            }
        }
        out.order.push_back({c, d});
        out.pieces.push_back({0, walk, c, d, 0.0});
        pos = walk.back();
        have_pos = true;
    }
    return out;
}

// --- results -----------------------------------------------------------------

// Field numbers, shared with native_bridge.py and, later, the C# wrapper.
enum Field {
    F_CELL_COUNTS = 0,   // int, per cell: outline vertex count
    F_CELL_POINTS = 1,   // double, x y per outline vertex, cell after cell
    F_LINE_COUNTS = 2,   // int, per cell: fill line count
    F_LINE_POINTS = 3,   // double, x0 y0 x1 y1 per line (left end, right end)
    F_WALK_COUNTS = 4,   // int, 4 per cell: point count of each walk
    F_WALK_POINTS = 5,   // double, x y per point, walk after walk, cell after cell
    // coverage_plan only:
    F_ORDER = 6,         // int, cell then walk, per cell in the order stitched
    F_PIECE_KINDS = 7,   // int, per piece: 0 fill, 1 travel
    F_PIECE_COUNTS = 8,  // int, per piece: point count
    F_PIECE_POINTS = 9,  // double, x y per point, piece after piece
    F_PIECE_VISIBLE = 10,  // double, per piece: travel no stitching covers
    F_COUNT = 11
};

struct Result {
    std::vector<int> ints[F_COUNT];
    std::vector<double> doubles[F_COUNT];
};

bool is_int_field(int f) {
    return f == F_CELL_COUNTS || f == F_LINE_COUNTS || f == F_WALK_COUNTS || f == F_ORDER ||
           f == F_PIECE_KINDS || f == F_PIECE_COUNTS;
}

void put(std::vector<double>& out, const Pt& p) {
    out.push_back(p.x);
    out.push_back(p.y);
}

Result* hatch_result(const Decomposition& dec, const std::vector<Fill>& fills) {
    Result* r = new Result();
    for (const Cell& c : dec.cells) {
        r->ints[F_CELL_COUNTS].push_back((int)c.outline.size());
        for (const Pt& p : c.outline)
            put(r->doubles[F_CELL_POINTS], p);
    }
    for (const Fill& f : fills) {
        r->ints[F_LINE_COUNTS].push_back((int)f.lines.size());
        for (const auto& ln : f.lines) {
            put(r->doubles[F_LINE_POINTS], ln.first);
            put(r->doubles[F_LINE_POINTS], ln.second);
        }
        for (int d = 0; d < 4; ++d) {
            r->ints[F_WALK_COUNTS].push_back((int)f.walks[d].size());
            for (const Pt& p : f.walks[d])
                put(r->doubles[F_WALK_POINTS], p);
        }
    }
    return r;
}

void add_plan(Result* r, const PlanOut& p) {
    for (const auto& o : p.order) {
        r->ints[F_ORDER].push_back(o.first);
        r->ints[F_ORDER].push_back(o.second);
    }
    for (const PlanPiece& pc : p.pieces) {
        r->ints[F_PIECE_KINDS].push_back(pc.kind);
        r->ints[F_PIECE_COUNTS].push_back((int)pc.points.size());
        for (const Pt& q : pc.points)
            put(r->doubles[F_PIECE_POINTS], q);
        r->doubles[F_PIECE_VISIBLE].push_back(pc.visible);
    }
}

bool read_rings(const double* xy, const int* counts, int num_rings, std::vector<std::vector<Pt>>& rings) {
    if (xy == nullptr || counts == nullptr || num_rings < 1)
        return false;
    size_t at = 0;
    for (int i = 0; i < num_rings; ++i) {
        if (counts[i] < 0)
            return false;
        std::vector<Pt> ring;
        for (int k = 0; k < counts[i]; ++k, ++at)
            ring.push_back({xy[2 * at], xy[2 * at + 1]});
        rings.push_back(ring);
    }
    return true;
}

}  // namespace

extern "C" {

// Cut a region into monotone cells and hatch each: coverage_path.hatch_region.
//   ring_xy      x y of every ring vertex, ring after ring (2 * sum(ring_counts))
//   ring_counts  vertex count of each ring; a closing vertex repeated is fine
//   num_rings    at least 1; even-odd rule, so holes need no marking
//   spacing, angle (radians), phase, link (0 zig-zag, 1 one-way), tol
//   out_handle   receives the result, to read with coverage_result_* and
//                release with coverage_result_free; untouched on failure
// Returns CP_OK or a negative CP_ERR_* code.
DLL_EXPORT int coverage_hatch_region(const double* ring_xy, const int* ring_counts, int num_rings,
                                     double spacing, double angle, double phase, int link, double tol,
                                     void** out_handle) {
    if (out_handle == nullptr)
        return CP_ERR_BAD_ARGUMENT;
    try {
        std::vector<std::vector<Pt>> rings;
        if (!read_rings(ring_xy, ring_counts, num_rings, rings))
            return CP_ERR_BAD_ARGUMENT;
        Decomposition dec = decompose(rings, angle, tol);
        std::vector<Fill> fills = fill_cells(dec, spacing, phase, link, tol);
        *out_handle = hatch_result(dec, fills);
        return CP_OK;
    } catch (const Error& e) {
        return e.code;
    } catch (...) {
        return CP_ERR_INTERNAL;
    }
}

// The whole stitch path over a region: coverage_path.plan after hatch_region.
// Arguments as coverage_hatch_region, plus
//   start  x y where the needle starts, or null to let the plan choose
//   kicks  restarts of the order search; see coverage_path.plan
// The result holds the hatch fields and the plan's.
DLL_EXPORT int coverage_plan(const double* ring_xy, const int* ring_counts, int num_rings,
                             double spacing, double angle, double phase, int link,
                             const double* start, int kicks, double tol, void** out_handle) {
    if (out_handle == nullptr)
        return CP_ERR_BAD_ARGUMENT;
    try {
        std::vector<std::vector<Pt>> rings;
        if (!read_rings(ring_xy, ring_counts, num_rings, rings))
            return CP_ERR_BAD_ARGUMENT;
        Decomposition dec = decompose(rings, angle, tol);
        std::vector<Fill> fills = fill_cells(dec, spacing, phase, link, tol);
        Pt s{0.0, 0.0};
        if (start)
            s = {start[0], start[1]};
        PlanOut p = plan(dec, fills, start ? &s : nullptr, kicks, tol);
        Result* r = hatch_result(dec, fills);
        add_plan(r, p);
        *out_handle = r;
        return CP_OK;
    } catch (const Error& e) {
        return e.code;
    } catch (...) {
        return CP_ERR_INTERNAL;
    }
}

// Number of values in one field of a result: ints or doubles, as the field
// holds. -1 for a null handle or an unknown field.
DLL_EXPORT int coverage_result_size(const void* handle, int field) {
    if (handle == nullptr || field < 0 || field >= F_COUNT)
        return -1;
    const Result* r = static_cast<const Result*>(handle);
    return (int)(is_int_field(field) ? r->ints[field].size() : r->doubles[field].size());
}

// Copy an int field into out (capacity values). Returns the count copied, or
// -1 if the field is not an int field, the handle is null, or out is too small.
DLL_EXPORT int coverage_result_ints(const void* handle, int field, int* out, int capacity) {
    if (handle == nullptr || field < 0 || field >= F_COUNT || !is_int_field(field))
        return -1;
    const std::vector<int>& v = static_cast<const Result*>(handle)->ints[field];
    if ((int)v.size() > capacity || (out == nullptr && !v.empty()))
        return -1;
    std::copy(v.begin(), v.end(), out);
    return (int)v.size();
}

// As coverage_result_ints, for a double field.
DLL_EXPORT int coverage_result_doubles(const void* handle, int field, double* out, int capacity) {
    if (handle == nullptr || field < 0 || field >= F_COUNT || is_int_field(field))
        return -1;
    const std::vector<double>& v = static_cast<const Result*>(handle)->doubles[field];
    if ((int)v.size() > capacity || (out == nullptr && !v.empty()))
        return -1;
    std::copy(v.begin(), v.end(), out);
    return (int)v.size();
}

DLL_EXPORT void coverage_result_free(void* handle) {
    delete static_cast<Result*>(handle);
}

}  // extern "C"
