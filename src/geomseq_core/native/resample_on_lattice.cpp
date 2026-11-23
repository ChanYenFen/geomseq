// resample_on_lattice.cpp
// Native port of src/geomseq_core/resample_on_lattice.py: resample a path,
// given as runs on a staggered lattice or free, into points. The Python
// module is the reference and its docstrings say why each step is the way it
// is; every function here mirrors one there under the same name, and the
// arithmetic follows it operation for operation so the two agree to the bit.

#include <cmath>
#include <new>
#include <vector>

// sin and cos each called alone, as Python's math does. Called side by side,
// clang fuses them into __sincos_stret, which on macOS arm64 can differ from
// sin and cos in the last bit; a call it cannot see into stops the fusion.
#if defined(_MSC_VER)
#define GEOMSEQ_NOINLINE __declspec(noinline)
#else
#define GEOMSEQ_NOINLINE __attribute__((noinline))
#endif
static GEOMSEQ_NOINLINE double sin_alone(double a) { return std::sin(a); }
static GEOMSEQ_NOINLINE double cos_alone(double a) { return std::cos(a); }

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

namespace {

// Negative: nothing was written. geometry_utils.py maps each to the
// ValueError the Python module raises for the same input.
const int RL_OK = 0;
const int RL_ERR_BAD_ARGUMENT = -1;   // null buffer, negative count
const int RL_ERR_SPACING = -2;        // "spacing must be greater than 0"
const int RL_ERR_STEP = -3;           // "step must be greater than 0"
const int RL_ERR_STAGGER = -4;        // "stagger must be greater than 0"
const int RL_ERR_FREE_STEP = -5;      // "free_step must be greater than 0"
const int RL_ERR_MIN_STEP = -6;       // "min_step cannot be negative"
const int RL_ERR_INTERNAL = -99;

struct Pt {
    double x, y;
};

double dist(const Pt& a, const Pt& b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

// Python's float x % 1.0: the sign follows the divisor.
double py_mod1(double x) {
    double m = std::fmod(x, 1.0);
    if (m != 0.0) {
        if (m < 0.0)
            m += 1.0;
    } else {
        m = std::copysign(0.0, 1.0);
    }
    return m;
}

std::vector<Pt> drop_repeats(const std::vector<Pt>& pts, double tol) {
    std::vector<Pt> out;
    for (const Pt& p : pts)
        if (out.empty() || dist(p, out.back()) > tol)
            out.push_back(p);
    return out;
}

void on_lattice(const Pt& a, const Pt& b, double y_row, double spacing, double step, double phase,
                double stagger, double tol, std::vector<Pt>& out) {
    double r = (y_row - phase) / spacing;
    double offset = py_mod1(r / stagger) * step;
    double lo = std::fmin(a.x, b.x), hi = std::fmax(a.x, b.x);
    double m = std::floor((lo - offset) / step) + 1.0;
    std::vector<Pt> here;
    while (true) {
        double x = offset + m * step;
        if (x >= hi - tol)
            break;
        if (x > lo + tol) {
            double t = (x - a.x) / (b.x - a.x);
            here.push_back({x, a.y + t * (b.y - a.y)});
        }
        m += 1.0;
    }
    if (a.x > b.x)
        out.insert(out.end(), here.rbegin(), here.rend());
    else
        out.insert(out.end(), here.begin(), here.end());
}

void even(const Pt& a, const Pt& b, double step, std::vector<Pt>& out) {
    int n = (int)std::ceil(dist(a, b) / step - 1e-9);
    for (int k = 1; k < n; ++k)
        out.push_back({a.x + (b.x - a.x) * k / n, a.y + (b.y - a.y) * k / n});
}

void lattice_run(const std::vector<Pt>& local, double spacing, double step, double phase, double stagger,
                 bool skip_row_ends, double tol, std::vector<Pt>& out) {
    long last = (long)local.size() - 2;
    for (size_t i = 0; i + 1 < local.size(); ++i) {
        const Pt& a = local[i];
        const Pt& b = local[i + 1];
        double dx = std::fabs(b.x - a.x), dy = std::fabs(b.y - a.y);
        if (dy <= tol) {
            on_lattice(a, b, b.y, spacing, step, phase, stagger, tol, out);
            if (!(skip_row_ends && (long)i < last))
                out.push_back(b);
        } else if (dy <= spacing + tol && dx > dy) {
            on_lattice(a, b, b.y, spacing, step, phase, stagger, tol, out);
            out.push_back(b);
        } else {
            even(a, b, step, out);
            out.push_back(b);
        }
    }
}

void along(const std::vector<Pt>& pts, double free_step, std::vector<Pt>& out) {
    std::vector<double> lengths;
    for (size_t i = 0; i + 1 < pts.size(); ++i)
        lengths.push_back(dist(pts[i], pts[i + 1]));
    double total = 0.0;
    for (double length : lengths)
        total += length;
    int n = (int)std::ceil(total / free_step - 1e-9);
    size_t seg = 0;
    double walked = 0.0;
    for (int k = 1; k < n; ++k) {
        double target = total * k / n;
        while (seg + 1 < lengths.size() && walked + lengths[seg] < target) {
            walked += lengths[seg];
            ++seg;
        }
        double t = lengths[seg] > 0.0 ? (target - walked) / lengths[seg] : 0.0;
        const Pt& a = pts[seg];
        const Pt& b = pts[seg + 1];
        out.push_back({a.x + t * (b.x - a.x), a.y + t * (b.y - a.y)});
    }
}

std::vector<Pt> merge_short(const std::vector<Pt>& points, double min_step, double tol) {
    if (min_step <= tol)
        return drop_repeats(points, tol);
    std::vector<Pt> out{points.front()};
    for (size_t i = 1; i + 1 < points.size(); ++i)
        if (dist(points[i], out.back()) >= min_step - tol)
            out.push_back(points[i]);
    const Pt& end = points.back();
    if (out.size() > 1 && dist(end, out.back()) < min_step - tol)
        out.pop_back();
    if (dist(end, out.back()) > tol)
        out.push_back(end);
    return out;
}

}  // namespace

extern "C" {

// Resample a path given as runs: resample_on_lattice.resample_on_lattice.
//   xy          x y of every run vertex, run after run (2 * sum(run_counts))
//   run_counts  vertex count of each run
//   run_free    per run: 0 on the lattice, 1 free
//   num_runs    0 or more
//   spacing, step, angle (radians), phase, stagger: the lattice
//   free_step   longest step along a free run; > 0 (the Python's None is
//               resolved by the caller)
//   skip_row_ends  0/1
//   min_step    shortest step kept; >= 0 (None resolved by the caller)
//   out_xy      room for out_capacity points, x y each
//   out_count   receives the number of points REQUIRED; above out_capacity,
//               nothing was written and the caller repeats with more room
// Returns RL_OK or a negative RL_ERR_* code.
DLL_EXPORT int resample_on_lattice(const double* xy, const int* run_counts, const int* run_free, int num_runs,
                                   double spacing, double step, double angle, double phase, double stagger,
                                   double free_step, int skip_row_ends, double min_step, double tol,
                                   double* out_xy, int out_capacity, int* out_count) {
    if (out_count == nullptr || num_runs < 0 || out_capacity < 0 ||
        (num_runs > 0 && (xy == nullptr || run_counts == nullptr || run_free == nullptr)))
        return RL_ERR_BAD_ARGUMENT;
    if (!(spacing > 0.0))
        return RL_ERR_SPACING;
    if (!(step > 0.0))
        return RL_ERR_STEP;
    if (!(stagger > 0.0))
        return RL_ERR_STAGGER;
    if (!(free_step > 0.0))
        return RL_ERR_FREE_STEP;
    if (min_step < 0.0)
        return RL_ERR_MIN_STEP;
    try {
        double c = cos_alone(-angle), s = sin_alone(-angle);
        std::vector<Pt> out;
        size_t at = 0;
        for (int r = 0; r < num_runs; ++r) {
            if (run_counts[r] < 0)
                return RL_ERR_BAD_ARGUMENT;
            std::vector<Pt> turned;
            for (int k = 0; k < run_counts[r]; ++k, ++at) {
                double x = xy[2 * at], y = xy[2 * at + 1];
                turned.push_back({c * x - s * y, s * x + c * y});
            }
            std::vector<Pt> local = drop_repeats(turned, tol);
            if (local.empty())
                continue;
            if (!out.empty() && dist(out.back(), local.front()) > tol)
                out.push_back(local.front());
            else if (out.empty())
                out.push_back(local.front());
            if (local.size() < 2)
                continue;
            if (run_free[r]) {
                along(local, free_step, out);
                out.push_back(local.back());
            } else {
                lattice_run(local, spacing, step, phase, stagger, skip_row_ends != 0, tol, out);
            }
        }
        if (!out.empty())
            out = merge_short(out, min_step, tol);
        *out_count = (int)out.size();
        if ((int)out.size() > out_capacity)
            return RL_OK;
        double cb = cos_alone(angle), sb = sin_alone(angle);
        for (size_t i = 0; i < out.size(); ++i) {
            double x = out[i].x, y = out[i].y;
            out_xy[2 * i] = cb * x - sb * y;
            out_xy[2 * i + 1] = sb * x + cb * y;
        }
        return RL_OK;
    } catch (...) {
        return RL_ERR_INTERNAL;
    }
}

}  // extern "C"
