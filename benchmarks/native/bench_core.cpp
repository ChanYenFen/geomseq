// bench_core.cpp -- pure-native timing for the geomseq core.
//
// Companion to benchmarks/python/, not a replacement. That harness times the
// wrapper as a caller experiences it (Python + ctypes + marshaling + native);
// this one links the .cpp sources straight into an executable and times only
// the algorithm. benchmarks/compare.py subtracts the two to show where each
// function's time actually goes.
//
// It exists because for the microsecond-scale functions the Python harness
// mostly measures itself: build_turn_waypoints reports ~5.8 us/call for a
// 2-point straight turn, which is wrapper and ctypes overhead, not arithmetic.
//
// Deliberately NOT wired into the shipped DLL -- no timing entry points are
// added to the production API. This compiles the same .cpp files a second
// time, into its own binary.
//
// Build (from benchmarks/native/):
//   Windows (x64 Native Tools Command Prompt)
//     cl /std:c++17 /O2 /EHsc /MT bench_core.cpp ..\..\src\geomseq_core\native\redistribute_arc_lengths.cpp ..\..\src\geomseq_core\native\build_turn_waypoints.cpp ..\..\src\geomseq_core\native\point_in_curve.cpp /Fe:bench_core.exe
//   macOS / Linux
//     c++ -std=c++17 -O2 -o bench_core bench_core.cpp ../../src/geomseq_core/native/redistribute_arc_lengths.cpp ../../src/geomseq_core/native/build_turn_waypoints.cpp ../../src/geomseq_core/native/point_in_curve.cpp
//
// Emits JSON on stdout -- redirect into ../results/. Formatting for humans is
// compare.py's job, so there is only one place that renders a table.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

// Declared here rather than included: the core has no public header, and the
// point is to exercise exactly the symbols ctypes would call. These must stay
// in step with the .cpp definitions -- a mismatch is a link error, not silent
// corruption, which is the one advantage this has over the ctypes side.
extern "C" {
void redistribute_arc_lengths(double total_length, double low, double high, int mode,
                          double flat_pct, const double* corner_lengths, int num_corners,
                          double* out_arc_lengths, int* out_count);

void build_turn_waypoints(double Ex, double Ey, double a_vx, double a_vy,
                          double Sx, double Sy, double b_vx, double b_vy,
                          double theta_max_deg, double step_len, double extend_len,
                          double* out_exit_pts, int* out_exit_count,
                          double* out_entry_pts, int* out_entry_count);

int point_in_curve(const double* polylines, const int* vertex_counts,
                   int num_polylines, const double* query_pts, int num_query,
                   double tol, int* out_relationship);
}

using clk = std::chrono::steady_clock;

// Accumulates a value derived from every result so /O2 cannot decide the calls
// are dead and delete the loop. Emitted at the end; the number is meaningless,
// its existence is the point.
static volatile double g_sink = 0.0;

struct Rec {
    std::string group;
    std::string key_json;   // object body, e.g. "\"corners\": 10, \"mode\": 0"
    int out_n;
    double per_call_us;
};
static std::vector<Rec> g_recs;

// Minimum per-call microseconds over `reps` batches -- minimum for the same
// reason the Python harness uses it: scheduling noise only ever adds.
template <typename F>
static double bench_us(F fn, int batch, int reps = 7) {
    double best = 1e300;
    for (int r = 0; r < reps; ++r) {
        auto t0 = clk::now();
        for (int i = 0; i < batch; ++i) fn();
        auto t1 = clk::now();
        double us = std::chrono::duration<double, std::micro>(t1 - t0).count() / batch;
        if (us < best) best = us;
    }
    return best;
}

static std::string toolchain() {
    char buf[64];
#if defined(_MSC_VER)
    std::snprintf(buf, sizeof buf, "MSVC %d", _MSC_VER);
#elif defined(__clang__)
    std::snprintf(buf, sizeof buf, "clang %d.%d", __clang_major__, __clang_minor__);
#elif defined(__GNUC__)
    std::snprintf(buf, sizeof buf, "g++ %d.%d", __GNUC__, __GNUC_MINOR__);
#else
    std::snprintf(buf, sizeof buf, "unknown");
#endif
    return buf;
}

// --------------------------------------------------------------------------
// build_turn_waypoints
// --------------------------------------------------------------------------
// Same three geometries, thetas, step_len and extend_len as
// benchmarks/python/cases.py, so the rows align one to one. Small and
// deterministic, so the inputs are written out here -- no fixture files.

struct Turn { const char* name; double Eprev[2], E[2], S[2], Snext[2]; };

static const Turn TURNS[] = {
    {"straight",    {0, 0}, {10, 0}, {30,  0}, {40, 0}},
    {"right_angle", {0, 0}, {10, 0}, {20, 10}, {20, 20}},
    {"hairpin",     {0, 0}, {10, 0}, {10,  5}, { 0, 5}},
};
static const double THETAS[] = {30.0, 10.0, 5.0, 1.0};
static const int TURN_BATCH = 2000;

static void bench_turns() {
    for (const Turn& t : TURNS) {
        for (double theta : THETAS) {
            double a_vx = t.E[0] - t.Eprev[0], a_vy = t.E[1] - t.Eprev[1];
            double b_vx = t.Snext[0] - t.S[0], b_vy = t.Snext[1] - t.S[1];

            int max_pts = (int)std::ceil(180.0 / theta) + 2;
            std::vector<double> exit_pts(max_pts * 2), entry_pts(max_pts * 2);
            int ec = 0, nc = 0;

            auto call = [&] {
                build_turn_waypoints(t.E[0], t.E[1], a_vx, a_vy, t.S[0], t.S[1],
                                     b_vx, b_vy, theta, 1.0, 2.0,
                                     exit_pts.data(), &ec, entry_pts.data(), &nc);
                g_sink += exit_pts[0] + entry_pts[0];
            };
            double us = bench_us(call, TURN_BATCH);

            char key[128];
            std::snprintf(key, sizeof key, "\"geometry\": \"%s\", \"theta_max_deg\": %g",
                          t.name, theta);
            g_recs.push_back({"build_turn_waypoints", key, ec + nc, us});
        }
    }
}

// --------------------------------------------------------------------------
// redistribute_arc_lengths
// --------------------------------------------------------------------------
// Note there is no input-size axis here any more. Since the ABI change the
// native side takes total_length and the corner arc lengths only -- the input
// arc_length array it used to receive and ignore is gone, so `input_n` is now a
// purely Python-side concept. That absence is the finding, not an omission;
// compare.py annotates it on the Python rows.

static const double TOTAL = 1000.0;
static const int REDIST_BATCH = 200;

// Mirrors cases.spread_corners(10001, count) resolved through even_arc_lengths:
// index int(1 + i*step) over a 10001-sample list spanning TOTAL.
static std::vector<double> spread_corners(int count) {
    std::vector<double> out;
    if (count <= 0) return out;
    double step = (10001 - 2) / (double)count;
    for (int i = 0; i < count; ++i) {
        int idx = (int)(1 + i * step);
        out.push_back(TOTAL * idx / 10000.0);
    }
    return out;
}

static void redist_row(double low, double high, int mode, int corners) {
    std::vector<double> cl = spread_corners(corners);
    double min_step = low < high ? low : high;
    std::vector<double> out((size_t)(TOTAL / min_step) + corners + 10);
    int count = 0;

    auto call = [&] {
        redistribute_arc_lengths(TOTAL, low, high, mode, 50.0,
                             cl.empty() ? nullptr : cl.data(), corners,
                             out.data(), &count);
        g_sink += out[0];
    };
    double us = bench_us(call, REDIST_BATCH);

    char key[128];
    std::snprintf(key, sizeof key, "\"band\": \"%g-%g\", \"corners\": %d, \"mode\": %d",
                  low, high, corners, mode);
    g_recs.push_back({"redistribute_arc_lengths", key, count, us});
}

static void bench_redistribute() {
    redist_row(8.0, 20.0, 0, 0);
    redist_row(2.0, 8.0, 0, 0);
    redist_row(0.5, 2.0, 0, 0);
    redist_row(0.2, 0.8, 0, 0);
    redist_row(0.5, 2.0, 0, 10);
    redist_row(0.5, 2.0, 0, 100);
    redist_row(0.5, 2.0, 0, 1000);
    redist_row(0.5, 2.0, 1, 0);
}

// --------------------------------------------------------------------------
// point_in_curve
// --------------------------------------------------------------------------
// Added with the Python group rather than after it. shatter_at_crossings went
// the other way -- its native row was left out because the bridge was under
// 0.1% of the call, and that reason expired when the broad phase made the
// native side ten times cheaper, leaving the group with nothing to subtract
// against. Here the bridge is the open question, not an afterthought: the
// wrapper rebuilds both arrays on every call, so for a short query list against
// a dense curve the marshaling could outweigh the classification.
//
// Placement matches cases.py by distribution, not point for point. The two
// harnesses cannot share a generator, and what the bridge column needs is the
// same amount and kind of work, not the same coordinates.

static const double PIC_PI = 3.14159265358979323846;
static const double PIC_R  = 400.0;    // EXTENT * 0.4 in cases.py
static const double PIC_CX = 500.0;
static const double PIC_CY = 500.0;

// A 64-bit LCG rather than <random>: the engines there are implementation
// defined in their output, and a row has to be reproducible across the three
// compilers CI builds with.
struct Lcg {
    unsigned long long s;
    explicit Lcg(unsigned long long seed) : s(seed) {}
    double next() {
        s = s * 6364136223846793005ULL + 1442695040888963407ULL;
        return (double)(s >> 11) * (1.0 / 9007199254740992.0);
    }
    double in(double lo, double hi) { return lo + (hi - lo) * next(); }
};

static std::vector<double> pic_curve(int v, const char* shape) {
    std::vector<double> out;
    out.reserve((size_t)v * 2);
    if (std::strcmp(shape, "circle") == 0) {
        int n = v - 1;
        for (int k = 0; k < n; ++k) {
            double a = 2.0 * PIC_PI * k / n;
            out.push_back(PIC_CX + PIC_R * std::cos(a));
            out.push_back(PIC_CY + PIC_R * std::sin(a));
        }
    } else {
        int teeth = v - 3;
        for (int k = 0; k < teeth; ++k) {
            out.push_back(PIC_CX - PIC_R + 2.0 * PIC_R * k / (teeth - 1));
            out.push_back(PIC_CY + ((k % 2) ? PIC_R : 0.0));
        }
        out.push_back(PIC_CX + PIC_R); out.push_back(PIC_CY - PIC_R);
        out.push_back(PIC_CX - PIC_R); out.push_back(PIC_CY - PIC_R);
    }
    double x0 = out[0], y0 = out[1];   // copied out first: push_back may reallocate
    out.push_back(x0); out.push_back(y0);
    return out;
}

static std::vector<double> pic_query(int n, const char* where) {
    Lcg rng(2);
    std::vector<double> q;
    q.reserve((size_t)n * 2);
    for (int i = 0; i < n; ++i) {
        if (std::strcmp(where, "uniform") == 0 || std::strcmp(where, "spread") == 0) {
            double r = (std::strcmp(where, "uniform") == 0) ? PIC_R : 3.0 * PIC_R;
            q.push_back(rng.in(PIC_CX - r, PIC_CX + r));
            q.push_back(rng.in(PIC_CY - r, PIC_CY + r));
        } else if (std::strcmp(where, "inside") == 0) {
            double dx, dy;
            do {
                dx = rng.in(-0.3, 0.3) * PIC_R;
                dy = rng.in(-0.3, 0.3) * PIC_R;
            } while (dx * dx + dy * dy > (0.3 * PIC_R) * (0.3 * PIC_R));
            q.push_back(PIC_CX + dx);
            q.push_back(PIC_CY - 0.5 * PIC_R + dy);
        } else {
            double a = rng.in(0.0, 2.0 * PIC_PI);
            double d = rng.in(2.0, 4.0) * PIC_R;
            q.push_back(PIC_CX + d * std::cos(a));
            q.push_back(PIC_CY + d * std::sin(a));
        }
    }
    return q;
}

static void pic_row(int n, int v, const char* shape, const char* where, double tol) {
    std::vector<double> verts = pic_curve(v, shape);
    std::vector<double> query = pic_query(n, where);
    int count = (int)(verts.size() / 2);
    std::vector<int> rel((size_t)n);

    auto call = [&] {
        point_in_curve(verts.data(), &count, 1, query.data(), n, tol, rel.data());
        g_sink += rel[0];
    };

    // Batched by the work estimate rather than a constant: the `outside` row is
    // a box test per point and lands in microseconds, while n=64000 against
    // 1000 edges is tens of milliseconds, and one constant cannot time both.
    // The estimate has to know that `outside` never reaches the scan -- billing
    // it for V edges per point gave it a batch of 2 and a reading of 0.00 us.
    double per_point = (std::strcmp(where, "outside") == 0) ? 1.0 : (double)count;
    double work = (double)n * per_point;
    int batch = (int)(2.0e7 / (work > 0.0 ? work : 1.0));
    if (batch < 1) batch = 1;
    if (batch > 2000) batch = 2000;

    double us = bench_us(call, batch);

    char key[192];
    std::snprintf(key, sizeof key,
                  "\"n\": %d, \"v\": %d, \"shape\": \"%s\", \"where\": \"%s\","
                  " \"tol\": %g, \"batch\": %d",
                  n, v, shape, where, tol, batch);
    g_recs.push_back({"point_in_curve", key, n, us});
}

static void bench_point_in_curve() {
    // n, at the V the other rows hold fixed. The smallest row is the one the
    // bridge column exists for.
    for (int n : {1000, 8000, 64000}) pic_row(n, 1000, "circle", "uniform", 0.0);

    // V, at the fixed n. n*V is the cost today, and the edge index is a claim
    // about this column specifically.
    for (int v : {100, 1000, 16000}) pic_row(8000, v, "circle", "uniform", 0.0);

    // The placement axis. `uniform` above is drawn over the box and so never
    // fires the rejection; these are the floor, the ceiling and the mixed row.
    pic_row(8000, 1000, "circle", "inside",  0.0);
    pic_row(8000, 1000, "circle", "outside", 0.0);
    pic_row(8000, 1000, "circle", "spread",  0.0);

    // The comb, whose active set is a large fraction of V where the circle's is
    // 2 at any V. Recorded now so the index cannot later be credited with a
    // difference that was the shape's all along.
    pic_row(8000, 1000, "comb", "uniform", 0.0);
}

int main() {
    bench_turns();
    bench_redistribute();
    bench_point_in_curve();

    std::printf("{\n");
    std::printf("  \"harness\": \"native\",\n");
    std::printf("  \"env\": {\"toolchain\": \"%s\", \"pointer_bits\": %zu,"
                " \"turn_batch\": %d, \"redistribute_batch\": %d},\n",
                toolchain().c_str(), sizeof(void*) * 8, TURN_BATCH, REDIST_BATCH);
    std::printf("  \"records\": [\n");
    for (size_t i = 0; i < g_recs.size(); ++i) {
        const Rec& r = g_recs[i];
        std::printf("    {\"group\": \"%s\", \"key\": {%s}, \"out_n\": %d,"
                    " \"per_call_us\": %.4f}%s\n",
                    r.group.c_str(), r.key_json.c_str(), r.out_n, r.per_call_us,
                    i + 1 < g_recs.size() ? "," : "");
    }
    std::printf("  ],\n");
    std::printf("  \"sink\": %g\n", (double)g_sink);
    std::printf("}\n");
    return 0;
}
