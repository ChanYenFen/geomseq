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
//     cl /std:c++17 /O2 /EHsc /MT bench_core.cpp ..\..\src\geomseq_core\native\redistribute_lookups.cpp ..\..\src\geomseq_core\native\build_turn_waypoints.cpp /Fe:bench_core.exe
//   macOS / Linux
//     c++ -std=c++17 -O2 -o bench_core bench_core.cpp ../../src/geomseq_core/native/redistribute_lookups.cpp ../../src/geomseq_core/native/build_turn_waypoints.cpp
//
// Emits JSON on stdout -- redirect into ../results/. Formatting for humans is
// compare.py's job, so there is only one place that renders a table.

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

// Declared here rather than included: the core has no public header, and the
// point is to exercise exactly the symbols ctypes would call. These must stay
// in step with the .cpp definitions -- a mismatch is a link error, not silent
// corruption, which is the one advantage this has over the ctypes side.
extern "C" {
void redistribute_lookups(double total_length, double low, double high, int mode,
                          double flat_pct, const double* corner_lengths, int num_corners,
                          double* out_lookups, int* out_count);

void build_turn_waypoints(double Ex, double Ey, double a_vx, double a_vy,
                          double Sx, double Sy, double b_vx, double b_vy,
                          double theta_max_deg, double step_len, double extend_len,
                          double* out_exit_pts, int* out_exit_count,
                          double* out_entry_pts, int* out_entry_count);
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
// redistribute_lookups
// --------------------------------------------------------------------------
// Note there is no input-size axis here any more. Since the ABI change the
// native side takes total_length and the corner arc lengths only -- the input
// lookup array it used to receive and ignore is gone, so `input_n` is now a
// purely Python-side concept. That absence is the finding, not an omission;
// compare.py annotates it on the Python rows.

static const double TOTAL = 1000.0;
static const int REDIST_BATCH = 200;

// Mirrors cases.spread_corners(10001, count) resolved through even_lookups:
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
        redistribute_lookups(TOTAL, low, high, mode, 50.0,
                             cl.empty() ? nullptr : cl.data(), corners,
                             out.data(), &count);
        g_sink += out[0];
    };
    double us = bench_us(call, REDIST_BATCH);

    char key[128];
    std::snprintf(key, sizeof key, "\"band\": \"%g-%g\", \"corners\": %d, \"mode\": %d",
                  low, high, corners, mode);
    g_recs.push_back({"redistribute_lookups", key, count, us});
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

int main() {
    bench_turns();
    bench_redistribute();

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
