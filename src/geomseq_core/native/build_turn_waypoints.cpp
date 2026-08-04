// build_turn_waypoints.cpp
// Smooth polygonal turn generation between two path segments.
// Ported from the smooth-turn-waypoints Python prototype.
//
// Only build_turn_waypoints is exported (extern "C"). unit_vec,
// angle_between, and polygonal_turn are internal helpers, not
// meant to be called directly across the ctypes boundary.

#include <cmath>
#include <vector>
#include <algorithm>

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif


static void unit_vec(double vx, double vy, double& out_vx, double& out_vy) {
    double length = sqrt(vx * vx + vy * vy);
    if (length == 0) {
        out_vx = vx;
        out_vy = vy;
    }
    else {
       out_vx =  vx / length;
       out_vy =  vy / length;
    }
}

static double angle_between(double ux, double uy, double vx, double vy) {
    double ux_unit, uy_unit;
    unit_vec(ux, uy, ux_unit, uy_unit);    
    double vx_unit, vy_unit;
    unit_vec(vx, vy, vx_unit, vy_unit);

    double dot = ux_unit * vx_unit + uy_unit * vy_unit;
    dot = std::max(-1.0, std::min(1.0, dot));

    double cross = ux_unit * vy_unit - uy_unit * vx_unit;
    double angle = acos(dot);

    return (cross < 0) ? -angle : angle;
}

static void polygonal_turn(
    double start_x, double start_y,
    double heading_x, double heading_y,
    double beta,
    int k_steps,
    double step_len,
    double extend_len,
    double* out_pts,
    int* out_count)
{
    if (k_steps == 0) {
        *out_count = 0;
        return;
    }

    int count = 0;
    double cur_x = start_x;
    double cur_y = start_y;

    if (extend_len > 0.0) {
        cur_x = cur_x + heading_x * extend_len;
        cur_y = cur_y + heading_y * extend_len;

        out_pts[count * 2] = cur_x;
        out_pts[count * 2 + 1] = cur_y;
        count ++;
    }

    double cur_hx = heading_x;
    double cur_hy = heading_y;

    double dtheta = beta / k_steps;
    double cs = cos(dtheta);
    double sn = sin(dtheta);

    for (int step = 0; step < k_steps - 1; step++) {
        double new_hx = cur_hx*cs - cur_hy*sn;
        double new_hy = cur_hx*sn + cur_hy*cs;
        cur_hx = new_hx;
        cur_hy = new_hy;

        double norm_hx, norm_hy;
        unit_vec(cur_hx, cur_hy, norm_hx, norm_hy);
        cur_x = cur_x + norm_hx * step_len;
        cur_y = cur_y + norm_hy * step_len;

        out_pts[count * 2]     = cur_x;
        out_pts[count * 2 + 1] = cur_y;
        count++;
    }
    *out_count = count;
}

// ---------------------------------------------------------------------------
// Exported entry point
// ---------------------------------------------------------------------------

extern "C" {

// Build smooth turn waypoints between the end of one path (E, heading a_vec)
// and the start of the next path (S, heading b_vec).
//
// Inputs (read-only):
//   Ex, Ey       : end of current path
//   a_vx, a_vy   : heading leaving E (not necessarily unit length)
//   Sx, Sy       : start of next path
//   b_vx, b_vy   : desired heading into S (not necessarily unit length)
//   theta_max_deg: max turn angle per step (degrees)
//   step_len     : arc step length (must be > 0)
//   extend_len   : forward offset before the turn begins (>= 0)
//
// Outputs (caller pre-allocates, sized for at least k_steps+1 points --
// caller does not know k_steps in advance, so allocate conservatively,
// e.g. ceil(180/theta_max_deg) + 2 points):
//   out_exit_pts / out_exit_count   : waypoints leaving E
//   out_entry_pts / out_entry_count : waypoints arriving at S
DLL_EXPORT void build_turn_waypoints(
    double Ex, double Ey,
    double a_vx, double a_vy,
    double Sx, double Sy,
    double b_vx, double b_vy,
    double theta_max_deg,
    double step_len,
    double extend_len,
    double* out_exit_pts, int* out_exit_count,
    double* out_entry_pts, int* out_entry_count)
{
    // 1. Unitize headings
    double a_hx, a_hy;
    unit_vec(a_vx, a_vy, a_hx, a_hy);
    double b_hx, b_hy;
    unit_vec(b_vx, b_vy, b_hx, b_hy);

    // 2. Turn angle
    double beta = angle_between(a_hx, a_hy, b_hx, b_hy);

    // Exit and entry each cover half the turn, so the two halves add up to
    // one continuous arc instead of each independently sweeping the full
    // angle (which doubles the total rotation of the combined path).
    double beta_half = beta / 2.0;
    double beta_half_abs = fabs(beta_half);

    // 3. Steps needed (per half)
    double theta_rad = (theta_max_deg  > 1e-6) ? (theta_max_deg * acos(-1.0) / 180.0) : (1.0 * acos(-1.0) / 180.0);
    int k_steps_half = (beta_half_abs > 1e-6) ? (int)ceil(beta_half_abs / theta_rad) : 0;

    // 4. Distance needed vs available. Exit and entry each independently
    // consume extend_len + k_steps_half*step_len from their own end of the
    // E-S gap, so both sides' consumption is counted (2x, not 1x).
    double needed = 2.0 * extend_len + 2.0 * k_steps_half * step_len;
    double available = sqrt((Ex - Sx) * (Ex - Sx) + (Ey - Sy) * (Ey - Sy));

    // 5. Fallback: not enough space, just extend both ends
    if (needed > available) {
        // Exit: extend E toward a_h
        out_exit_pts[0] = Ex + a_hx * extend_len;
        out_exit_pts[1] = Ey + a_hy * extend_len;
        *out_exit_count = 1;
        // Entry: extend S toward b_h
        out_entry_pts[0] = Sx + b_hx * extend_len;
        out_entry_pts[1] = Sy + b_hy * extend_len;
        *out_entry_count = 1;
        return;
    }
    // 6. Exit: polygonal turn from E, heading a_h, rotating by beta_half
    polygonal_turn(Ex, Ey, a_hx, a_hy, beta_half, k_steps_half,
                   step_len, extend_len,
                   out_exit_pts, out_exit_count);

    // 7. Entry: computed in reverse (from S, heading -b_h, rotating by
    //    -beta_half), then reversed into correct order
    std::vector<double> entry_raw((k_steps_half + 1) * 2);
    int entry_count_raw;

    polygonal_turn(Sx, Sy, -b_hx, -b_hy, -beta_half, k_steps_half,
                   step_len, extend_len,
                   entry_raw.data(), &entry_count_raw);

    for (int i = 0; i < entry_count_raw; i++) {
        int reversed_i = entry_count_raw - 1 - i;
        out_entry_pts[i * 2]     = entry_raw[reversed_i * 2];
        out_entry_pts[i * 2 + 1] = entry_raw[reversed_i * 2 + 1];
    }
    *out_entry_count = entry_count_raw;
}

} // extern "C"