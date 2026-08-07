// build_turn_waypoints.cpp
// Smooth polygonal turn generation between two path segments.
//
// Only build_turn_waypoints is exported (extern "C"). unit_vec,
// angle_between, polygonal_turn, and fillet_corner are internal
// helpers, not meant to be called across the ctypes boundary.

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

// Rounds the corner at `vertex` by walking away from it, starting along
// fixed_x/y (the heading arriving at the vertex, in walk direction) and
// rotating at most theta_max_deg per step until it reaches rotate_x/y (the
// heading departing it) -- so no vertex in the emitted chain turns by more
// than theta_max_deg. Note both are travel headings, not outward rays from
// the vertex: at a hairpin the two rays nearly coincide while the headings
// are nearly opposed, and it's the heading change that must be smoothed.
// The vertex itself is not emitted; the caller places it. Both direction
// inputs must be unit length. out_pts needs ceil(180/theta_max_deg) points.
static void fillet_corner(
    double vertex_x, double vertex_y,
    double fixed_x, double fixed_y,
    double rotate_x, double rotate_y,
    double step_len,
    double theta_max_deg,
    double* out_pts,
    int* out_count)
{
    double beta = angle_between(fixed_x, fixed_y, rotate_x, rotate_y);
    double beta_abs = fabs(beta);

    double theta_rad = (theta_max_deg > 1e-6) ? (theta_max_deg * acos(-1.0) / 180.0)
                                              : (1.0 * acos(-1.0) / 180.0);
    int k_steps = (beta_abs > 1e-6) ? (int)ceil(beta_abs / theta_rad) : 0;

    if (k_steps == 0) {
        *out_count = 0;
        return;
    }

    int count = 0;
    double cur_x = vertex_x;
    double cur_y = vertex_y;
    double cur_hx = fixed_x;
    double cur_hy = fixed_y;

    double dtheta = beta / k_steps;
    double cs = cos(dtheta);
    double sn = sin(dtheta);

    // k_steps rotations, so the last heading lands exactly on rotate_x/y.
    for (int step = 0; step < k_steps; step++) {
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
// E and S are each extended by extend_len into the gap between them, and the
// corner at each extend point is filleted independently -- the real path
// segments themselves never rotate.
//
// Inputs (read-only):
//   Ex, Ey       : end of current path
//   a_vx, a_vy   : heading leaving E (not necessarily unit length)
//   Sx, Sy       : start of next path
//   b_vx, b_vy   : desired heading into S (not necessarily unit length)
//   theta_max_deg: max turn angle per waypoint (degrees)
//   step_len     : fillet step length (must be > 0)
//   extend_len   : how far past E / before S the fillet corners sit (>= 0)
//
// Outputs (caller pre-allocates; each side emits at most one extend point
// plus ceil(180/theta_max_deg) fillet points, so ceil(180/theta_max_deg) + 2
// is a safe size):
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
    // 1. Unitize the two real-path headings.
    double a_hx, a_hy;
    unit_vec(a_vx, a_vy, a_hx, a_hy);
    double b_hx, b_hy;
    unit_vec(b_vx, b_vy, b_hx, b_hy);

    // 2. Extend points, both pushed into the turn zone: E forward along its
    //    own heading, S backward along its own. This keeps E_prev/E/E_extend
    //    and S_extend/S/S_next collinear, so neither real path segment moves.
    double e_ext_x = Ex + a_hx * extend_len;
    double e_ext_y = Ey + a_hy * extend_len;
    double s_ext_x = Sx - b_hx * extend_len;
    double s_ext_y = Sy - b_hy * extend_len;

    // 3. The chord joining them is the free leg both fillets rotate toward.
    double chord_hx, chord_hy;
    unit_vec(s_ext_x - e_ext_x, s_ext_y - e_ext_y, chord_hx, chord_hy);

    // 4. Exit: E_extend, then round its corner -- arriving heading a_h,
    //    departing heading the chord. Walked along travel direction, so it
    //    appends directly in order.
    out_exit_pts[0] = e_ext_x;
    out_exit_pts[1] = e_ext_y;

    int exit_fillet_count;
    fillet_corner(e_ext_x, e_ext_y, a_hx, a_hy, chord_hx, chord_hy,
                  step_len, theta_max_deg,
                  out_exit_pts + 2, &exit_fillet_count);
    *out_exit_count = 1 + exit_fillet_count;

    // 5. Entry: same treatment at S_extend, but walked backwards -- arriving
    //    heading is -b_h (S->S_extend reversed), departing is -chord_h (back
    //    up the chord). So the fillet comes out back-to-front and is flipped,
    //    with S_extend last.
    int k_max = (int)ceil(180.0 / ((theta_max_deg > 1e-6) ? theta_max_deg : 1.0));
    std::vector<double> entry_raw(k_max * 2);
    int entry_fillet_count;

    fillet_corner(s_ext_x, s_ext_y, -b_hx, -b_hy, -chord_hx, -chord_hy,
                  step_len, theta_max_deg,
                  entry_raw.data(), &entry_fillet_count);

    for (int i = 0; i < entry_fillet_count; i++) {
        int reversed_i = entry_fillet_count - 1 - i;
        out_entry_pts[i * 2]     = entry_raw[reversed_i * 2];
        out_entry_pts[i * 2 + 1] = entry_raw[reversed_i * 2 + 1];
    }
    out_entry_pts[entry_fillet_count * 2]     = s_ext_x;
    out_entry_pts[entry_fillet_count * 2 + 1] = s_ext_y;
    *out_entry_count = entry_fillet_count + 1;
}

} // extern "C"