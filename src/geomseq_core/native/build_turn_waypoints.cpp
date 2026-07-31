// WIP

#include <cmath>
#include <algorithm>

void unit_vec(double vx, double vy, double& out_vx, double& out_vy) {
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

double angle_between(double ux, double uy, double vx, double vy) {
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

void build_turn_waypoints(
    double Ex, double Ey,
    double a_vx, double a_vy,
    double Sx, double Sy,
    double b_vx, double b_vy,
    double theta_max_deg,
    double step_len,
    double extend_len,
    double* out_pts,
    int* out_count)
{
    double a_hx, a_hy;
    unit_vec(a_vx, a_vy, a_hx, a_hy);
    double b_hx, b_hy;
    unit_vec(b_vx, b_vy, b_hx, b_hy);

    double beta = angle_between(a_hx, a_hy, b_hx, b_hy);
    double beta_abs =fabs(beta);
    double theta_rad = (theta_max_deg  > 1e-6) ? (theta_max_deg * acos(-1.0) / 180.0) : (1.0 * acos(-1.0) / 180.0);

    int k_steps = (beta_abs > 1e-6) ? (int)ceil(beta_abs / theta_rad) : 0;

    double needed = extend_len + k_steps * step_len;
    double available = sqrt((Ex - Sx) * (Ex - Sx) + (Ey - Sy) * (Ey - Sy));

}