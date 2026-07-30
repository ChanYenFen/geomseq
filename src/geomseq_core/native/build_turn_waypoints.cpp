// def unit_vec(v):
//     """Return a unitized copy of v. Returns zero vector if input is zero."""
//     v = rg.Vector3d(v)
//     if v.IsZero:
//         return v
//     v.Unitize()
//     return v
#include <cmath>

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

double angle_between(double ux, double uy) {

    return;
}