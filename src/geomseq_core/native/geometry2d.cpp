// geometry2d.cpp
// Pure 2D geometric primitives -- no kd-tree, no sorting, no CAD dependency.

# include <cmath>
# include <algorithm>

// Returns: positive if r is leftof line p0->p1, negative if rightof, 0 if collinear.
double orientation(double px, double py, double qx, double qy, double rx, double ry) {
    double cross = (qx - px) * (ry - py) - (qy - py) * (rx - px);

    return cross;
}

// Precondition: p, q, r are already known to be collinear.
// Returns true if r lies within the bounding box of segment p-q
// (i.e. r is between p and q, inclusive of endpoints).
bool on_segment(double px, double py, double qx, double qy, double rx, double ry) {
    bool x_in_range = rx >= std::min(px, qx) && rx <= std::max(px, qx);
    bool y_in_range = ry >= std::min(py, qy) && ry <= std::max(py, qy);
    return x_in_range && y_in_range;
}

bool segments_intersect(double a1x, double a1y, double a2x, double a2y,
                        double b1x, double b1y, double b2x, double b2y) {
    double d1 = orientation(a1x, a1y, a2x, a2y, b1x, b1y);
    double d2 = orientation(a1x, a1y, a2x, a2y, b2x, b2y);
    double d3 = orientation(b1x, b1y, b2x, b2y, a1x, a1y);                   
    double d4 = orientation(b1x, b1y, b2x, b2y, a2x, a2y);
    
    if (d1*d2 < 0 && d3*d4 < 0){
        return true;
    }

    // Collinear special cases: a zero orientation means the point lies
    // exactly on the other segment's line -- check it's within range.
    if (d1 == 0 && on_segment(a1x, a1y, a2x, a2y, b1x, b1y)) return true;
    if (d2 == 0 && on_segment(a1x, a1y, a2x, a2y, b2x, b2y)) return true;
    if (d3 == 0 && on_segment(b1x, b1y, b2x, b2y, a1x, a1y)) return true;
    if (d4 == 0 && on_segment(b1x, b1y, b2x, b2y, a2x, a2y)) return true;
    
    return false; 
}
