// geometry2d.h
// Declarations for the pure 2D geometric primitives implemented in geometry2d.cpp.

#pragma once

double orientation(double px, double py, double qx, double qy, double rx, double ry);
bool on_segment(double px, double py, double qx, double qy, double rx, double ry);
bool segments_intersect(double a1x, double a1y, double a2x, double a2y,
                        double b1x, double b1y, double b2x, double b2y);
