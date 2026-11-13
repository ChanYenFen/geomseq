// redistribute_arc_lengths.cpp
// Redistributes arc lengths along a curve to a new density profile.
// Pure 1D arithmetic; the caller maps the result back onto the curve.

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

// Inputs (read-only):
//   total_length   : arc length of the whole curve
//   low            : smallest step size (used at the density peak); must be > 0
//   high           : largest step size (used at the sparsest point)
//   mode           : 0 = dense_center, 1 = dense_sides
//   flat_pct       : percent of total_length (centered) held at constant
//                    density; the remainder fades between low and high
//   corner_lengths : arc lengths that must appear exactly in the output,
//                    ascending; may be nullptr if num_corners == 0
//   num_corners    : length of corner_lengths, 0 if none
//
// Outputs (caller pre-allocates, we fill):
//   out_arc_lengths : new arc-length samples; sizing formula is in
//                     geometry_utils.redistribute_arc_lengths_native
//   out_count       : number of entries written to out_arc_lengths
DLL_EXPORT void redistribute_arc_lengths(
    double        total_length,
    double        low,
    double        high,
    int           mode,
    double        flat_pct,
    const double* corner_lengths,
    int           num_corners,
    double*       out_arc_lengths,
    int*          out_count)
{
    double fade_len = total_length * (100.0 - flat_pct) / 2.0 / 100.0;

    // dense_center: sparse edges (high), dense middle (low). dense_sides is the mirror.
    double edge_step = (mode == 0) ? high : low;
    double mid_step  = (mode == 0) ? low  : high;

    double position = 0.0;
    int count = 0;
    int next_corner_idx = 0;

    out_arc_lengths[count] = position;
    count++;

    // Skip corners at or before the start: the first point covers them, and
    // left in place they would block every corner after them.
    while (next_corner_idx < num_corners && corner_lengths[next_corner_idx] <= position) {
        next_corner_idx++;
    }

    while (position < total_length) {
        double step;

        if (position < fade_len) {
            // Front fade: edge_step -> mid_step as position goes 0 -> fade_len.
            double t = position / fade_len;
            if (t > 1.0) t = 1.0;
            step = edge_step - t * (edge_step - mid_step);
        }
        else if (position > total_length - fade_len) {
            // Back fade: the same, measured from the end.
            double dist_from_end = total_length - position;
            double t = dist_from_end / fade_len;
            if (t > 1.0) t = 1.0;
            step = edge_step - t * (edge_step - mid_step);
        }
        else {
            // Flat middle: constant mid_step.
            step = mid_step;
        }

        double next_position = position + step;
        if (next_position > total_length) {
            next_position = total_length;
        }

        // Corner interception: once the next corner is within two steps, split
        // the remaining distance into even sub-steps so the last one lands on
        // the corner exactly instead of being cut short.
        if (next_corner_idx < num_corners) {
            double corner_length = corner_lengths[next_corner_idx];
            if (corner_length > position) {
                double remaining = corner_length - position;
                if (remaining <= 2.0 * step) {
                    int k = (int)(remaining / step + 0.5);  // round to nearest
                    if (k < 1) k = 1;

                    if (k == 1) {
                        next_position = corner_length;  // exact, no floating-point drift
                        next_corner_idx++;
                    } else {
                        next_position = position + remaining / k;
                    }
                }
            }
        }

        position = next_position;
        out_arc_lengths[count] = position;
        count++;
    }

    *out_count = count;
}

} // extern "C"
