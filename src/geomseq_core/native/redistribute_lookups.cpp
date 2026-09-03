// redistribute_lookups.cpp
// Redistributes arc-length lookups along a curve to a new density profile --
// pure 1D arithmetic; the curve itself is never touched (Python maps the result back via curve.PointAt()).

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

// Inputs (read-only):
//   total_length   : arc length of the whole curve (the old `lookups[n-1]`)
//   low            : smallest step size (used at the density peak)
//   high           : largest step size (used at the sparsest point)
//   mode           : 0 = dense_center, 1 = dense_sides
//   flat_pct       : percent of total_length (centered) held at constant
//                    density; the remainder fades between low and high
//   corner_lengths : arc lengths that must appear exactly in the output,
//                    ascending; may be nullptr if num_corners == 0
//   num_corners    : length of corner_lengths, 0 if none
//
// This used to take the whole original lookup array plus corner *indices* into
// it. It never read more than the last element and the corner entries, so at
// 100k input samples ~97% of the call's wall time was Python marshaling an
// array this function then ignored. Resolving the corners to arc lengths on
// the Python side (an O(num_corners) list index) makes the cost independent of
// the input resolution. The Python wrapper's own signature is unchanged.
//
// Outputs (caller pre-allocates, we fill):
//   out_lookups : new arc-length samples (caller must size the buffer
//                 generously -- see geometry_utils.redistribute_lookups_native
//                 for the sizing formula)
//   out_count   : number of entries actually written to out_lookups
DLL_EXPORT void redistribute_lookups(
    double        total_length,
    double        low,
    double        high,
    int           mode,
    double        flat_pct,
    const double* corner_lengths,
    int           num_corners,
    double*       out_lookups,
    int*          out_count)
{
    double fade_len = total_length * (100.0 - flat_pct) / 2.0 / 100.0;

    // dense_center: edges are sparse (high), the fade settles into a dense
    // (low) middle. dense_sides mirrors it -- edges dense (low), middle
    // sparse (high). Both fade branches and the middle branch below just
    // interpolate/hold between these two, so the mode fork lives only here.
    double edge_step = (mode == 0) ? high : low;
    double mid_step  = (mode == 0) ? low  : high;

    double position = 0.0;
    int count = 0;
    int next_corner_idx = 0;

    out_lookups[count] = position;
    count++;

    // Skip any corner at or before the start -- already covered by the
    // initial point above. Without this, a corner at arc length 0 would
    // permanently stall next_corner_idx at 0 (corner_length > position is
    // false when both are 0), and every real corner after it would never
    // get consumed.
    while (next_corner_idx < num_corners && corner_lengths[next_corner_idx] <= position) {
        next_corner_idx++;
    }

    while (position < total_length) {
        double step;

        if (position < fade_len) {
            // Front fade: edge_step -> mid_step as position goes 0 -> fade_len.
            // t is guaranteed < 1.0 here by this branch's own guard
            // (position < fade_len), so the clamp below is defensive, not
            // reachable under the current branch structure -- kept anyway
            // as a cheap safety net.
            double t = position / fade_len;
            if (t > 1.0) t = 1.0;
            step = edge_step - t * (edge_step - mid_step);
        }
        else if (position > total_length - fade_len) {
            // Back fade: mid_step -> edge_step, mirrored using distance from
            // the end. Same defensive-clamp note as above applies here.
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

        // Corner interception: checking only "would THIS step overshoot the
        // corner" (i.e. corner_length <= next_position) is too late to ever
        // redistribute into more than one sub-step -- by construction that
        // condition only becomes true once remaining <= step, so k below
        // would always round to 1 anyway. Instead look 2 natural steps
        // ahead: once the corner is that close, redistribute the remaining
        // distance into roughly natural-sized sub-steps now. Re-evaluated
        // every iteration as position gets closer, so it converges to an
        // exact landing on the corner instead of one abrupt final
        // truncation.
        if (next_corner_idx < num_corners) {
            double corner_length = corner_lengths[next_corner_idx];
            if (corner_length > position) {
                double remaining = corner_length - position;
                if (remaining <= 2.0 * step) {
                    int k = (int)(remaining / step + 0.5);  // round to nearest
                    if (k < 1) k = 1;

                    if (k == 1) {
                        next_position = corner_length;  // land exactly, no FP drift from position+remaining
                        next_corner_idx++;
                    } else {
                        next_position = position + remaining / k;
                    }
                }
            }
        }

        position = next_position;
        out_lookups[count] = position;
        count++;
    }

    *out_count = count;
}

} // extern "C"
