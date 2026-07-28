// redistribute_lookups.cpp
// Redistributes arc-length "lookups" along a curve to a new density profile,
// without touching the curve itself -- purely 1D arc-length arithmetic. The
// Python side turns the returned lookups back into curve points via
// curve.LengthParameter() + curve.PointAt().
//
// mode 0 (dense_center): points sparse near both ends, dense in the middle.
// mode 1 (dense_sides):   points dense near both ends, sparse in the middle.
// flat_pct controls how much of the curve (centered) stays at constant
// density; the rest fades between `low` and `high` step sizes.
//
// corner_indices names positions in `lookups` that must survive exactly in
// the output (e.g. polyline vertices) -- the gradient must not skip over or
// blur them away.

#if defined(_WIN32)
    #define DLL_EXPORT __declspec(dllexport)
#else
    #define DLL_EXPORT __attribute__((visibility("default")))
#endif

extern "C" {

// Inputs (read-only):
//   lookups        : original evenly-spaced arc-length samples; only
//                    lookups[n-1] (total curve length) and, via
//                    corner_indices, specific entries are used -- the rest
//                    of the original spacing is discarded and regenerated.
//   n              : number of lookups
//   low            : smallest step size (used at the density peak)
//   high           : largest step size (used at the sparsest point)
//   mode           : 0 = dense_center, 1 = dense_sides
//   flat_pct       : percent of total_length (centered) held at constant
//                    density; the remainder fades between low and high
//   corner_indices : indices into `lookups` that must appear exactly in the
//                    output; may be nullptr if num_corners == 0
//   num_corners    : length of corner_indices, 0 if none
//
// Outputs (caller pre-allocates, we fill):
//   out_lookups : new arc-length samples (caller must size the buffer
//                 generously -- see geometry_utils.redistribute_lookups_native
//                 for the sizing formula)
//   out_count   : number of entries actually written to out_lookups
DLL_EXPORT void redistribute_lookups(
    const double* lookups,
    int           n,
    double        low,
    double        high,
    int           mode,
    double        flat_pct,
    const int*    corner_indices,
    int           num_corners,
    double*       out_lookups,
    int*          out_count)
{
    double total_length = lookups[n - 1];
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
    // initial point above. Without this, a corner at lookups[0] == 0 would
    // permanently stall next_corner_idx at 0 (corner_length > position is
    // false when both are 0), and every real corner after it would never
    // get consumed.
    while (next_corner_idx < num_corners && lookups[corner_indices[next_corner_idx]] <= position) {
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
            double corner_length = lookups[corner_indices[next_corner_idx]];
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
