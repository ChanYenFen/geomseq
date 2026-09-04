"""Generate the synthetic benchmark fixtures in benchmarks/fixtures/.

Run once; the output is what the harness reads. Nothing imports this -- run.py
and cases.py never see it, so it can be deleted without touching the benchmark.
The recipe is also written up in docs/benchmark-fixtures.md, and every generated
file repeats its own settings in a `params` block, so the datasets stay
interpretable if this script goes away.

    python benchmarks/synth/make_fixtures.py

Two axes are varied INDEPENDENTLY, because a generator that changes both at
once cannot tell you which one moved the number:

    spatial distribution   uniform / clustered (density contrast) / lattice
    input order            shuffled / serpentine / partially shuffled

points_grid and points_zigzag share one lattice and one seed and differ ONLY in
output order, so the time between them is attributable to order alone.
"""

import json
import math
import os
import random

_HERE = os.path.dirname(os.path.abspath(__file__))          # benchmarks/synth
OUT_DIR = os.path.join(os.path.dirname(_HERE), "fixtures")  # benchmarks/fixtures

EXTENT = 1000.0     # matches cases.EXTENT, so uniform stays the same control
PRECISION = 4       # matches benchmarks/gh/export_fixture.py
N_POINTS = 64000    # max of cases.SORT_POINTS_SIZES -- every sweep size samples down
N_CURVES = 16000    # max of cases.SORT_CURVES_SIZES

SEG_MIN, SEG_MAX = 5.0, 20.0    # matches cases.make_segments

# clustered: dense patches covering PATCH_AREA_FRAC of the extent
N_PATCHES = 8
PATCH_AREA_FRAC = 0.01


def r(v):
    return round(v, PRECISION)


def write(kind, name, data, params, notes):
    payload = {
        "kind": kind,
        "name": name,
        "source": "synthetic, benchmarks/synth/make_fixtures.py; "
                  "see docs/benchmark-fixtures.md",
        "units": "mm (nominal -- synthetic, extent %g)" % EXTENT,
        "notes": notes,
        "params": params,
    }
    path = os.path.join(OUT_DIR, "%s_%s.json" % (kind, name))
    # Metadata one key per line so the header is scannable; `data` stays on a
    # single line. Indenting it too costs 2.4x the size for nothing anyone reads.
    head = json.dumps(payload, indent=2, ensure_ascii=False)[:-2]   # drop "\n}"
    with open(path, "w", encoding="utf-8") as fh:
        fh.write(head + ',\n  "data": '
                 + json.dumps(data, separators=(",", ":")) + "\n}\n")
    print("%-34s %6d rows  %6.2f MB" % (os.path.basename(path), len(data),
                                        os.path.getsize(path) / 1e6))


# --------------------------------------------------------------------------
# Spatial distributions
# --------------------------------------------------------------------------

def uniform_xy(n, seed):
    rng = random.Random(seed)
    return [(rng.uniform(0, EXTENT), rng.uniform(0, EXTENT)) for _ in range(n)]


def patches(seed):
    """N_PATCHES non-overlapping squares covering PATCH_AREA_FRAC of the extent."""
    side = math.sqrt(EXTENT * EXTENT * PATCH_AREA_FRAC / N_PATCHES)
    rng = random.Random(seed)
    boxes = []
    while len(boxes) < N_PATCHES:
        x = rng.uniform(0, EXTENT - side)
        y = rng.uniform(0, EXTENT - side)
        if all(abs(x - bx) >= side and abs(y - by) >= side for bx, by in boxes):
            boxes.append((x, y))
    return boxes, side


def clustered_xy(n, contrast, seed):
    """`contrast` = point density inside the patches / density outside.

    The fraction f of points falling inside solves
        contrast = (f / a) / ((1 - f) / (1 - a)),   a = PATCH_AREA_FRAC
    Background points are rejection-sampled OUTSIDE the patches, so the realised
    ratio is the stated one rather than being diluted by background landing in
    the dense regions.
    """
    boxes, side = patches(seed)
    a = PATCH_AREA_FRAC
    ratio = contrast * a / (1.0 - a)
    frac = ratio / (1.0 + ratio)
    n_dense = int(round(n * frac))
    rng = random.Random(seed + 1)

    pts = []
    for _ in range(n_dense):
        bx, by = boxes[rng.randrange(N_PATCHES)]
        pts.append((bx + rng.uniform(0, side), by + rng.uniform(0, side)))

    def inside(x, y):
        return any(bx <= x <= bx + side and by <= y <= by + side
                   for bx, by in boxes)

    while len(pts) < n:
        x, y = rng.uniform(0, EXTENT), rng.uniform(0, EXTENT)
        if not inside(x, y):
            pts.append((x, y))

    rng.shuffle(pts)        # a spatial test only -- order must carry no signal
    return pts, n_dense, side


# --------------------------------------------------------------------------
# Lattice -- one point set, three orders
# --------------------------------------------------------------------------

GRID_COLS, GRID_ROWS, GRID_STEP = 256, 250, 4.0     # 256 * 250 = 64000

# Equal row and column spacing is deliberate: it produces EXACT ties in
# nearest-neighbour distance, which never occur in float-random data and which
# nothing else in the benchmark exercises.


def lattice_rows():
    """Rows of lattice points, each row left-to-right."""
    return [[(c * GRID_STEP, rw * GRID_STEP) for c in range(GRID_COLS)]
            for rw in range(GRID_ROWS)]


def serpentine(rows):
    """Boustrophedon: every other row reversed, so consecutive points are
    adjacent -- the order a hatch fill is actually stitched in."""
    out = []
    for i, row in enumerate(rows):
        out.extend(row if i % 2 == 0 else row[::-1])
    return out


def partial_shuffle(seq, frac, seed):
    """Permute `frac` of the positions among themselves, leaving the rest in
    place -- a dial between fully ordered (0) and fully shuffled (1)."""
    out = list(seq)
    rng = random.Random(seed)
    idx = rng.sample(range(len(out)), int(round(len(out) * frac)))
    vals = [out[i] for i in idx]
    rng.shuffle(vals)
    for i, v in zip(idx, vals):
        out[i] = v
    return out


# --------------------------------------------------------------------------
# Curves
# --------------------------------------------------------------------------

def segments_from_starts(starts, seed):
    """Randomly-oriented short segments -- stitches, not a mesh (as make_segments)."""
    rng = random.Random(seed)
    out = []
    for x0, y0 in starts:
        ang = rng.uniform(0, 2 * math.pi)
        ln = rng.uniform(SEG_MIN, SEG_MAX)
        out.append((x0, y0, x0 + math.cos(ang) * ln, y0 + math.sin(ang) * ln))
    return out


CURVE_ROWS, CURVE_PER_ROW = 100, 160        # 100 * 160 = 16000
CURVE_ROW_STEP = 10.0
CURVE_PITCH = EXTENT / CURVE_PER_ROW        # 6.25
CURVE_GAP = 0.5


def zigzag_segments():
    """Serpentine rows of short segments. Row direction alternates, so
    consecutive segments run head-to-tail and reversing any of them costs
    travel -- which is what gives if_flip something to lose here."""
    out = []
    ln = CURVE_PITCH - CURVE_GAP
    for i in range(CURVE_ROWS):
        y = i * CURVE_ROW_STEP
        for j in range(CURVE_PER_ROW):
            x = j * CURVE_PITCH
            if i % 2 == 0:
                out.append((x, y, x + ln, y))
            else:
                out.append((EXTENT - x, y, EXTENT - x - ln, y))
    return out


# --------------------------------------------------------------------------

def main():
    # 1. uniform -- the control ---------------------------------------------
    # Committed rather than left to cases.make_points so the control is the same
    # bytes on every machine, not whatever random.Random happens to produce.
    write("points", "uniform", [[r(x), r(y)] for x, y in uniform_xy(N_POINTS, 11)],
          dict(n=N_POINTS, extent=EXTENT, seed=11, order="random"),
          "Uniform scatter over an %g x %g square -- the control. Friendliest "
          "possible case for a kd-tree; every other dataset here exists to "
          "break one assumption it makes." % (EXTENT, EXTENT))

    write("curves", "uniform",
          [[r(a), r(b), r(c), r(d)]
           for a, b, c, d in segments_from_starts(uniform_xy(N_CURVES, 21), 22)],
          dict(n=N_CURVES, extent=EXTENT, seed=21, order="random",
               seg_len="%g-%g" % (SEG_MIN, SEG_MAX)),
          "Uniform short segments, random orientation -- the curve control.")

    # 2. clustered -- density contrast, order held random --------------------
    for contrast in (10, 100, 1000):
        pts, n_dense, side = clustered_xy(N_POINTS, float(contrast), 31 + contrast)
        write("points", "clustered_%dx" % contrast, [[r(x), r(y)] for x, y in pts],
              dict(n=N_POINTS, extent=EXTENT, seed=31 + contrast, order="random",
                   density_contrast=contrast, patches=N_PATCHES,
                   patch_side=r(side), patch_area_frac=PATCH_AREA_FRAC,
                   n_in_patches=n_dense),
              "%d dense square patches (%.1f%% of the area) holding %d of %d "
              "points -- %dx the background density. Probes the greedy phase's "
              "window expansion: locally dense, globally sparse."
              % (N_PATCHES, PATCH_AREA_FRAC * 100, n_dense, N_POINTS, contrast))

    starts, n_dense, side = clustered_xy(N_CURVES, 100.0, 131)
    write("curves", "clustered_100x",
          [[r(a), r(b), r(c), r(d)]
           for a, b, c, d in segments_from_starts(starts, 132)],
          dict(n=N_CURVES, extent=EXTENT, seed=131, order="random",
               density_contrast=100, patches=N_PATCHES, patch_side=r(side),
               patch_area_frac=PATCH_AREA_FRAC, n_in_patches=n_dense,
               seg_len="%g-%g" % (SEG_MIN, SEG_MAX)),
          "Segment starts drawn from the 100x clustered point process.")

    # 3. lattice -- geometry held fixed, order varied ------------------------
    rows = lattice_rows()
    ordered = serpentine(rows)
    lat = dict(n=N_POINTS, cols=GRID_COLS, rows=GRID_ROWS, step=GRID_STEP,
               extent="%g x %g" % ((GRID_COLS - 1) * GRID_STEP,
                                   (GRID_ROWS - 1) * GRID_STEP))

    shuffled = list(ordered)
    random.Random(41).shuffle(shuffled)
    write("points", "grid", [[r(x), r(y)] for x, y in shuffled],
          dict(lat, seed=41, order="random"),
          "Regular %d x %d lattice at %g spacing, order fully shuffled. Equal "
          "row and column spacing means exact ties in nearest-neighbour "
          "distance -- which never occur in float-random data. Same point set "
          "as points_zigzag; the difference between them is order alone."
          % (GRID_COLS, GRID_ROWS, GRID_STEP))

    write("points", "zigzag", [[r(x), r(y)] for x, y in ordered],
          dict(lat, order="serpentine"),
          "The points_grid lattice in serpentine (boustrophedon) order -- "
          "already optimally sorted. Best case: greedy has nothing to do and "
          "2-opt should converge immediately. A lower bound, and a test of "
          "whether existing order is detected.")

    write("points", "zigzag_s10",
          [[r(x), r(y)] for x, y in partial_shuffle(ordered, 0.10, 42)],
          dict(lat, seed=42, order="serpentine, 10% of positions permuted"),
          "points_zigzag with 10%% of positions permuted -- mostly ordered with "
          "local disorder, which is what a real toolpath usually looks like. "
          "Interpolates between points_zigzag (0%%) and points_grid (100%%).")

    write("curves", "zigzag",
          [[r(a), r(b), r(c), r(d)] for a, b, c, d in zigzag_segments()],
          dict(n=N_CURVES, rows=CURVE_ROWS, per_row=CURVE_PER_ROW,
               row_step=CURVE_ROW_STEP, pitch=CURVE_PITCH, gap=CURVE_GAP,
               order="serpentine"),
          "Serpentine rows of short segments, already in stitch order and with "
          "row direction alternating -- so consecutive segments run head to "
          "tail and reversing any of them costs travel. Exercises if_flip on "
          "input that is already correct.")


if __name__ == "__main__":
    main()
