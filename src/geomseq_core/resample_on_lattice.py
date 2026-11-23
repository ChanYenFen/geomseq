"""Resample a path onto a staggered lattice.

The path comes in runs, each either on the lattice or free:

- A lattice run is made of rows -- segments along the lattice's x axis --
  and the joins between them. A row takes its points from the lattice; a
  join that climbs at most one row spacing and runs mostly along the rows
  takes the x of the lattice of the row it arrives on, so its points stand
  in that row's columns; anything steeper is split evenly, no longer than
  step.
- A free run ignores its vertices: points are spread evenly along its
  length, free_step apart at most. The points lie on the run, so a chord
  between two of them cuts a corner by at most the corner's sagitta.

The lattice, in the frame turned by `angle` (rows run along x):
    row r sits at y = phase + r * spacing (r need not be whole: a line
    halfway between two rows has r = k + 0.5);
    its points sit at x = offset + m * step for whole m, with
    offset = ((r / stagger) % 1) * step.
So each row's points shift by step / stagger from the row below and repeat
every `stagger` rows. The lattice is anchored to the origin, not to the
shape, so two regions sharing spacing, angle and phase line up.

Last, a point closer than `min_step` to the one kept before it is dropped:
the path then runs straight from that one to the next. The first and the
last point always stay.

The C++ port is native/resample_on_lattice.cpp; this module is its
reference, and the arithmetic here is written to match it bit for bit:
lengths are sqrt(dx*dx + dy*dy), sums run left to right.
"""

import math

DEFAULT_TOL = 1e-9


def _dist(a, b):
    """sqrt, not math.dist: the C++ port must match to the last bit."""
    dx, dy = a[0] - b[0], a[1] - b[1]
    return math.sqrt(dx * dx + dy * dy)


def resample_on_lattice(runs, spacing, step, angle=0.0, phase=0.0, stagger=4.0,
                        free_step=None, skip_row_ends=False, min_step=None, tol=DEFAULT_TOL):
    """The resampled points of a path given as runs, in order.

    runs: [(points, free)], points a list of (x, y), free a bool. Each run
        is expected to start where the one before it ended.
    spacing, angle, phase: the rows' spacing, direction (radians) and
        offset, so the rows found are the rows the path was laid on.
    step: point distance along a row, and the longest step of an evenly
        split segment in a lattice run.
    stagger: rows before the lattice repeats. Need not be whole; > 0.
    free_step: the longest step along a free run. None means step.
    skip_row_ends: leave out a row's last point where the run goes on: the
        path goes from the row's last lattice point straight on.
    min_step: the shortest step kept. None means spacing: it drops a
        half-spacing climb and keeps a full one. 0 keeps every point.
    """
    if spacing <= 0.0:
        raise ValueError("spacing must be greater than 0")
    if step <= 0.0:
        raise ValueError("step must be greater than 0")
    if stagger <= 0.0:
        raise ValueError("stagger must be greater than 0")
    if free_step is None:
        free_step = step
    if free_step <= 0.0:
        raise ValueError("free_step must be greater than 0")
    if min_step is None:
        min_step = spacing
    if min_step < 0.0:
        raise ValueError("min_step cannot be negative")

    c, s = math.cos(-angle), math.sin(-angle)
    out = []
    for points, free in runs:
        local = _drop_repeats([(c * p[0] - s * p[1], s * p[0] + c * p[1]) for p in points], tol)
        if not local:
            continue
        if out and _dist(out[-1], local[0]) > tol:
            out.append(local[0])
        elif not out:
            out.append(local[0])
        if len(local) < 2:
            continue
        if free:
            out.extend(_along(local, free_step))
            out.append(local[-1])
        else:
            out.extend(_lattice_run(local, spacing, step, phase, stagger, skip_row_ends, tol))

    if not out:
        return []
    out = _merge_short(out, min_step, tol)
    cb, sb = math.cos(angle), math.sin(angle)
    return [(cb * x - sb * y, sb * x + cb * y) for x, y in out]


def _drop_repeats(pts, tol):
    out = []
    for p in pts:
        if not out or _dist(p, out[-1]) > tol:
            out.append(p)
    return out


def _lattice_run(local, spacing, step, phase, stagger, skip_row_ends, tol):
    """Points after local[0] along one lattice run."""
    out = []
    last = len(local) - 2
    for i, (a, b) in enumerate(zip(local, local[1:])):
        dx, dy = abs(b[0] - a[0]), abs(b[1] - a[1])
        if dy <= tol:
            out.extend(_on_lattice(a, b, b[1], spacing, step, phase, stagger, tol))
            if not (skip_row_ends and i < last):
                out.append(b)
        elif dy <= spacing + tol and dx > dy:
            out.extend(_on_lattice(a, b, b[1], spacing, step, phase, stagger, tol))
            out.append(b)
        else:
            out.extend(_even(a, b, step))
            out.append(b)
    return out


def _on_lattice(a, b, y_row, spacing, step, phase, stagger, tol):
    """Points strictly between a and b where ab crosses the lattice columns
    of the row at y_row, in order from a."""
    r = (y_row - phase) / spacing
    offset = ((r / stagger) % 1.0) * step
    lo, hi = min(a[0], b[0]), max(a[0], b[0])
    m = math.floor((lo - offset) / step) + 1
    out = []
    while True:
        x = offset + m * step
        if x >= hi - tol:
            break
        if x > lo + tol:
            t = (x - a[0]) / (b[0] - a[0])
            out.append((x, a[1] + t * (b[1] - a[1])))
        m += 1
    if a[0] > b[0]:
        out.reverse()
    return out


def _even(a, b, step):
    """Points strictly between a and b, splitting ab into equal steps no
    longer than step."""
    n = int(math.ceil(_dist(a, b) / step - 1e-9))
    return [(a[0] + (b[0] - a[0]) * k / n, a[1] + (b[1] - a[1]) * k / n) for k in range(1, n)]


def _along(pts, free_step):
    """Points strictly between the ends of polyline pts, evenly spaced by
    length along it, no further apart than free_step."""
    lengths = [_dist(p, q) for p, q in zip(pts, pts[1:])]
    total = 0.0
    for length in lengths:
        total += length
    n = int(math.ceil(total / free_step - 1e-9))
    out = []
    seg, walked = 0, 0.0
    for k in range(1, n):
        target = total * k / n
        while seg < len(lengths) - 1 and walked + lengths[seg] < target:
            walked += lengths[seg]
            seg += 1
        t = (target - walked) / lengths[seg] if lengths[seg] > 0.0 else 0.0
        a, b = pts[seg], pts[seg + 1]
        out.append((a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])))
    return out


def _merge_short(points, min_step, tol):
    """Drop points closer than min_step to the one kept before them. The
    last point always stays: if it is too close, the one before it goes."""
    if min_step <= tol:
        return _drop_repeats(points, tol)
    out = [points[0]]
    for p in points[1:-1]:
        if _dist(p, out[-1]) >= min_step - tol:
            out.append(p)
    end = points[-1]
    if len(out) > 1 and _dist(end, out[-1]) < min_step - tol:
        out.pop()
    if _dist(end, out[-1]) > tol:
        out.append(end)
    return out
