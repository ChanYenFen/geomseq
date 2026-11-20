"""Coverage path planning: one continuous stitch path over a region.

Four stages, each usable on its own: decompose (cut the region into
monotone cells), fill_cells (fill lines per cell, linked into one polyline
per cell, walkable four ways; hatch_region does both),
travel (join two points down the middle of the cells, hidden under later
stitching where it can be), plan (order the cells and their walks into
one path). No trims anywhere: a travel always exists.

Pure Python, no Rhino. The region is a list of closed rings in plane
coordinates, read with the even-odd rule, so holes need no special marking
and nesting any depth works.

The cut is a boustrophedon decomposition along a sweep direction: every
scanline perpendicular to the direction crosses a cell in exactly one
interval, so a zig-zag fill of a cell never leaves it. Cells meet across
*portals*, the open intervals where two cells share a scanline; a travel
move between cells goes through portals, and a cell touching another only
at a point has no portal to it.

How the cut is made. All vertex heights (after rotating the sweep direction
onto +y) are the slab boundaries. Inside one slab no edge starts or ends, so
the spans on every scanline are the same two edges, and the slab's spans are
read once, at its midpoint. That midpoint is strictly between two distinct
vertex heights, so no scanline ever passes through a vertex or along a
horizontal edge: the usual degeneracies cannot arise. Each span is a
trapezoid; trapezoids in consecutive slabs are the same cell when they
overlap on the shared boundary one-to-one, and a portal otherwise.
"""

import heapq
import math
import random

# Vertex heights closer than this (in input units) are one slab boundary;
# a span or an overlap narrower than this is nothing.
DEFAULT_TOL = 1e-9


class Cell(object):
    """One monotone cell.

    `outline` is its boundary polygon in the input frame, counter-clockwise,
    first vertex not repeated. `rows` is the cell in the sweep frame: a list
    of (y, x_left, x_right) at every slab boundary it spans, bottom to top,
    with a boundary listed twice where the cell steps sideways across it
    (a horizontal edge). `rows` is what the fill stage will walk.
    """

    __slots__ = ("outline", "rows")

    def __init__(self, outline, rows):
        self.outline = outline
        self.rows = rows


class Portal(object):
    """An open interval of scanline shared by two cells, in the input frame."""

    __slots__ = ("cell_a", "cell_b", "p0", "p1")

    def __init__(self, cell_a, cell_b, p0, p1):
        self.cell_a, self.cell_b = cell_a, cell_b
        self.p0, self.p1 = p0, p1


class Decomposition(object):
    """`rings` is the region boundary as given, closing vertex stripped:
    what a travel keeps its clearance from."""

    __slots__ = ("cells", "portals", "angle", "rings")

    def __init__(self, cells, portals, angle, rings):
        self.cells = cells
        self.portals = portals
        self.angle = angle
        self.rings = rings

    def neighbours(self, i):
        """Indices of the cells that share a portal with cell i."""
        out = []
        for p in self.portals:
            if p.cell_a == i:
                out.append(p.cell_b)
            elif p.cell_b == i:
                out.append(p.cell_a)
        return out


# --- geometry helpers --------------------------------------------------------

def polygon_area(ring):
    """Signed shoelace area; positive for counter-clockwise."""
    n = len(ring)
    s = 0.0
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        s += x0 * y1 - x1 * y0
    return 0.5 * s


def region_area(rings):
    """Area under the even-odd rule: the absolute areas of the rings add
    or cancel by nesting parity, which the sum of signed areas gives only
    when holes are oriented against their outer ring. Done by parity here so
    orientation does not matter."""
    total = 0.0
    for i, ring in enumerate(rings):
        depth = 0
        probe = ring[0]
        for j, other in enumerate(rings):
            if j != i and _point_in_ring(probe, other):
                depth += 1
        a = abs(polygon_area(ring))
        total += a if depth % 2 == 0 else -a
    return total


def _point_in_ring(pt, ring):
    """Crossing count, used only for the nesting parity of region_area.
    A probe sitting on another ring's boundary counts as outside."""
    x, y = pt
    inside = False
    n = len(ring)
    for i in range(n):
        x0, y0 = ring[i]
        x1, y1 = ring[(i + 1) % n]
        if (y0 > y) != (y1 > y):
            xi = x0 + (y - y0) * (x1 - x0) / (y1 - y0)
            if x < xi:
                inside = not inside
    return inside


def _rotate(pt, c, s):
    x, y = pt
    return (c * x - s * y, s * x + c * y)


def _strip_closing(ring):
    """A ring may repeat its first vertex last; the cut wants it once."""
    pts = [(float(p[0]), float(p[1])) for p in ring]
    if len(pts) > 1 and pts[0] == pts[-1]:
        pts.pop()
    return pts


# --- the cut -----------------------------------------------------------------

def decompose(rings, angle=0.0, tol=DEFAULT_TOL):
    """Cut the region bounded by `rings` into monotone cells.

    rings: list of closed polygons, each a list of (x, y). Even-odd rule.
    angle: direction of the scanlines in radians, i.e. the direction the
        fill lines of a cell will run; the sweep advances perpendicular to
        it. 0 gives horizontal scanlines, cells monotone in y.
    Returns a Decomposition in the input frame.
    """
    if tol < 0.0:
        raise ValueError("tol cannot be negative")

    # Sweep frame: rotate by -angle so the sweep direction is +y.
    c, s = math.cos(-angle), math.sin(-angle)
    edges = []
    clean_rings = []
    for ring in rings:
        clean = _strip_closing(ring)
        if len(clean) < 3:
            raise ValueError("a ring needs at least three distinct vertices")
        clean_rings.append(clean)
        pts = [_rotate(p, c, s) for p in clean]
        for i in range(len(pts)):
            edges.append((pts[i], pts[(i + 1) % len(pts)]))

    levels = _slab_levels(edges, tol)
    trapezoids = [_slab_spans(edges, levels[k], levels[k + 1], tol)
                  for k in range(len(levels) - 1)]

    cells_rows, portal_rows = _link_slabs(trapezoids, levels, tol)

    # Back to the input frame.
    cb, sb = math.cos(angle), math.sin(angle)
    cells = []
    for rows in cells_rows:
        outline = [_rotate(p, cb, sb) for p in _rows_to_outline(rows)]
        cells.append(Cell(outline, rows))
    portals = []
    for a, b, y, x0, x1 in portal_rows:
        portals.append(Portal(a, b, _rotate((x0, y), cb, sb), _rotate((x1, y), cb, sb)))
    return Decomposition(cells, portals, angle, clean_rings)


def _slab_levels(edges, tol):
    """Distinct vertex heights, ascending; heights within tol are one level."""
    ys = sorted(set(p[1] for e in edges for p in e))
    levels = [ys[0]]
    for y in ys[1:]:
        if y - levels[-1] > tol:
            levels.append(y)
    if len(levels) < 2:
        raise ValueError("the region has no height along the sweep direction")
    return levels


def _x_at(edge, y):
    (x0, y0), (x1, y1) = edge
    return x0 + (y - y0) * (x1 - x0) / (y1 - y0)


def _slab_spans(edges, y_lo, y_hi, tol):
    """The trapezoids of one slab: a list of (xl_lo, xr_lo, xl_hi, xr_hi),
    left to right, each the span's x-interval at the slab's bottom and top.

    Read at the slab midpoint, where no vertex can lie, so each crossing edge
    is strictly crossed. Even-odd pairing of the sorted crossings gives the
    spans; a span thinner than tol (two rings touching along a line) is
    dropped.
    """
    y_mid = 0.5 * (y_lo + y_hi)
    crossing = []
    for e in edges:
        (_, ya), (_, yb) = e
        if (ya < y_mid) != (yb < y_mid):
            crossing.append((_x_at(e, y_mid), e))
    crossing.sort(key=lambda t: t[0])
    if len(crossing) % 2:
        raise ValueError("a scanline crossed the boundary an odd number of "
                         "times; the rings are not closed or they cross each other")

    # Pairs that touch end to start are one span: two rings sharing an edge
    # cancel it under even-odd, and the edge is interior.
    pairs = []
    for i in range(0, len(crossing), 2):
        (xm0, e0), (xm1, e1) = crossing[i], crossing[i + 1]
        if pairs and xm0 - pairs[-1][1] <= tol:
            pairs[-1] = (pairs[-1][0], xm1, pairs[-1][2], e1)
        else:
            pairs.append((xm0, xm1, e0, e1))

    spans = []
    for xm0, xm1, e0, e1 in pairs:
        if xm1 - xm0 <= tol:
            continue
        spans.append((_x_at(e0, y_lo), _x_at(e1, y_lo),
                      _x_at(e0, y_hi), _x_at(e1, y_hi)))
    return spans


def _link_slabs(trapezoids, levels, tol):
    """Merge trapezoids into cells and record portals.

    trapezoids[k] are the spans of the slab between levels[k] and
    levels[k + 1]. Returns (cells_rows, portals): cells_rows[i] is the row
    list of cell i (see Cell.rows); portals holds (cell_a, cell_b, y, x0, x1)
    with cell_a the lower cell.
    """
    # cell_of[k][j]: the cell that trapezoid j of slab k belongs to.
    cell_of = [[-1] * len(tz) for tz in trapezoids]
    cells_rows = []
    portals = []

    for k, here in enumerate(trapezoids):
        below = trapezoids[k - 1] if k > 0 else []
        y_lo, y_hi = levels[k], levels[k + 1]

        # Across the boundary at y_lo: overlaps[j] lists (i, x0, x1) for the
        # trapezoids i below whose top overlaps the bottom of j; above[i]
        # lists the j each i overlaps. A touch at a point is no overlap.
        overlaps = [[] for _ in here]
        above = [[] for _ in below]
        for i, (_, _, bl, br) in enumerate(below):
            for j, (hl, hr, _, _) in enumerate(here):
                x0, x1 = max(bl, hl), min(br, hr)
                if x1 - x0 > tol:
                    overlaps[j].append((i, x0, x1))
                    above[i].append(j)

        for j, (xl_lo, xr_lo, xl_hi, xr_hi) in enumerate(here):
            one_to_one = (len(overlaps[j]) == 1 and len(above[overlaps[j][0][0]]) == 1)
            if one_to_one:
                cell = cell_of[k - 1][overlaps[j][0][0]]
                cell_of[k][j] = cell
                rows = cells_rows[cell]
                # The boundary row is already there from below. Repeat it
                # only where the cell steps sideways, along a horizontal edge.
                _, pl, pr = rows[-1]
                if abs(pl - xl_lo) > tol or abs(pr - xr_lo) > tol:
                    rows.append((y_lo, xl_lo, xr_lo))
                rows.append((y_hi, xl_hi, xr_hi))
            else:
                cell_of[k][j] = len(cells_rows)
                cells_rows.append([(y_lo, xl_lo, xr_lo), (y_hi, xl_hi, xr_hi)])
                for i, x0, x1 in overlaps[j]:
                    portals.append((cell_of[k - 1][i], cell_of[k][j], y_lo, x0, x1))

    return cells_rows, portals


# --- the fill ----------------------------------------------------------------

FORWARD, REVERSE, MIRRORED, MIRRORED_REVERSE = 0, 1, 2, 3

# How a cell's fill lines are linked into one polyline.
#   ZIGZAG   lines alternate direction; each joins the next along the
#            cell's side, so the joins lie on the boundary.
#   ONE_WAY  every line runs the same direction; each joins the next by
#            the shortest way across the cell, from the end of one line
#            to the start of the next (a straight diagonal unless a side
#            bends into its way).
ZIGZAG, ONE_WAY = 0, 1


class Fill(object):
    """The polyline that covers one cell, in the input frame.

    A cell can be walked four ways, `walks[direction]`:
      FORWARD           bottom to top in the sweep, from the left end of
                        the lowest line
      REVERSE           FORWARD walked the other way
      MIRRORED          bottom to top, from the right end of the lowest line
      MIRRORED_REVERSE  MIRRORED walked the other way
    All four lay the same lines; they differ in which corner they enter at
    and which they leave from, which is what the order of cells chooses
    between. A walk's entry is its first point and its exit its last.
    Linked ONE_WAY, every walk leaves at the corner across from where it
    entered; linked ZIGZAG, that depends on the line count.
    `lines` are the fill lines themselves, bottom to top, each as its
    (left end, right end): what a travel is routed along the middle of.
    """

    __slots__ = ("cell", "walks", "lines")

    def __init__(self, cell, forward, mirrored, lines):
        self.cell = cell
        self.walks = (forward, forward[::-1], mirrored, mirrored[::-1])
        self.lines = lines

    @property
    def forward(self):
        return self.walks[FORWARD]

    @property
    def reverse(self):
        return self.walks[REVERSE]

    @property
    def line_count(self):
        return len(self.lines)


def fill_cells(dec, spacing, phase=0.0, link=ZIGZAG, tol=DEFAULT_TOL):
    """Lay fill lines across every cell of a Decomposition and link each
    cell's lines into one polyline.

    spacing: distance between lines, in input units. Must be > 0.
    phase: offset of the line grid along the sweep. Lines sit at
        phase + k * spacing for integer k, in the sweep frame, for every
        cell alike, so lines of neighbouring cells are collinear across a
        portal and the pattern reads as one. A cell too thin to catch a
        grid line gets one line through its middle.
    A grid line exactly on a portal belongs to the cell above it; the cell
    below stops short of it, or that row would be stitched twice.
    link: ZIGZAG or ONE_WAY, see their definitions. The lines are the same
        either way; only the joins between them differ.
    Returns one Fill per cell, in cell order.
    """
    if spacing <= 0.0:
        raise ValueError("spacing must be greater than 0")
    if link not in (ZIGZAG, ONE_WAY):
        raise ValueError("link must be ZIGZAG (0) or ONE_WAY (1)")
    cb, sb = math.cos(dec.angle), math.sin(dec.angle)
    has_above = set(p.cell_a for p in dec.portals)
    fills = []
    for i, cell in enumerate(dec.cells):
        drop_top = i in has_above
        pts, lines = _zigzag(cell.rows, spacing, phase, tol, True, drop_top)
        mirrored, _ = _zigzag(cell.rows, spacing, phase, tol, False, drop_top)
        if link == ONE_WAY:
            pts = _one_way(cell.rows, lines, True, tol)
            mirrored = _one_way(cell.rows, lines, False, tol)
        fills.append(Fill(i,
                          [_rotate(p, cb, sb) for p in pts],
                          [_rotate(p, cb, sb) for p in mirrored],
                          [(_rotate((xl, y), cb, sb), _rotate((xr, y), cb, sb))
                           for y, xl, xr in lines]))
    return fills


def hatch_region(rings, spacing, angle=0.0, phase=0.0, link=ZIGZAG, tol=DEFAULT_TOL):
    """Cut a region into monotone cells and hatch each: decompose followed
    by fill_cells, with the same arguments. Returns (dec, fills): dec.cells
    are the cells, fills[i].lines cell i's fill lines, fills[i].walks its
    linked polyline, four ways."""
    dec = decompose(rings, angle=angle, tol=tol)
    return dec, fill_cells(dec, spacing, phase=phase, link=link, tol=tol)


def _interval_at(rows, y):
    """The cell's x-interval on the scanline y, from its rows. At a boundary
    the cell steps across, the interval is the union of both sides."""
    xl, xr = None, None
    for k in range(len(rows) - 1):
        y0, l0, r0 = rows[k]
        y1, l1, r1 = rows[k + 1]
        if y < y0 or y > y1:
            continue
        if y1 == y0:
            a, b = min(l0, l1), max(r0, r1)
        else:
            t = (y - y0) / (y1 - y0)
            a, b = l0 + t * (l1 - l0), r0 + t * (r1 - r0)
        xl = a if xl is None else min(xl, a)
        xr = b if xr is None else max(xr, b)
    return xl, xr


def _zigzag(rows, spacing, phase, tol, start_left=True, drop_top=False):
    """Fill one cell in the sweep frame: (points, lines), lines as (y, xl, xr).

    Lines are the grid lines inside the cell's height, bottom to top,
    walked in alternating directions, the lowest one left to right when
    start_left, else right to left. Between two lines the
    polyline follows the cell's side chain through any row vertices in
    between, so it stays inside a side that bends inward. drop_top leaves
    out a line exactly on the cell's top.
    """
    y_bot, y_top = rows[0][0], rows[-1][0]
    k0 = int(math.ceil((y_bot - phase) / spacing - 1e-9))
    if drop_top:
        k1 = int(math.ceil((y_top - phase) / spacing - 1e-9)) - 1
    else:
        k1 = int(math.floor((y_top - phase) / spacing + 1e-9))
    ys = [phase + k * spacing for k in range(k0, k1 + 1)]
    ys = [min(max(y, y_bot), y_top) for y in ys]

    lines = []
    for y in ys:
        xl, xr = _interval_at(rows, y)
        if xl is not None and xr - xl > tol:
            lines.append((y, xl, xr))
    if not lines:
        y = 0.5 * (y_bot + y_top)
        xl, xr = _interval_at(rows, y)
        if xl is None or xr - xl <= tol:
            return [], []
        lines.append((y, xl, xr))

    pts = []
    for n, (y, xl, xr) in enumerate(lines):
        left_to_right = (n % 2 == 0) == start_left
        if n > 0:
            # Side chain from the previous line's end up to this one's start.
            y_prev = lines[n - 1][0]
            # This line starts where the last one ended: on the left if it
            # runs left to right. Index into (y, xl, xr).
            side = 1 if left_to_right else 2
            start = (xl, y) if left_to_right else (xr, y)
            for row in rows:
                if y_prev < row[0] < y:
                    _add_corner(pts, (row[side], row[0]), start, tol)
        if left_to_right:
            pts.append((xl, y))
            pts.append((xr, y))
        else:
            pts.append((xr, y))
            pts.append((xl, y))
    return pts, lines


def _add_corner(pts, v, nxt, tol):
    """Append corner v unless it is within tol of the last point or of nxt:
    a step of no length makes an invalid curve."""
    if not _same(v, pts[-1], tol) and not _same(v, nxt, tol):
        pts.append(v)


def _one_way(rows, lines, start_left, tol):
    """The lines walked all in one direction, bottom to top, left to right
    when start_left, each joined to the next by the shortest way through
    the cell. In the sweep frame, like _zigzag."""
    pts = []
    for n, (y, xl, xr) in enumerate(lines):
        a, b = ((xl, y), (xr, y)) if start_left else ((xr, y), (xl, y))
        if n > 0:
            for v in _shortest_in_cell(rows, pts[-1], a, tol)[1:-1]:
                _add_corner(pts, v, a, tol)
        pts.append(a)
        pts.append(b)
    return pts


def _narrowest(rows, y_lo, y_hi):
    """(y, xl, xr) at each row height strictly between y_lo and y_hi,
    keeping the narrower interval where the cell steps at that height."""
    out = {}
    for y, xl, xr in rows:
        if y_lo < y < y_hi:
            if y in out:
                lo, hi = out[y]
                out[y] = (max(lo, xl), min(hi, xr))
            else:
                out[y] = (xl, xr)
    return sorted((y, lo, hi) for y, (lo, hi) in out.items())


def _shortest_in_cell(rows, a, b, tol):
    """The shortest polyline from a to b inside a monotone cell, a below b
    (sweep frame). The sides are straight between row heights, so a
    segment is inside when it is inside at every row height it crosses,
    and the shortest path bends only at row vertices, climbing as it goes."""
    levels = _narrowest(rows, a[1], b[1])

    def clear(p, q):
        for y, lo, hi in levels:
            if p[1] < y < q[1]:
                x = p[0] + (q[0] - p[0]) * (y - p[1]) / (q[1] - p[1])
                if x < lo - tol or x > hi + tol:
                    return False
        return True

    if clear(a, b):
        return [a, b]
    nodes = [a]
    for y, lo, hi in levels:
        nodes.append((lo, y))
        nodes.append((hi, y))
    nodes.append(b)
    dist = [math.inf] * len(nodes)
    prev = [None] * len(nodes)
    dist[0] = 0.0
    # Nodes are in height order and the path climbs, so one pass settles all.
    for j in range(1, len(nodes)):
        for i in range(j):
            if dist[i] < math.inf and nodes[i][1] < nodes[j][1] and clear(nodes[i], nodes[j]):
                d = dist[i] + math.dist(nodes[i], nodes[j])
                if d < dist[j]:
                    dist[j], prev[j] = d, i
    path, j = [], len(nodes) - 1
    while j is not None:
        path.append(nodes[j])
        j = prev[j]
    return path[::-1]


def _rows_to_outline(rows):
    """Cell boundary from its rows: up the right side, down the left,
    counter-clockwise in the sweep frame."""
    pts = []
    for y, xl, xr in rows:
        pts.append((xr, y))
    for y, xl, xr in reversed(rows):
        pts.append((xl, y))
    # Consecutive duplicates appear where a row is listed twice without a
    # step on that side.
    out = []
    for p in pts:
        if not out or out[-1] != p:
            out.append(p)
    if len(out) > 1 and out[0] == out[-1]:
        out.pop()
    return out


# --- the travel --------------------------------------------------------------

def _same(a, b, tol):
    return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol


def _on_segment(p, a, b, tol):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        return _same(p, a, tol)
    t = min(max(((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2, 0.0), 1.0)
    return _same(p, (a[0] + t * dx, a[1] + t * dy), tol)


def _in_closed_polygon(p, ring, tol):
    """Inside or on the boundary."""
    n = len(ring)
    for i in range(n):
        if _on_segment(p, ring[i], ring[(i + 1) % n], tol):
            return True
    return _point_in_ring(p, ring)


def _inside_union(p, rings, tol):
    return any(_in_closed_polygon(p, r, tol) for r in rings)


def _segment_inside_union(a, b, rings, tol):
    """True if the whole segment ab lies in the closed union of rings.

    The segment is cut at every point where it meets a ring edge, so each
    piece lies wholly inside or wholly outside; the midpoint of each piece
    decides it. Collinear overlaps are cut at the edge endpoints.
    """
    if not (_inside_union(a, rings, tol) and _inside_union(b, rings, tol)):
        return False
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    if L <= tol:
        return True
    ts = _cuts(a, b, rings, tol)
    for t0, t1 in zip(ts, ts[1:]):
        if (t1 - t0) * L <= tol:
            continue
        tm = 0.5 * (t0 + t1)
        if not _inside_union((a[0] + tm * dx, a[1] + tm * dy), rings, tol):
            return False
    return True


def _cuts(a, b, rings, tol):
    """Sorted parameters along ab, 0 and 1 included, where it meets a ring
    edge; between two consecutive ones the segment does not cross any."""
    return _cut_params(a, b, ((r[i], r[(i + 1) % len(r)]) for r in rings for i in range(len(r))), tol)


def _cut_params(a, b, edges, tol):
    """As _cuts, against any iterable of edges (p, q)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    ts = [0.0, 1.0]
    for p, q in edges:
        ex, ey = q[0] - p[0], q[1] - p[1]
        ap = (p[0] - a[0], p[1] - a[1])
        denom = dx * ey - dy * ex
        if abs(denom) > tol * L * math.hypot(ex, ey):
            t = (ap[0] * ey - ap[1] * ex) / denom
            u = (ap[0] * dy - ap[1] * dx) / denom
            if 0.0 <= t <= 1.0 and 0.0 <= u <= 1.0:
                ts.append(t)
        elif abs(ap[0] * dy - ap[1] * dx) <= tol * L:
            for r in (p, q):
                t = ((r[0] - a[0]) * dx + (r[1] - a[1]) * dy) / (L * L)
                if 0.0 <= t <= 1.0:
                    ts.append(t)
    ts.sort()
    return ts


def _dijkstra(nodes, adj, src, dst, zero=(0.0,)):
    """Cheapest path from node src to node dst, as a list of points, or None.

    Weights are tuples of non-negative numbers, added term by term and
    compared in order: the first term decides, the next only breaks ties.
    """
    n = len(nodes)
    dist = [None] * n
    prev = [-1] * n
    dist[src] = zero
    heap = [(zero, src)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist[u]:
            continue
        if u == dst:
            break
        for v, w in adj[u]:
            nd = tuple(x + y for x, y in zip(d, w))
            if dist[v] is None or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd, v))
    if dist[dst] is None:
        return None
    path = []
    u = dst
    while u != -1:
        path.append(nodes[u])
        u = prev[u]
    return path[::-1]


def _mid(a, b):
    return (0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1]))


def _step_targets(e, line, tol):
    """Where a step from e across to line may land: the foot of the
    perpendicular, then the foot moved toward the line's middle by the
    step's own length (a 45 degree step). Empty when e is not over line."""
    foot = _foot_on_segment(e, line[0], line[1], tol)
    if foot is None:
        return []
    gap = math.dist(e, foot)
    m = _mid(*line)
    toward = math.dist(foot, m)
    if gap <= tol or toward <= tol:
        return [foot]
    s = min(gap, toward) / toward
    return [foot, (foot[0] + s * (m[0] - foot[0]), foot[1] + s * (m[1] - foot[1]))]


def _foot_on_segment(p, a, b, tol):
    """The foot of the perpendicular from p to segment ab, or None when it
    falls outside the segment."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 == 0.0:
        return None
    t = ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2
    slack = tol / math.sqrt(L2)
    if t < -slack or t > 1.0 + slack:
        return None
    t = min(max(t, 0.0), 1.0)
    return (a[0] + t * dx, a[1] + t * dy)


def _shown(pieces, hidden):
    """Length of the pieces no cell in hidden covers."""
    s = 0.0
    for length, cells in pieces:
        if cells.isdisjoint(hidden):
            s += length
    return s


def _search(adj, src, hidden, targets=()):
    """Dijkstra from src over links (node, length, pieces, via), cost
    (visible length, length) for the given hidden set; via holds the points
    a contracted link passes through on the way. Stops once every node in
    targets is settled, if any are given. Returns (dist, prev), prev[v]
    being (u, via) for the link that reached v."""
    n = len(adj)
    dist = [None] * n
    prev = [None] * n
    dist[src] = (0.0, 0.0)
    heap = [(0.0, 0.0, src)]
    left = set(targets)
    while heap:
        vis, length_so_far, u = heapq.heappop(heap)
        if (vis, length_so_far) > dist[u]:
            continue
        if left:
            left.discard(u)
            if not left:
                break
        for v, length, pieces, via in adj[u]:
            # _shown, inlined: this loop is where a plan spends its time.
            shown = 0.0
            for piece_length, cells in pieces:
                if cells.isdisjoint(hidden):
                    shown += piece_length
            nd = (vis + shown, length_so_far + length)
            if dist[v] is None or nd < dist[v]:
                dist[v] = nd
                prev[v] = (u, via)
                heapq.heappush(heap, (nd[0], nd[1], v))
    return dist, prev


def _merge_pieces(runs):
    """Pieces of consecutive links as one list, neighbours in the same cells
    merged."""
    out = []
    for pieces in runs:
        for length, cells in pieces:
            if out and out[-1][1] == cells:
                out[-1] = (out[-1][0] + length, cells)
            else:
                out.append((length, cells))
    return out


def _walk_back(nodes, prev, dst):
    path = [nodes[dst]]
    u = dst
    while prev[u] is not None:
        u, via = prev[u]
        path.extend(reversed(via))
        path.append(nodes[u])
    return path[::-1]


class _Router(object):
    """The travel graph of one decomposition, built once.

    Nodes are the midpoints of the fill lines and of the portals. Each link
    is kept as its pieces, (length, cells the piece lies in), cut wherever
    it crosses a cell edge; the visible length for any set of unstitched
    cells is then a sum over pieces rather than a geometry query, so the
    one graph serves every step of an order.
    """

    def __init__(self, dec, fills, tol):
        self.tol = tol
        self.fills = fills
        self.region = [c.outline for c in dec.cells]
        self.nodes, self.adj = [], []
        self.mids = []  # (node, cell, line) for attaching ends
        self._attached = {}
        self._pieces = {}
        self._costs = {}
        self._gaps = {}
        self._corners = None
        # No travel passes through a corner of the outline: there a thread
        # along the edge shows. A corner may still end a segment (the
        # fallback's nodes are corners, and a fill may end on one).
        self.outline_corners = [v for r in dec.rings for v in r]

        # Every geometry query is filtered by bounding boxes first: a curved
        # outline flattens to thousands of edges, and a query that meets
        # them all made one plan take minutes.
        self._boxes = []   # per cell: (xmin, ymin, xmax, ymax)
        self._edges = []   # per cell: [(p, q, xmin, ymin, xmax, ymax)]
        for ring in self.region:
            xs = [v[0] for v in ring]
            ys = [v[1] for v in ring]
            self._boxes.append((min(xs) - tol, min(ys) - tol, max(xs) + tol, max(ys) + tol))
            edges = []
            for i in range(len(ring)):
                p, q = ring[i], ring[(i + 1) % len(ring)]
                edges.append((p, q, min(p[0], q[0]) - tol, min(p[1], q[1]) - tol,
                              max(p[0], q[0]) + tol, max(p[1], q[1]) + tol))
            self._edges.append(edges)

        # A fill line may sit on the region boundary itself (the grid landed
        # on a cell's top or bottom row, and that row is a hole's edge); its
        # midpoint is then on the boundary and is not a place a travel may be.
        ring_edges = [(r[i], r[(i + 1) % len(r)]) for r in dec.rings for i in range(len(r))]

        def on_boundary(p):
            return any(_on_segment(p, a, b, tol) for a, b in ring_edges)

        # Midline of each cell: its line midpoints, chained.
        first, last = {}, {}
        chain = {}  # cell -> its midline nodes, bottom line first
        self._anchors = set()  # nodes the contraction must keep
        for c in range(len(dec.cells)):
            prev_node = None
            for k, (a, b) in enumerate(fills[c].lines):
                m = _mid(a, b)
                if on_boundary(m):
                    continue
                node = self._add_node(m)
                self.mids.append((node, c, k))
                chain.setdefault(c, []).append(node)
                if prev_node is not None:
                    self._link(prev_node, node)
                else:
                    first[c] = node
                prev_node = node
            if prev_node is not None:
                last[c] = prev_node
                self._anchors.update((first[c], last[c]))

        # Portals join the lower cell's last line to the upper cell's first,
        # through the portal's midpoint. Each cell also keeps its ways out,
        # up and down (the portal's midpoint, and the neighbour's line next
        # to it), so an end on its top or bottom line can leave across the
        # gap to the portal instead of back across the cell.
        # A link into a cell goes to the nearest midline node it can reach:
        # a line lying on the portal may only be reachable through a corner.
        def link_in(node, cell, from_top):
            nodes = chain[cell][::-1] if from_top else chain[cell]
            for target in nodes:
                if self._link(node, target):
                    return target
            return nodes[0]

        self.ways_up, self.ways_down = {}, {}
        for p in dec.portals:
            if p.cell_a in last and p.cell_b in first:
                node = self._add_node(_mid(p.p0, p.p1))
                self._anchors.add(node)
                below = link_in(node, p.cell_a, True)
                above = link_in(node, p.cell_b, False)
                self.ways_up.setdefault(p.cell_a, []).extend([node, above])
                self.ways_down.setdefault(p.cell_b, []).extend([node, below])

        # Across a portal, the zig-zag can simply carry on: from each end
        # of the lower cell's top line straight across to the upper cell's
        # bottom line, and back down from the ends of that one. The step is
        # the fill's own step between rows, so it counts as part of both
        # cells' fill: hidden while either is still to be stitched. A walk
        # that ends by a portal then leaves without showing anything; one
        # that ends across the cell from it still shows the way back.
        # Every walk's entry and exit is such an end, so the hops are keyed
        # by the point. Only a short step counts: the end must lie over the
        # other line (else the step is a diagonal across a cell). Straight
        # across first; where that runs through a corner of the outline (an
        # end beside a notch), slanted 45 degrees inward instead.
        self.hops = {}
        for p in dec.portals:
            below, above = p.cell_a, p.cell_b
            if below not in last or above not in first:
                continue
            for ends, line, cell, from_top in ((fills[below].lines[-1], fills[above].lines[0], above, False),
                                               (fills[above].lines[0], fills[below].lines[-1], below, True)):
                for e in ends:
                    for q in _step_targets(e, line, tol):
                        step = self.pieces(e, q)
                        if step is None:
                            continue
                        both = frozenset([below, above]).union(*[cells for _, cells in step])
                        key = (e, q) if e <= q else (q, e)
                        self._pieces[key] = [(sum(length for length, _ in step), both)]
                        node = self._add_node(q)
                        link_in(node, cell, from_top)
                        self.hops.setdefault(e, []).append(node)
                        self._anchors.add(node)
                        break
        self._contract()

    def _contract(self):
        """Fold each run of plain midline nodes (two links, nothing else
        attached) into one link between the nodes at its ends, carrying the
        run's points as via and its pieces end to end. Searches then run
        over a few nodes per cell instead of one per fill line, and find the
        same costs. An end attached to a folded node unfolds it for that
        search (see route), through self.folded."""
        n = len(self.nodes)
        keep = [len(self.adj[i]) != 2 or i in self._anchors for i in range(n)]
        self.cadj = [[] for _ in range(n)]
        self.folded = {}  # node -> [(end, length, pieces, via)] toward both ends
        for u in range(n):
            if not keep[u]:
                continue
            for v, length, pieces in self.adj[u]:
                run, lengths, piece_runs = [], [length], [pieces]
                prev, cur = u, v
                while not keep[cur] and cur not in run:
                    run.append(cur)
                    nxt = [e for e in self.adj[cur] if e[0] != prev]
                    if not nxt:
                        break
                    prev, cur = cur, nxt[0][0]
                    lengths.append(nxt[0][1])
                    piece_runs.append(nxt[0][2])
                if not keep[cur]:
                    continue  # a run with no kept node at its far end
                via = [self.nodes[x] for x in run]
                self.cadj[u].append((cur, sum(lengths), _merge_pieces(piece_runs), via))
                for i, x in enumerate(run):
                    # Toward u: links 0..i, through run[:i] in reverse.
                    self.folded.setdefault(x, []).append(
                        (u, sum(lengths[:i + 1]), _merge_pieces(piece_runs[:i + 1]),
                         [self.nodes[y] for y in reversed(run[:i])]))

    def _add_node(self, p):
        self.nodes.append(p)
        self.adj.append([])
        return len(self.nodes) - 1

    def _link(self, i, j):
        a, b = self.nodes[i], self.nodes[j]
        pieces = self.pieces(a, b)
        if pieces is None:
            return False
        length = math.dist(a, b)
        self.adj[i].append((j, length, pieces))
        self.adj[j].append((i, length, pieces))
        return True

    def pieces(self, a, b):
        """Segment ab as (length, cells) pieces, or None if it leaves the
        region. A piece on a shared edge lies in both cells. Remembered per
        segment, either way round."""
        key = (a, b) if a <= b else (b, a)
        if key not in self._pieces:
            self._pieces[key] = self._cut_into_pieces(*key)
        return self._pieces[key]

    def _cells_at(self, p):
        """Cells whose closed outline holds p."""
        x, y = p
        out = []
        for c, (x0, y0, x1, y1) in enumerate(self._boxes):
            if x0 <= x <= x1 and y0 <= y <= y1 and self._contains(c, p):
                out.append(c)
        return out

    def _contains(self, c, p):
        if _point_in_ring(p, self.region[c]):
            return True
        x, y = p
        for q0, q1, x0, y0, x1, y1 in self._edges[c]:
            if x0 <= x <= x1 and y0 <= y <= y1 and _on_segment(p, q0, q1, self.tol):
                return True
        return False

    def _cut_into_pieces(self, a, b):
        tol = self.tol
        if not self._cells_at(a) or not self._cells_at(b):
            return None
        bx0, by0 = min(a[0], b[0]) - tol, min(a[1], b[1]) - tol
        bx1, by1 = max(a[0], b[0]) + tol, max(a[1], b[1]) + tol
        for v in self.outline_corners:
            if (bx0 <= v[0] <= bx1 and by0 <= v[1] <= by1 and not _same(v, a, tol)
                    and not _same(v, b, tol) and _on_segment(v, a, b, tol)):
                return None
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L <= tol:
            return []
        near = []
        for c, (x0, y0, x1, y1) in enumerate(self._boxes):
            if x0 <= bx1 and bx0 <= x1 and y0 <= by1 and by0 <= y1:
                for q0, q1, ex0, ey0, ex1, ey1 in self._edges[c]:
                    if ex0 <= bx1 and bx0 <= ex1 and ey0 <= by1 and by0 <= ey1:
                        near.append((q0, q1))
        ts = _cut_params(a, b, near, tol)
        out = []
        for t0, t1 in zip(ts, ts[1:]):
            piece = (t1 - t0) * L
            if piece <= tol:
                continue
            tm = 0.5 * (t0 + t1)
            cells = frozenset(self._cells_at((a[0] + tm * dx, a[1] + tm * dy)))
            if not cells:
                return None
            out.append((piece, cells))
        return out

    def visible(self, path, hidden):
        return sum(_shown(self.pieces(a, b), hidden) for a, b in zip(path, path[1:]))

    def _attach(self, p):
        """Links from p into the midline: to the midpoint of the line p lies
        on (a fill's entry and exit do), and, when that is the cell's top
        or bottom line, straight to the cell's ways out on that side: a fill
        that ends next to a portal leaves through it rather than back across
        its own fresh rows. Else to the nearest midpoint in sight.
        Remembered per point: an order asks for the same walk ends again
        and again."""
        if p not in self._attached:
            self._attached[p] = self._find_attachments(p)
        return self._attached[p]

    def _find_attachments(self, p):
        targets = list(self.hops.get(p, []))
        for c, f in enumerate(self.fills):
            for k, (a, b) in enumerate(f.lines):
                if not _on_segment(p, a, b, self.tol):
                    continue
                if k == len(f.lines) - 1:
                    targets.extend(self.ways_up.get(c, []))
                if k == 0:
                    targets.extend(self.ways_down.get(c, []))
        for node, c, k in self.mids:
            a, b = self.fills[c].lines[k]
            if _on_segment(p, a, b, self.tol):
                targets.append(node)
        out = []
        for node in dict.fromkeys(targets):
            pieces = self.pieces(p, self.nodes[node])
            if pieces is not None:
                out.append((node, math.dist(p, self.nodes[node]), pieces))
        if out:
            return out
        for node, _, _ in sorted(self.mids, key=lambda m: math.dist(p, self.nodes[m[0]])):
            pieces = self.pieces(p, self.nodes[node])
            if pieces is not None:
                return [(node, math.dist(p, self.nodes[node]), pieces)]
        return out

    def route(self, start, goals, hidden, paths=True):
        """The cheapest travel from start to each goal, as (cost, path),
        cost (visible length, length); path None only when the goal is in
        another piece of the region, or when paths is False. No direct
        start-to-goal link: between two fill ends it could run along the
        boundary, which is the one place a travel must not be."""
        n0 = len(self.nodes)
        nodes = self.nodes + [start] + list(goals)
        adj = [list(a) for a in self.cadj] + [[] for _ in range(1 + len(goals))]
        unfolded = set()

        def unfold(x):
            if x in self.folded and x not in unfolded:
                unfolded.add(x)
                for end, length, pieces, via in self.folded[x]:
                    adj[x].append((end, length, pieces, via))
                    adj[end].append((x, length, pieces, via[::-1]))

        for node, length, pieces in self._attach(start):
            unfold(node)
            adj[n0].append((node, length, pieces, ()))
            adj[node].append((n0, length, pieces, ()))
        for g, goal in enumerate(goals):
            for node, length, pieces in self._attach(goal):
                unfold(node)
                adj[node].append((n0 + 1 + g, length, pieces, ()))  # into the goal only
        dist, prev = _search(adj, n0, hidden, range(n0 + 1, n0 + 1 + len(goals)))

        out = []
        for g, goal in enumerate(goals):
            if _same(start, goal, self.tol):
                out.append(((0.0, 0.0), [start, goal]))
            elif dist[n0 + 1 + g] is not None:
                path = _walk_back(nodes, prev, n0 + 1 + g) if paths else None
                out.append((dist[n0 + 1 + g], path))
            else:
                out.append(self._corner_route(start, goal, hidden))
        return out

    def costs(self, start, goals, hidden):
        """route's costs alone (None for an unreachable goal), remembered:
        an order search asks for the same exit, entries and stitched set
        many times over."""
        key = (start, goals, hidden)
        if key not in self._costs:
            self._costs[key] = [cost for cost, _ in self.route(start, list(goals), hidden, paths=False)]
        return self._costs[key]

    def _corner_route(self, start, goal, hidden):
        """The fallback, for ends the midline does not join (cells meeting
        only at a point have no portal): the same cost over the cells'
        corners, which passes wherever the region does. It hugs the
        boundary, which is why it is the fallback and not the rule."""
        tol = self.tol
        if self._corners is None:
            verts = []
            for ring in self.region:
                for v in ring:
                    if not any(_same(v, w, tol) for w in verts):
                        verts.append(v)
            adj = [[] for _ in verts]
            for i in range(len(verts)):
                for j in range(i + 1, len(verts)):
                    pieces = self.pieces(verts[i], verts[j])
                    if pieces is not None:
                        length = math.dist(verts[i], verts[j])
                        adj[i].append((j, length, pieces, ()))
                        adj[j].append((i, length, pieces, ()))
            self._corners = (verts, adj)
        verts, base = self._corners
        nodes = verts + [start, goal]
        adj = [list(a) for a in base] + [[], []]
        s, g = len(verts), len(verts) + 1
        for end in (s, g):
            for i, v in enumerate(verts):
                pieces = self.pieces(nodes[end], v)
                if pieces is not None:
                    length = math.dist(nodes[end], v)
                    adj[end].append((i, length, pieces, ()))
                    adj[i].append((end, length, pieces, ()))
        pieces = self.pieces(start, goal)
        if pieces is not None:
            adj[s].append((g, math.dist(start, goal), pieces, ()))
        dist, prev = _search(adj, s, hidden)
        if dist[g] is None:
            return None, None
        return dist[g], _walk_back(nodes, prev, g)

    def shortcut(self, path, hidden):
        """Cut the doubling back that routing through midpoints leaves.

        Going to a line's midpoint and coming back is pointless when a later
        point of the path is nearer than the next one: from each point, jump
        to the farthest later point that is nearer than the next hop, when
        the jump stays in the region and shows no more than the stretch it
        replaces. The bound keeps the jumps local, so the path does not grow
        a long run along the boundary.
        """
        shown = [0.0]
        for a, b in zip(path, path[1:]):
            shown.append(shown[-1] + _shown(self.pieces(a, b), hidden))
        out = [path[0]]
        i = 0
        while i < len(path) - 1:
            nxt = i + 1
            hop = math.dist(path[i], path[nxt])
            for j in range(len(path) - 1, i + 1, -1):
                if math.dist(path[i], path[j]) >= hop:
                    continue
                pieces = self.pieces(path[i], path[j])
                if pieces is not None and _shown(pieces, hidden) <= shown[j] - shown[i] + self.tol:
                    nxt = j
                    break
            # A portal's midpoint can coincide with a line's on it. The goal
            # replaces a point it coincides with rather than follow it.
            if not _same(path[nxt], out[-1], self.tol):
                out.append(path[nxt])
            elif nxt == len(path) - 1:
                if len(out) > 1:
                    out[-1] = path[nxt]
                else:
                    out.append(path[nxt])
            i = nxt
        return out


def travel(dec, fills, unstitched, start, goal, tol=DEFAULT_TOL):
    """A path from start to goal down the middle of the cells, hidden under
    later stitching wherever it can be. Never None on a connected region:
    a trim is not an option, so a travel that cannot be hidden is shown,
    as little of it as possible.

    The path runs through the midpoints of the fill lines: line to line
    within a cell, and from a cell's last line through the midpoint of a
    portal into the neighbour's first line. Every cell takes part. A link's
    cost is (visible length, length), compared in that order: visible is
    the part outside the unstitched cells, which no later stitching covers.
    So the path keeps to unstitched cells whenever they join start to goal
    (visible length 0), and otherwise crosses stitched ones by the least
    visible length the midline allows. It keeps to the middle of the
    fabric by construction: the fill lines say where the middle is.

    start and goal are joined to the midpoint of the fill line they lie on
    (a fill's entry and exit do), or else to the nearest line midpoint in
    sight. Every link is checked to lie inside the region, so a cell whose
    sides bend sharply between two lines cannot let the path out.

    When the midline does not join the two ends (cells that meet only at a
    point, which is not a portal), the path falls back to the corners of
    the cells, at the same cost. Raises ValueError only when start and goal
    lie in separate pieces of the region, where any travel leaves it.
    Returns the polyline, start first, goal last; visible_length() says how
    much of it shows.
    """
    if tol < 0.0:
        raise ValueError("tol cannot be negative")
    if _same(start, goal, tol):
        return [start, goal]
    router = _Router(dec, fills, tol)
    hidden = frozenset(unstitched)
    _, path = router.route(start, [goal], hidden)[0]
    if path is None:
        raise ValueError("start and goal lie in separate pieces of the region")
    return router.shortcut(path, hidden)


def visible_length(path, dec, fills, unstitched, tol=DEFAULT_TOL):
    """How much of a travel no later stitching covers: its length outside
    the unstitched cells, except a straight step across a portal between
    two cells' rows, which is the fill's own step and hidden while either
    cell is unstitched. The same rule travel and plan minimise. Raises
    ValueError if the path leaves the region."""
    router = _Router(dec, fills, tol)
    hidden = frozenset(unstitched)
    total = 0.0
    for a, b in zip(path, path[1:]):
        pieces = router.pieces(a, b)
        if pieces is None:
            raise ValueError("the path leaves the region")
        total += _shown(pieces, hidden)
    return total


def shortest_travel(dec, unstitched, start, goal, tol=DEFAULT_TOL):
    """The shortest path from start to goal inside the unstitched cells,
    regardless of where in the fabric it runs: the lower bound a travel's
    length is compared against, or None when they do not join the two.

    Exact: the shortest path in a polygon bends only at reflex vertices, so
    the visibility graph over the cell vertices, with start and goal added,
    contains it, and Dijkstra finds it. It hugs corners, which is why it is
    not the travel itself.
    """
    if tol < 0.0:
        raise ValueError("tol cannot be negative")
    rings = [dec.cells[i].outline for i in unstitched]
    if not rings:
        return None
    if _same(start, goal, tol):
        return [start]

    nodes = [start, goal]
    for ring in rings:
        for v in ring:
            if not any(_same(v, n, tol) for n in nodes):
                nodes.append(v)

    n = len(nodes)
    adj = [[] for _ in range(n)]
    for i in range(n):
        for j in range(i + 1, n):
            if _segment_inside_union(nodes[i], nodes[j], rings, tol):
                w = (math.hypot(nodes[i][0] - nodes[j][0], nodes[i][1] - nodes[j][1]),)
                adj[i].append((j, w))
                adj[j].append((i, w))
    return _dijkstra(nodes, adj, 0, 1)


# --- the order ---------------------------------------------------------------

FILL, TRAVEL = 0, 1

# Default restarts of the order search from a shaken copy of its best
# order. 0 keeps a 23-cell region near half a minute in Python; 20 reached
# the subset-DP optimum on every shape measured (5 to 11 cells), 5 left one
# 23% above it, each restart costing about one more search.
KICKS = 0


class Piece(object):
    """One stretch of the stitch path: a cell's fill, or a travel.

    kind       FILL or TRAVEL
    points     the polyline; it starts where the previous piece ended
    cell       the cell filled (FILL), or None
    direction  the walk the cell is filled in (FILL), or None
    visible    length of the travel no stitching covers (TRAVEL), else 0
    """

    __slots__ = ("kind", "points", "cell", "direction", "visible")

    def __init__(self, kind, points, cell=None, direction=None, visible=0.0):
        self.kind = kind
        self.points = points
        self.cell = cell
        self.direction = direction
        self.visible = visible

    @property
    def length(self):
        return sum(math.dist(a, b) for a, b in zip(self.points, self.points[1:]))


class Plan(object):
    """The whole stitch path: fills and travels in order, each piece
    starting where the last one ended.

    order  [(cell, direction)] in the order stitched
    pieces [Piece], fills and travels alternating where a travel is needed
    """

    __slots__ = ("order", "pieces")

    def __init__(self, order, pieces):
        self.order = order
        self.pieces = pieces

    @property
    def path(self):
        """Every piece joined into one polyline, no point repeated at a join."""
        out = []
        for piece in self.pieces:
            pts = piece.points
            if out and pts and _same(pts[0], out[-1], DEFAULT_TOL):
                pts = pts[1:]
            out.extend(pts)
        return out

    @property
    def visible_length(self):
        return sum(p.visible for p in self.pieces if p.kind == TRAVEL)

    @property
    def travel_length(self):
        return sum(p.length for p in self.pieces if p.kind == TRAVEL)

    @property
    def fill_length(self):
        return sum(p.length for p in self.pieces if p.kind == FILL)

    @property
    def cost(self):
        """What an order is judged by: visible length first, then travel."""
        return (self.visible_length, self.travel_length)


def plan(dec, fills, start=None, kicks=KICKS, tol=DEFAULT_TOL):
    """Order the cells and pick each one's walk, joining them with travels
    into one stitch path. No trims: every travel exists, and the order is
    chosen to show as little of them as possible, then to travel least.

    A travel is hidden only by the cells not stitched yet: over the cell
    just finished it shows like over any other, so a walk has to end next
    to the way on, not across the cell from it.

    Three steps. A greedy seed: from where the needle is, the cheapest next
    (cell, walk) by the travel to its entry. Then, for any order of cells,
    the best walk for every cell is exact: the stitched set at each step
    depends on the order alone, so the walks are a shortest path through
    four states per cell. A local search rearranges the order (a run of
    up to three cells moved, as it was or reversed; two cells swapped; a
    stretch reversed), keeping a change whenever that exact cost
    improves, until a whole round of them finds none; then, `kicks`
    times, it shakes the best order and searches again from there. Costs
    are (visible length, travel length), compared in order. The seed only
    has to be somewhere to start from: the search moves any cell to the
    front.

    start: where the needle starts, or None to let the plan choose.
    kicks: restarts of the search. Each costs about one more search; more
        finds shorter travel on larger regions, 0 stops at the first
        local optimum. Seeded, so the same input gives the same plan.
    Cells too thin to hold a fill line are left out. Returns a Plan.
    """
    if kicks < 0:
        raise ValueError("kicks cannot be negative")
    if tol < 0.0:
        raise ValueError("tol cannot be negative")
    if len(fills) != len(dec.cells):
        raise ValueError("fills must be one per cell of dec")
    router = _Router(dec, fills, tol)
    todo = [c for c in range(len(dec.cells)) if fills[c].lines]
    if not todo:
        return Plan([], [])
    first = None if start is not None else (todo[0], FORWARD)
    seed = _greedy(router, fills, todo, start, first)
    cells = [c for c, _ in seed.order]
    cells, _, walks = _improve(router, fills, cells, start, tol, kicks)
    return _assemble(router, fills, cells, walks, start)


def _greedy(router, fills, todo, start, first):
    order, pieces = [], []
    remaining = set(todo)
    pos = start

    def take(c, d):
        walk = fills[c].walks[d]
        order.append((c, d))
        pieces.append(Piece(FILL, list(walk), cell=c, direction=d))
        remaining.discard(c)
        return walk[-1]

    if first is not None:
        pos = take(*first)

    while remaining:
        hidden = frozenset(remaining)
        cands = [(c, d) for c in sorted(remaining) for d in range(4)]
        routes = router.route(pos, [fills[c].walks[d][0] for c, d in cands], hidden)
        reachable = [(cost, k) for k, (cost, path) in enumerate(routes) if path is not None]
        if not reachable:
            raise ValueError("the cells left lie in another piece of the region")
        _, k = min(reachable)
        c, d = cands[k]
        path = router.shortcut(routes[k][1], hidden)
        if not _same(path[0], path[-1], router.tol):
            pieces.append(Piece(TRAVEL, path, visible=router.visible(path, hidden)))
        pos = take(c, d)
    return Plan(order, pieces)


def _entries(fills, c):
    return tuple(fills[c].walks[d][0] for d in range(4))


def _forward(router, fills, cells, start, table=None, lo=0):
    """The forward pass behind the best walks of a fixed order.

    Exact: with the order fixed, the cells still to come at each travel
    are fixed too, so a travel's cost depends only on the walk it leaves
    and the walk it enters, and a shortest path through four states per
    cell (one per walk) finds the best walks. table[k] is (cost, came_from)
    for cells[k]: the cheapest cost of reaching each of its walks, None
    where none can be reached, and the walk of cells[k - 1] it came from.

    Rows before lo are taken from table: an order changed only from
    position lo on has the same cells there and the same cells still to
    come after each, so those rows are the same.
    """
    out = list(table[:lo]) if table else []
    for k in range(len(out), len(cells)):
        out.append(_row(router, fills, cells, start, k, out[k - 1][0] if k else None))
    return out


def _row(router, fills, cells, start, k, prev, cut=None):
    """Row k of the forward table, from row k - 1's costs (prev).

    cut, if given, is (best, tol, rest): rest a floor on everything after
    leaving cells[k - 1], in both terms. A walk of cells[k - 1] whose cost
    plus rest is no better than best is not travelled from, since nothing
    through it can beat best; the row is then exact for every order that
    does, which is all _spliced_cost needs."""
    if k == 0:
        if start is None:
            return [(0.0, 0.0)] * 4, None
        return router.costs(start, _entries(fills, cells[0]), frozenset(cells)), None
    hidden = frozenset(cells[k:])
    entries = _entries(fills, cells[k])
    cost, came_from = [None] * 4, [0] * 4
    for dp in range(4):
        if prev[dp] is None:
            continue
        if cut is not None:
            best, tol, rest = cut
            if not _better((prev[dp][0] + rest[0], prev[dp][1] + rest[1]), best, tol):
                continue
        step = router.costs(fills[cells[k - 1]].walks[dp][-1], entries, hidden)
        for d in range(4):
            if step[d] is None:
                continue
            total = (prev[dp][0] + step[d][0], prev[dp][1] + step[d][1])
            if cost[d] is None or total < cost[d]:
                cost[d], came_from[d] = total, dp
    return cost, came_from


def _backward(router, fills, cells):
    """back[k][d]: the cheapest cost from the end of cells[k] walked d to
    the end of the order, each later cell in its best walk; None where the
    rest cannot be reached."""
    n = len(cells)
    back = [None] * n
    back[n - 1] = [(0.0, 0.0)] * 4
    for k in range(n - 2, -1, -1):
        hidden = frozenset(cells[k + 1:])
        entries = _entries(fills, cells[k + 1])
        row = []
        for d in range(4):
            step = router.costs(fills[cells[k]].walks[d][-1], entries, hidden)
            best = None
            for dn in range(4):
                if step[dn] is None or back[k + 1][dn] is None:
                    continue
                total = (step[dn][0] + back[k + 1][dn][0], step[dn][1] + back[k + 1][dn][1])
                if best is None or total < best:
                    best = total
            row.append(best)
        back[k] = row
    return back


def _lowest(costs):
    found = [c for c in costs if c is not None]
    return min(found) if found else None


def _floor(costs):
    """(least visible, least length) over costs, each taken on its own: a
    bound under every one of them in both terms, which the lexicographic
    least is not. None if there are none."""
    found = [c for c in costs if c is not None]
    if not found:
        return None
    return (min(c[0] for c in found), min(c[1] for c in found))


def _gap(router, fills, a, b):
    """No travel from an exit of cell a to an entry of cell b is shorter
    than this, whatever is stitched: the shortest such travel when every
    cell is still to come, so that none shows and only length counts.
    Every travel runs on the same graph, so its length is at least that.
    Where the graph does not join them, the straight distance."""
    key = (a, b)
    found = router._gaps.get(key)
    if found is None:
        everything = frozenset(range(len(fills)))
        entries = _entries(fills, b)
        found = math.inf
        for d in range(4):
            exit_ = fills[a].walks[d][-1]
            for q, cost in zip(entries, router.costs(exit_, entries, everything)):
                found = min(found, cost[1] if cost is not None else math.dist(exit_, q))
        router._gaps[key] = found
    return found


def _spliced_cost(router, fills, cells, cand, start, table, back, lo, best, tol):
    """The cost of cand, an order equal to cells (whose forward table and
    back are given) before lo and after its last changed position hi.
    After hi the cells, and the cells still to come at each, are the
    current order's, so the cost from there on is back[hi + 1]: only rows
    lo to hi + 1 are computed.

    Gives up, returning None, once cand provably cannot beat best: the
    rows computed so far, plus _gap for each travel still to cost, plus
    the cheapest finish, is already no better. No travel is shorter than
    its _gap and no part of it shows less than nothing, so this bound is
    under the true cost in both terms. The bound is tried before any row
    is computed, which is what saves the most."""
    n = len(cand)
    hi = n - 1
    while hi >= lo and cand[hi] == cells[hi]:
        hi -= 1
    if hi < lo:
        return None  # the same order
    last = min(hi + 1, n - 1)
    finish = _floor(back[hi + 1]) if hi + 1 < n else (0.0, 0.0)
    if finish is None:
        return None
    # run: the length floor of every travel into rows lo..last;
    # floors[k]: of those into rows k+1..last.
    run = 0.0
    floors = {}
    for k in range(last, lo - 1, -1):
        floors[k] = run
        if k >= 1:
            run += _gap(router, fills, cand[k - 1], cand[k])
    # Before any row: the row before lo, plus every travel from lo on.
    if best is not None:
        if lo == 0:
            head = (0.0, 0.0)
            if start is not None:
                head = (0.0, min(math.dist(start, q) for q in _entries(fills, cand[0])))
        else:
            head = _floor(table[lo - 1][0])
        if head is not None and not _better(
                (head[0] + finish[0], head[1] + run + finish[1]), best, tol):
            return None
    prev = table[lo - 1][0] if lo else None
    for k in range(lo, last + 1):
        cut = None
        if best is not None and k >= 1:
            cut = (best, tol, (finish[0], _gap(router, fills, cand[k - 1], cand[k])
                               + floors[k] + finish[1]))
        row, _ = _row(router, fills, cand, start, k, prev, cut)
        low = _floor(row)
        if low is None:
            return None
        if best is not None and k < last and not _better(
                (low[0] + finish[0], low[1] + floors[k] + finish[1]), best, tol):
            return None
        prev = row
    if hi + 1 >= n:
        return _lowest(prev)
    ends = [(prev[d][0] + back[hi + 1][d][0], prev[d][1] + back[hi + 1][d][1])
            for d in range(4) if prev[d] is not None and back[hi + 1][d] is not None]
    return min(ends) if ends else None


def _best_walks(table):
    """(cost, walks) read back from a forward table, or (None, None)."""
    ends = [(c, d) for d, c in enumerate(table[-1][0]) if c is not None]
    if not ends:
        return None, None
    best, d = min(ends)
    walks = [d]
    for k in range(len(table) - 1, 0, -1):
        d = table[k][1][d]
        walks.append(d)
    return best, walks[::-1]


def _walks_for(router, fills, cells, start):
    """The best walk for every cell of a fixed order, and the total cost:
    (cost, walks), or (None, None) if some travel cannot be made."""
    return _best_walks(_forward(router, fills, cells, start))


def _better(a, b, tol):
    return a[0] < b[0] - tol or (a[0] <= b[0] + tol and a[1] < b[1] - tol)


def _rearrangements(cells):
    """Every order one move away, as (first changed position, order):
    a run of one to three cells taken out and put back elsewhere, as it
    was or reversed; two cells swapped; a stretch of the order reversed.
    Moving one cell at a time cannot take a run of neighbouring cells
    somewhere else together: each single step out of the run costs more,
    so the search never starts it."""
    n = len(cells)
    for size in (1, 2, 3):
        for i in range(n - size + 1):
            block = cells[i:i + size]
            rest = cells[:i] + cells[i + size:]
            for run in ((block, block[::-1]) if size > 1 else (block,)):
                for j in range(len(rest) + 1):
                    if j == i and run is block:
                        continue
                    yield min(i, j), rest[:j] + run + rest[j:]
    for i in range(n):
        for j in range(i + 2, n):
            out = list(cells)
            out[i], out[j] = out[j], out[i]
            yield i, out
    for i in range(n):
        for j in range(i + 4, n + 1):
            yield i, cells[:i] + cells[i:j][::-1] + cells[j:]


def _descend(router, fills, cells, start, tol):
    """Local search over the order, each order costed with its best walks.

    Takes every rearrangement that improves, carrying on from there through
    the next ones rather than starting over, and stops after a whole round
    in which none does. Starting over after each change made a region of
    thirty cells take minutes. Each candidate is costed by _spliced_cost:
    only the stretch it changes, and not even that once it cannot win.
    Returns (cells, cost, walks).
    """
    table = _forward(router, fills, cells, start)
    best, walks = _best_walks(table)
    back = _backward(router, fills, cells)
    moves = list(_rearrangements(cells))
    k, quiet = 0, 0
    while moves and quiet < len(moves):
        lo, cand = moves[k % len(moves)]
        cost = _spliced_cost(router, fills, cells, cand, start, table, back, lo, best, tol)
        if cost is not None and (best is None or _better(cost, best, tol)):
            cells = cand
            table = _forward(router, fills, cells, start, table, lo)
            best, walks = _best_walks(table)
            back = _backward(router, fills, cells)
            moves = list(_rearrangements(cells))
            quiet = 0
        else:
            quiet += 1
        k += 1
    return cells, best, walks


def _kick(cells, rng):
    """Cut the order in four and swap the middle two: A B C D -> A C B D.
    No single rearrangement undoes it, so the search restarts somewhere
    it could not walk to."""
    p, q, r = sorted(rng.sample(range(1, len(cells)), 3))
    return cells[:p] + cells[q:r] + cells[p:q] + cells[r:]


def _improve(router, fills, cells, start, tol, kicks):
    """Local search, then `kicks` times: shake the best order found and
    search again from there, keeping the result if it is better. Seeded,
    so a plan is the same every run."""
    cells, best, walks = _descend(router, fills, cells, start, tol)
    if len(cells) < 4:
        return cells, best, walks
    rng = random.Random(0)
    for _ in range(kicks):
        cand, cost, cand_walks = _descend(router, fills, _kick(cells, rng), start, tol)
        if cost is not None and (best is None or _better(cost, best, tol)):
            cells, best, walks = cand, cost, cand_walks
    return cells, best, walks


def _assemble(router, fills, cells, walks, start):
    order, pieces = [], []
    pos = start
    for i, (c, d) in enumerate(zip(cells, walks)):
        walk = fills[c].walks[d]
        if pos is not None:
            hidden = frozenset(cells[i:])
            _, path = router.route(pos, [walk[0]], hidden)[0]
            path = router.shortcut(path, hidden)
            if not _same(path[0], path[-1], router.tol):
                pieces.append(Piece(TRAVEL, path, visible=router.visible(path, hidden)))
        order.append((c, d))
        pieces.append(Piece(FILL, list(walk), cell=c, direction=d))
        pos = walk[-1]
    return Plan(order, pieces)
