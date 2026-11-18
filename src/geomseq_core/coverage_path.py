"""Coverage path planning: one continuous stitch path over a region.

Four stages, each usable on its own: decompose (cut the region into
monotone cells), fill_cells (a zig-zag per cell, walkable four ways),
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


class Fill(object):
    """The zig-zag that covers one cell, in the input frame.

    A cell can be stitched four ways, `walks[direction]`:
      FORWARD           bottom to top in the sweep, from the left end of
                        the lowest line
      REVERSE           FORWARD walked the other way
      MIRRORED          bottom to top, from the right end of the lowest line
      MIRRORED_REVERSE  MIRRORED walked the other way
    All four lay the same lines; they differ in which corner they enter at
    and which they leave from, which is what the order of cells chooses
    between. A walk's entry is its first point and its exit its last.
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


def fill_cells(dec, spacing, phase=0.0, tol=DEFAULT_TOL):
    """Lay fill lines across every cell of a Decomposition.

    spacing: distance between lines, in input units. Must be > 0.
    phase: offset of the line grid along the sweep. Lines sit at
        phase + k * spacing for integer k, in the sweep frame, for every
        cell alike, so lines of neighbouring cells are collinear across a
        portal and the pattern reads as one. A cell too thin to catch a
        grid line gets one line through its middle.
    A grid line exactly on a portal belongs to the cell above it; the cell
    below stops short of it, or that row would be stitched twice.
    Returns one Fill per cell, in cell order.
    """
    if spacing <= 0.0:
        raise ValueError("spacing must be greater than 0")
    cb, sb = math.cos(dec.angle), math.sin(dec.angle)
    has_above = set(p.cell_a for p in dec.portals)
    fills = []
    for i, cell in enumerate(dec.cells):
        drop_top = i in has_above
        pts, lines = _zigzag(cell.rows, spacing, phase, tol, True, drop_top)
        mirrored, _ = _zigzag(cell.rows, spacing, phase, tol, False, drop_top)
        fills.append(Fill(i,
                          [_rotate(p, cb, sb) for p in pts],
                          [_rotate(p, cb, sb) for p in mirrored],
                          [(_rotate((xl, y), cb, sb), _rotate((xr, y), cb, sb))
                           for y, xl, xr in lines]))
    return fills


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
            for row in rows:
                if y_prev < row[0] < y:
                    pts.append((row[side], row[0]))
        if left_to_right:
            pts.append((xl, y))
            pts.append((xr, y))
        else:
            pts.append((xr, y))
            pts.append((xl, y))
    return pts, lines


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
    dx, dy = b[0] - a[0], b[1] - a[1]
    L = math.hypot(dx, dy)
    ts = [0.0, 1.0]
    for ring in rings:
        n = len(ring)
        for i in range(n):
            p, q = ring[i], ring[(i + 1) % n]
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
    """Dijkstra from src over links (node, length, pieces), cost
    (visible length, length) for the given hidden set. Stops once every
    node in targets is settled, if any are given. Returns (dist, prev)."""
    n = len(adj)
    dist = [None] * n
    prev = [-1] * n
    dist[src] = (0.0, 0.0)
    heap = [(0.0, 0.0, src)]
    left = set(targets)
    while heap:
        vis, length_so_far, u = heapq.heappop(heap)
        d = (vis, length_so_far)
        if d > dist[u]:
            continue
        if left:
            left.discard(u)
            if not left:
                break
        for v, length, pieces in adj[u]:
            nd = (vis + _shown(pieces, hidden), length_so_far + length)
            if dist[v] is None or nd < dist[v]:
                dist[v] = nd
                prev[v] = u
                heapq.heappush(heap, (nd[0], nd[1], v))
    return dist, prev


def _walk_back(nodes, prev, dst):
    path = []
    u = dst
    while u != -1:
        path.append(nodes[u])
        u = prev[u]
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
        self._corners = None
        # No travel passes through a corner of the outline: there a thread
        # along the edge shows. A corner may still end a segment (the
        # fallback's nodes are corners, and a fill may end on one).
        self.outline_corners = [v for r in dec.rings for v in r]

        # A fill line may sit on the region boundary itself (the grid landed
        # on a cell's top or bottom row, and that row is a hole's edge); its
        # midpoint is then on the boundary and is not a place a travel may be.
        ring_edges = [(r[i], r[(i + 1) % len(r)]) for r in dec.rings for i in range(len(r))]

        def on_boundary(p):
            return any(_on_segment(p, a, b, tol) for a, b in ring_edges)

        # Midline of each cell: its line midpoints, chained.
        first, last = {}, {}
        for c in range(len(dec.cells)):
            prev_node = None
            for k, (a, b) in enumerate(fills[c].lines):
                m = _mid(a, b)
                if on_boundary(m):
                    continue
                node = self._add_node(m)
                self.mids.append((node, c, k))
                if prev_node is not None:
                    self._link(prev_node, node)
                else:
                    first[c] = node
                prev_node = node
            if prev_node is not None:
                last[c] = prev_node

        # Portals join the lower cell's last line to the upper cell's first,
        # through the portal's midpoint. Each cell also keeps its ways out,
        # up and down (the portal's midpoint, and the neighbour's line next
        # to it), so an end on its top or bottom line can leave across the
        # gap to the portal instead of back across the cell.
        self.ways_up, self.ways_down = {}, {}
        for p in dec.portals:
            if p.cell_a in last and p.cell_b in first:
                node = self._add_node(_mid(p.p0, p.p1))
                self._link(last[p.cell_a], node)
                self._link(node, first[p.cell_b])
                self.ways_up.setdefault(p.cell_a, []).extend([node, first[p.cell_b]])
                self.ways_down.setdefault(p.cell_b, []).extend([node, last[p.cell_a]])

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
            for ends, line, target in ((fills[below].lines[-1], fills[above].lines[0], first[above]),
                                       (fills[above].lines[0], fills[below].lines[-1], last[below])):
                for e in ends:
                    for q in _step_targets(e, line, tol):
                        step = self.pieces(e, q)
                        if step is None:
                            continue
                        both = frozenset([below, above]).union(*[cells for _, cells in step])
                        key = (e, q) if e <= q else (q, e)
                        self._pieces[key] = [(sum(length for length, _ in step), both)]
                        node = self._add_node(q)
                        self._link(node, target)
                        self.hops.setdefault(e, []).append(node)
                        break

    def _add_node(self, p):
        self.nodes.append(p)
        self.adj.append([])
        return len(self.nodes) - 1

    def _link(self, i, j):
        a, b = self.nodes[i], self.nodes[j]
        pieces = self.pieces(a, b)
        if pieces is None:
            return
        length = math.dist(a, b)
        self.adj[i].append((j, length, pieces))
        self.adj[j].append((i, length, pieces))

    def pieces(self, a, b):
        """Segment ab as (length, cells) pieces, or None if it leaves the
        region. A piece on a shared edge lies in both cells. Remembered per
        segment, either way round."""
        key = (a, b) if a <= b else (b, a)
        if key not in self._pieces:
            self._pieces[key] = self._cut_into_pieces(*key)
        return self._pieces[key]

    def _cut_into_pieces(self, a, b):
        tol, region = self.tol, self.region
        if not (_inside_union(a, region, tol) and _inside_union(b, region, tol)):
            return None
        for v in self.outline_corners:
            if not _same(v, a, tol) and not _same(v, b, tol) and _on_segment(v, a, b, tol):
                return None
        dx, dy = b[0] - a[0], b[1] - a[1]
        L = math.hypot(dx, dy)
        if L <= tol:
            return []
        ts = _cuts(a, b, region, tol)
        out = []
        for t0, t1 in zip(ts, ts[1:]):
            piece = (t1 - t0) * L
            if piece <= tol:
                continue
            tm = 0.5 * (t0 + t1)
            m = (a[0] + tm * dx, a[1] + tm * dy)
            cells = frozenset(i for i, r in enumerate(region) if _in_closed_polygon(m, r, tol))
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
            if _segment_inside_union(p, self.nodes[node], self.region, self.tol):
                pieces = self.pieces(p, self.nodes[node])
                return [(node, math.dist(p, self.nodes[node]), pieces)]
        return out

    def route(self, start, goals, hidden):
        """The cheapest travel from start to each goal, as (cost, path),
        cost (visible length, length); path None only when the goal is in
        another piece of the region. No direct start-to-goal link: between
        two fill ends it could run along the boundary, which is the one
        place a travel must not be."""
        n0 = len(self.nodes)
        nodes = self.nodes + [start] + list(goals)
        adj = [list(a) for a in self.adj] + [[] for _ in range(1 + len(goals))]
        for node, length, pieces in self._attach(start):
            adj[n0].append((node, length, pieces))
            adj[node].append((n0, length, pieces))
        for g, goal in enumerate(goals):
            for node, length, pieces in self._attach(goal):
                adj[node].append((n0 + 1 + g, length, pieces))  # into the goal only
        dist, prev = _search(adj, n0, hidden, range(n0 + 1, n0 + 1 + len(goals)))

        out = []
        for g, goal in enumerate(goals):
            if _same(start, goal, self.tol):
                out.append(((0.0, 0.0), [start, goal]))
            elif dist[n0 + 1 + g] is not None:
                out.append((dist[n0 + 1 + g], _walk_back(nodes, prev, n0 + 1 + g)))
            else:
                out.append(self._corner_route(start, goal, hidden))
        return out

    def costs(self, start, goals, hidden):
        """route's costs alone (None for an unreachable goal), remembered:
        an order search asks for the same exit, entries and stitched set
        many times over."""
        key = (start, goals, hidden)
        if key not in self._costs:
            self._costs[key] = [cost for cost, _ in self.route(start, list(goals), hidden)]
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
                        adj[i].append((j, length, pieces))
                        adj[j].append((i, length, pieces))
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
                    adj[end].append((i, length, pieces))
                    adj[i].append((end, length, pieces))
        pieces = self.pieces(start, goal)
        if pieces is not None:
            adj[s].append((g, math.dist(start, goal), pieces))
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
            # A portal's midpoint can coincide with a line's on it.
            if not _same(path[nxt], out[-1], self.tol) or nxt == len(path) - 1:
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


def plan(dec, fills, start=None, tol=DEFAULT_TOL):
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
    four states per cell. A local search moves and swaps cells in the
    order, keeping a change whenever that exact cost improves, until none
    does. Costs are (visible length, travel length), compared in order.

    start: where the needle starts, or None to let the plan choose (every
    first (cell, walk) is tried for the seed). Cells too thin to hold a
    fill line are left out. Returns a Plan.
    """
    if tol < 0.0:
        raise ValueError("tol cannot be negative")
    if len(fills) != len(dec.cells):
        raise ValueError("fills must be one per cell of dec")
    router = _Router(dec, fills, tol)
    todo = [c for c in range(len(dec.cells)) if fills[c].lines]
    if not todo:
        return Plan([], [])
    if start is not None:
        seed = _greedy(router, fills, todo, start, None)
    else:
        seed = None
        for c in todo:
            for d in range(4):
                p = _greedy(router, fills, todo, None, (c, d))
                if seed is None or p.cost < seed.cost:
                    seed = p
    cells = [c for c, _ in seed.order]
    cells, _, walks = _improve(router, fills, cells, start, tol)
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


def _walks_for(router, fills, cells, start):
    """The best walk for every cell of a fixed order, and the total cost.

    Exact: with the order fixed, the cells still to come at each travel
    are fixed too, so a travel's cost depends only on the walk it leaves
    and the walk it enters. A shortest path through four states per cell
    (one per walk) finds the best walks. Returns (cost, walks), or
    (None, None) if some travel cannot be made.
    """
    if start is None:
        cost = [(0.0, 0.0)] * 4
    else:
        cost = router.costs(start, _entries(fills, cells[0]), frozenset(cells))
    back = []
    for i in range(1, len(cells)):
        hidden = frozenset(cells[i:])
        entries = _entries(fills, cells[i])
        new, came_from = [None] * 4, [0] * 4
        for dp in range(4):
            if cost[dp] is None:
                continue
            step = router.costs(fills[cells[i - 1]].walks[dp][-1], entries, hidden)
            for d in range(4):
                if step[d] is None:
                    continue
                total = (cost[dp][0] + step[d][0], cost[dp][1] + step[d][1])
                if new[d] is None or total < new[d]:
                    new[d], came_from[d] = total, dp
        cost = new
        back.append(came_from)
    ends = [(c, d) for d, c in enumerate(cost) if c is not None]
    if not ends:
        return None, None
    best, d = min(ends)
    walks = [d]
    for came_from in reversed(back):
        d = came_from[d]
        walks.append(d)
    return best, walks[::-1]


def _better(a, b, tol):
    return a[0] < b[0] - tol or (a[0] <= b[0] + tol and a[1] < b[1] - tol)


def _rearrangements(cells):
    """Every order one move away: a cell taken out and put back elsewhere,
    or two cells swapped."""
    n = len(cells)
    for i in range(n):
        rest = cells[:i] + cells[i + 1:]
        for j in range(n):
            if j != i:
                yield rest[:j] + [cells[i]] + rest[j:]
    for i in range(n):
        for j in range(i + 2, n):
            out = list(cells)
            out[i], out[j] = out[j], out[i]
            yield out


def _improve(router, fills, cells, start, tol):
    """Local search over the order, each order costed with its best walks.
    Takes the first rearrangement that improves and starts over, until
    none does."""
    best, walks = _walks_for(router, fills, cells, start)
    improved = True
    while improved:
        improved = False
        for cand in _rearrangements(cells):
            cost, cand_walks = _walks_for(router, fills, cand, start)
            if cost is not None and _better(cost, best, tol):
                cells, best, walks = cand, cost, cand_walks
                improved = True
                break
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
