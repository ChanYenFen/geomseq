"""The pruned point_in_curve against a scan of every edge.

The native side skips edges twice over -- the bounding box, then a sweep in y
that only hands each query the edges near its height. Both are meant to be
exact, not approximate: the answer must be the one a full scan gives. The
reference here is that full scan, written plainly in Python with the same
per-edge arithmetic, and the inputs are built to crowd the places where a
wrongly skipped edge would change an answer -- points just inside the band,
points level with a vertex, long edges that span the whole polygon, and a
sawtooth whose every tooth shares one band.

Development check, not committed as a test: with the sweep's exit height
narrowed instead of widened (`leave_y = ymax - span`), so that edges leave the
active set while still within the band of points below them, 20 of the 21
cases below failed; restored, all pass. That is what makes them a guard.
"""

import math
import os
import random
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import (
    PIC_COINCIDENT,
    PIC_INSIDE,
    PIC_OUTSIDE,
    point_in_curve_native,
    point_in_curves_native,
)

EPS = 1e-9  # the native floor on tol


# --- the reference: every edge, every time ------------------------------------

def _dist2_to_edge(a, b, q):
    dx, dy = b[0] - a[0], b[1] - a[1]
    len2 = dx * dx + dy * dy
    t = 0.0
    if len2 > 0.0:
        t = ((q[0] - a[0]) * dx + (q[1] - a[1]) * dy) / len2
        t = min(1.0, max(0.0, t))
    ex, ey = q[0] - (a[0] + t * dx), q[1] - (a[1] + t * dy)
    return ex * ex + ey * ey


def _ray_crosses(a, b, q):
    if (a[1] > q[1]) == (b[1] > q[1]):
        return False
    t = (q[1] - a[1]) / (b[1] - a[1])
    return a[0] + t * (b[0] - a[0]) > q[0]


def reference_classify(polyline, q, tol):
    span = tol if tol > EPS else EPS
    edges = list(zip(polyline[:-1], polyline[1:]))
    if any(_dist2_to_edge(a, b, q) <= span * span for a, b in edges):
        return PIC_COINCIDENT
    crossings = sum(1 for a, b in edges if _ray_crosses(a, b, q))
    return PIC_INSIDE if crossings & 1 else PIC_OUTSIDE


# --- inputs built to crowd the edge cases -------------------------------------

def _circle(n, r=10.0, cx=0.0, cy=0.0):
    pts = [(cx + r * math.cos(2 * math.pi * k / n), cy + r * math.sin(2 * math.pi * k / n))
           for k in range(n)]
    return pts + [pts[0]]


def _comb(teeth, width=20.0, height=10.0):
    """Teeth that all span one band, so every tooth edge is active at once."""
    pts = [(width * k / (teeth - 1), height if k % 2 else 0.0) for k in range(teeth)]
    pts += [(width, -height), (0.0, -height)]
    return pts + [pts[0]]


def _star(n, r_in=4.0, r_out=10.0):
    pts = []
    for k in range(2 * n):
        r = r_out if k % 2 == 0 else r_in
        a = math.pi * k / n
        pts.append((r * math.cos(a), r * math.sin(a)))
    return pts + [pts[0]]


def _crowded_queries(rnd, polyline, count, tol):
    """Random points over the box, plus points placed where a wrongly skipped
    edge would show: on the band's edge, level with vertices, on edges."""
    xs = [p[0] for p in polyline]
    ys = [p[1] for p in polyline]
    lo_x, hi_x, lo_y, hi_y = min(xs) - 1, max(xs) + 1, min(ys) - 1, max(ys) + 1
    out = [(rnd.uniform(lo_x, hi_x), rnd.uniform(lo_y, hi_y)) for _ in range(count)]

    for (x0, y0), (x1, y1) in zip(polyline[:-1], polyline[1:]):
        t = rnd.random()
        mx, my = x0 + t * (x1 - x0), y0 + t * (y1 - y0)
        out.append((mx, my))                       # on the edge
        out.append((mx + 0.999 * tol, my))         # just inside the band, across
        out.append((mx + 1.001 * tol, my))         # just outside it
        out.append((mx, my + 0.999 * tol))         # just inside the band, above:
        out.append((mx, my - 0.999 * tol))         #   what the sweep's exit and
        out.append((mx, my + 1.001 * tol))         #   entry heights decide
        out.append((mx, my - 1.001 * tol))
        out.append((rnd.uniform(lo_x, hi_x), y0))  # level with a vertex
        out.append((rnd.uniform(lo_x, hi_x), y0 + tol))
        out.append((rnd.uniform(lo_x, hi_x), y0 - tol))
    return out


SHAPES = {
    "circle":   lambda: _circle(64),
    "comb":     lambda: _comb(41),
    "star":     lambda: _star(12),
    "square":   lambda: [(0.0, 0.0), (10.0, 0.0), (10.0, 10.0), (0.0, 10.0), (0.0, 0.0)],
    "tall":     lambda: [(0.0, 0.0), (1.0, 0.0), (1.0, 100.0), (0.0, 100.0), (0.0, 0.0)],
}


@pytest.mark.parametrize("shape", sorted(SHAPES))
@pytest.mark.parametrize("tol", [0.0, 1e-6, 0.05, 0.5])
def test_pruned_scan_matches_every_edge(shape, tol):
    rnd = random.Random(11)
    polyline = SHAPES[shape]()
    query = _crowded_queries(rnd, polyline, 400, tol)

    got, _ = point_in_curve_native([polyline], query, tol=tol)
    want = [reference_classify(polyline, q, tol) for q in query]

    mismatches = [(q, g, w) for q, g, w in zip(query, got, want) if g != w]
    assert mismatches == [], mismatches[:5]


def test_the_plural_matches_the_singular_curve_by_curve():
    """point_in_curves runs the same sweep per curve and stops at the first
    hit; per point it must agree with asking each curve on its own."""
    rnd = random.Random(7)
    curves = [_circle(32, r=5.0, cx=0.0, cy=0.0),
              _circle(32, r=5.0, cx=6.0, cy=0.0),     # overlaps the first
              _comb(21),                              # overlaps both
              _circle(16, r=2.0, cx=40.0, cy=40.0)]   # apart
    query = [(rnd.uniform(-10, 50), rnd.uniform(-15, 50)) for _ in range(600)]
    tol = 0.1

    rel, idx, _ = point_in_curves_native(curves, query, tol=tol)
    singles = [point_in_curve_native([c], query, tol=tol)[0] for c in curves]

    for q in range(len(query)):
        first = next((c for c in range(len(curves)) if singles[c][q] != PIC_OUTSIDE), -1)
        assert idx[q] == first
        assert rel[q] == (singles[first][q] if first >= 0 else PIC_OUTSIDE)
