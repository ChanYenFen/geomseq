"""Property tests for build_turn_waypoints_native; cases are inline, plain floats.
The turn cap holds inside each fillet but not across the exit->entry junction, so
property 3 is per-case here, not a universal invariant (see README Notes)."""

import math
import os
import sys

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(os.path.dirname(_HERE), "src"))

from geomseq_core.geometry_utils import build_turn_waypoints_native

TOL = 1e-9

# `straight`: the turn is ~0, so the chain must not detour off the direct line.
CASES = [
    dict(name="known_hand_verified",
         E_prev=(126.124342, -138.414543), E=(126.124342, -140.399417),
         S=(108.303747, -209.630030), S_next=(106.755897, -204.875660),
         theta_max_deg=30.0, step_len=1.0, extend_len=2.0, straight=False),
    dict(name="parallel_same_dir",
         E_prev=(0.0, 0.0), E=(10.0, 0.0), S=(30.0, 0.0), S_next=(40.0, 0.0),
         theta_max_deg=30.0, step_len=1.0, extend_len=2.0, straight=True),
    dict(name="parallel_hairpin",
         E_prev=(0.0, 0.0), E=(10.0, 0.0), S=(10.0, 5.0), S_next=(0.0, 5.0),
         theta_max_deg=30.0, step_len=1.0, extend_len=2.0, straight=False),
    dict(name="right_angle",
         E_prev=(0.0, 0.0), E=(10.0, 0.0), S=(20.0, 10.0), S_next=(20.0, 20.0),
         theta_max_deg=30.0, step_len=1.0, extend_len=2.0, straight=False),
]


def unit(v):
    n = math.hypot(*v)
    return (v[0] / n, v[1] / n)


def turn_angles(chain):
    """Angle in degrees between consecutive segments, one per interior vertex."""
    angles = []
    for p, q, r in zip(chain, chain[1:], chain[2:]):
        u, v = (q[0] - p[0], q[1] - p[1]), (r[0] - q[0], r[1] - q[1])
        nu, nv = math.hypot(*u), math.hypot(*v)
        if nu <= TOL or nv <= TOL:  # coincident points have no direction
            continue
        dot = max(-1.0, min(1.0, (u[0] * v[0] + u[1] * v[1]) / (nu * nv)))
        angles.append(math.degrees(math.acos(dot)))
    return angles


def check_case(name, E_prev, E, S, S_next, theta_max_deg, step_len, extend_len, straight):
    a_vec = (E[0] - E_prev[0], E[1] - E_prev[1])
    b_vec = (S_next[0] - S[0], S_next[1] - S[1])
    a_h, b_h = unit(a_vec), unit(b_vec)
    e_ext = (E[0] + a_h[0] * extend_len, E[1] + a_h[1] * extend_len)
    s_ext = (S[0] - b_h[0] * extend_len, S[1] - b_h[1] * extend_len)

    exit_pts, entry_pts = build_turn_waypoints_native(
        E[0], E[1], a_vec[0], a_vec[1], S[0], S[1], b_vec[0], b_vec[1],
        theta_max_deg, step_len, extend_len)

    assert exit_pts and entry_pts, f"{name}: empty output"

    # 1/2. Both ends pinned to the extend points, so the real path never moves.
    assert all(abs(g - w) <= TOL for g, w in zip(exit_pts[0], e_ext)), (
        f"{name}: exit_pts[0] {exit_pts[0]} != E_extend {e_ext}")
    assert all(abs(g - w) <= TOL for g, w in zip(entry_pts[-1], s_ext)), (
        f"{name}: entry_pts[-1] {entry_pts[-1]} != S_extend {s_ext}")

    # 3. No vertex turns by more than theta_max_deg (see module docstring).
    chain = exit_pts + entry_pts
    angles = turn_angles(chain)
    worst = max(angles) if angles else 0.0
    assert worst <= theta_max_deg + 1e-6, (
        f"{name}: vertex turns {worst:.3f} deg, over theta_max_deg {theta_max_deg}")

    # 4. A ~0 turn should produce no turning and no detour.
    if straight:
        span = math.dist(e_ext, s_ext)
        length = sum(math.dist(p, q) for p, q in zip(chain, chain[1:]))
        assert worst <= TOL, f"{name}: expected no turn, got {worst}"
        assert length <= span + TOL, f"{name}: detours ({length} vs straight {span})"

    print(f"{name:<24} exit={len(exit_pts):3d} entry={len(entry_pts):3d}"
          f"   max_turn={worst:7.3f}")


def main():
    for case in CASES:
        check_case(**case)
    print("All tests passed")


if __name__ == "__main__":
    main()
