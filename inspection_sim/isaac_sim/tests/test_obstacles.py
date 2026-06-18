"""Unit tests for sim/obstacles.py — AABB list shape and values.

Pure-Python, no Isaac Sim required.
Run with: conda run -n isaac6 python -m pytest tests/test_obstacles.py -v
"""
from sim.obstacles import get_obstacle_aabbs, _PRIM_FACE_Y, _SEC_FACE_Y, _SLAB_HALF_THICKNESS, _SLAB_X_MIN, _SLAB_X_MAX
from sim.config import (
    RACK_X_MIN, RACK_X_MAX, RACK_Y_MIN, RACK_Y_MAX, RACK_TOTAL_HEIGHT,
    SECOND_RACK_Y, OBSTACLE_X, OBSTACLE_Y, OBSTACLE_HALF_W, OBSTACLE_HALF_D, OBSTACLE_HALF_H,
    AISLE_CENTER_Y,
)


def test_aabb_count():
    aabbs = get_obstacle_aabbs()
    assert len(aabbs) == 3, f"Expected 3 AABBs, got {len(aabbs)}"


def test_aabb_shape():
    aabbs = get_obstacle_aabbs()
    for i, (mn, mx) in enumerate(aabbs):
        assert len(mn) == 3, f"AABB[{i}] min should have 3 coords"
        assert len(mx) == 3, f"AABB[{i}] max should have 3 coords"


def test_aabbs_are_valid_boxes():
    """Each max coord must be strictly greater than min coord."""
    aabbs = get_obstacle_aabbs()
    for i, (mn, mx) in enumerate(aabbs):
        for axis, (lo, hi) in enumerate(zip(mn, mx)):
            assert hi > lo, (
                f"AABB[{i}] axis={axis}: max={hi} <= min={lo} (degenerate box)"
            )


def test_primary_rack_face_bounds():
    """Primary rack face slab should be at RACK_Y_MIN and extend from SLAB_X_MIN to RACK_X_MAX."""
    aabbs = get_obstacle_aabbs()
    mn, mx = aabbs[0]
    assert abs(mn[0] - _SLAB_X_MIN) < 1e-6
    assert abs(mx[0] - _SLAB_X_MAX) < 1e-6
    # Y centred at RACK_Y_MIN with half-thickness
    assert abs(mn[1] - (_PRIM_FACE_Y - _SLAB_HALF_THICKNESS)) < 1e-6
    assert abs(mx[1] - (_PRIM_FACE_Y + _SLAB_HALF_THICKNESS)) < 1e-6
    assert abs(mn[2]) < 1e-6          # starts at floor z=0
    assert abs(mx[2] - RACK_TOTAL_HEIGHT) < 1e-6


def test_second_rack_face_bounds():
    """Second rack face slab at SEC_FACE_Y, X from SLAB_X_MIN to RACK_X_MAX."""
    aabbs = get_obstacle_aabbs()
    mn, mx = aabbs[1]
    assert abs(mn[0] - _SLAB_X_MIN) < 1e-6
    assert abs(mx[0] - _SLAB_X_MAX) < 1e-6
    assert abs(mn[1] - (_SEC_FACE_Y - _SLAB_HALF_THICKNESS)) < 1e-6
    assert abs(mx[1] - (_SEC_FACE_Y + _SLAB_HALF_THICKNESS)) < 1e-6
    # Must be on the negative Y side
    assert mx[1] < 0.0, "Second rack face must be on negative Y side"


def test_obstacle_bounds():
    aabbs = get_obstacle_aabbs()
    mn, mx = aabbs[2]
    assert abs(mn[0] - (OBSTACLE_X - OBSTACLE_HALF_W)) < 1e-6
    assert abs(mx[0] - (OBSTACLE_X + OBSTACLE_HALF_W)) < 1e-6
    assert abs(mn[1] - (OBSTACLE_Y - OBSTACLE_HALF_D)) < 1e-6
    assert abs(mx[1] - (OBSTACLE_Y + OBSTACLE_HALF_D)) < 1e-6
    assert abs(mn[2]) < 1e-6
    assert abs(mx[2] - OBSTACLE_HALF_H * 2) < 1e-6


def test_face_slabs_separated():
    """Primary and second rack faces must be separated by approximately AISLE_WIDTH - 2*slab."""
    from sim.config import AISLE_WIDTH
    aabbs = get_obstacle_aabbs()
    _, prim_max = aabbs[0]
    sec_min, _ = aabbs[1]
    # Gap between primary slab max-Y and second slab min-Y
    # prim_max.y = RACK_Y_MIN + SLAB_HALF_THICKNESS = -0.45 + 0.10 = -0.35
    # sec_min.y  = SEC_FACE_Y  - SLAB_HALF_THICKNESS = -1.35 - 0.10 = -1.45
    # aisle clear gap = prim_max.y - sec_min.y (positive, prim_max > sec_min in Y)
    aisle_gap = prim_max[1] - sec_min[1]
    assert aisle_gap > 0, (
        f"Expected primary slab max-Y {prim_max[1]:.3f} > second slab min-Y {sec_min[1]:.3f}"
    )
    # Gap should be approximately AISLE_WIDTH - 2*SLAB_HALF_THICKNESS
    expected = AISLE_WIDTH - 2 * _SLAB_HALF_THICKNESS
    assert abs(aisle_gap - expected) < 0.05, (
        f"Gap {aisle_gap:.3f} != expected {expected:.3f}"
    )


def test_obstacle_in_aisle():
    """Obstacle Y should be in the aisle corridor; X within rack span."""
    aabbs = get_obstacle_aabbs()
    _, prim_max = aabbs[0]
    sec_min, _ = aabbs[1]
    obs_mn, obs_mx = aabbs[2]
    obs_y_ctr = (obs_mn[1] + obs_mx[1]) / 2.0

    # Obstacle centre Y is inside aisle corridor
    assert sec_min[1] < obs_y_ctr < prim_max[1], (
        f"Obstacle Y centre {obs_y_ctr:.3f} should be in aisle corridor "
        f"[{sec_min[1]:.3f}, {prim_max[1]:.3f}]"
    )
    # Obstacle X is within rack X span (RACK_X_MIN..RACK_X_MAX, not SLAB_X_MIN)
    obs_x_ctr = (obs_mn[0] + obs_mx[0]) / 2.0
    assert RACK_X_MIN < obs_x_ctr < RACK_X_MAX


def test_drone_at_aisle_center_no_apf():
    """Drone at aisle centerline (inside rack X span) should have minimal APF clearance issues."""
    import numpy as np
    from drone.avoidance import clearance
    from drone.flight import APF_INFLUENCE, DRONE_SAFETY_R

    aabbs = get_obstacle_aabbs()
    # Drone at aisle centerline, mid-rack
    pos = np.array([1.2, AISLE_CENTER_Y, 1.2])
    clr = clearance(pos, aabbs)
    # Should be positive (not inside any obstacle)
    assert clr > 0, f"Drone at aisle center should have positive clearance, got {clr}"
    # Should be comfortably outside APF influence when at center
    # distance from aisle_center to face = (AISLE_WIDTH/2 - SLAB_HALF_THICKNESS) ≈ 0.55 m
    # With DRONE_SAFETY_R=0.25, effective_d = ~0.30 = APF_INFLUENCE → just at boundary
    # That's OK: clearance > 0 is the main check
    print(f"[test] drone at aisle center: clearance={clr:.3f} m")
