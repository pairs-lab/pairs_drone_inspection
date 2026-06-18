"""Unit tests for drone/avoidance.py — APF repulsive force and clearance.

Pure-Python / NumPy, no Isaac Sim required.
Run with: conda run -n isaac6 python -m pytest tests/test_avoidance.py -v
"""
import numpy as np
import pytest
from drone.avoidance import repulsive_force, clearance


# Simple unit AABB centred at origin, half-size 1 m in each axis
_UNIT_BOX = [((-1.0, -1.0, -1.0), (1.0, 1.0, 1.0))]


def test_outside_influence_zero_force():
    """Position far away -> zero repulsive force."""
    pos = np.array([5.0, 0.0, 0.0])
    f = repulsive_force(pos, _UNIT_BOX, influence=0.6)
    assert np.allclose(f, 0.0), f"Expected zero force, got {f}"


def test_closer_obstacle_larger_force():
    """Force magnitude grows as drone approaches the obstacle."""
    influence = 1.0
    f_far = repulsive_force(np.array([2.5, 0.0, 0.0]), _UNIT_BOX, influence=influence)
    f_mid = repulsive_force(np.array([2.0, 0.0, 0.0]), _UNIT_BOX, influence=influence)
    f_close = repulsive_force(np.array([1.5, 0.0, 0.0]), _UNIT_BOX, influence=influence)

    mag_far = np.linalg.norm(f_far)
    mag_mid = np.linalg.norm(f_mid)
    mag_close = np.linalg.norm(f_close)

    # Only positions inside the influence sphere should produce force
    # (far is 1.5 m from surface > influence=1.0, so it should be ~0)
    assert mag_far < 1e-6, f"Expected ~0 force far away, got {mag_far}"
    assert mag_close > mag_mid, (
        f"Close force {mag_close:.3f} should exceed mid force {mag_mid:.3f}"
    )


def test_force_points_away():
    """Force direction should point away from the closest obstacle surface."""
    # Approach from +X side of the box
    pos = np.array([1.3, 0.0, 0.0])
    f = repulsive_force(pos, _UNIT_BOX, influence=1.0)
    # Force should have a strong +X component (pushing away from +X face)
    assert f[0] > 0.0, f"Force should be in +X direction, got {f}"
    # Lateral components should be much smaller than axial
    assert abs(f[1]) < abs(f[0]), "Y component should not dominate"


def test_force_clamp():
    """Force must not exceed the internal clamp (50 N) even at zero distance."""
    pos = np.array([1.0, 0.0, 0.0])  # exactly on the surface
    f = repulsive_force(pos, _UNIT_BOX, influence=1.0, gain=100.0)
    assert np.linalg.norm(f) <= 50.0 + 1e-6


def test_clearance_outside():
    """Point outside box -> positive clearance."""
    pos = np.array([2.0, 0.0, 0.0])
    c = clearance(pos, _UNIT_BOX)
    assert c > 0.0, f"Expected positive clearance, got {c}"
    assert abs(c - 1.0) < 1e-6, f"Expected clearance 1.0, got {c}"


def test_clearance_inside():
    """Point inside box -> negative clearance (penetration)."""
    pos = np.array([0.0, 0.0, 0.0])
    c = clearance(pos, _UNIT_BOX)
    assert c < 0.0, f"Expected negative clearance inside box, got {c}"


def test_clearance_on_surface():
    """Point on surface -> zero clearance."""
    pos = np.array([1.0, 0.0, 0.0])
    c = clearance(pos, _UNIT_BOX)
    assert abs(c) < 1e-6, f"Expected ~0 clearance on surface, got {c}"


def test_clearance_no_aabbs():
    """Empty AABB list -> infinite clearance."""
    pos = np.array([0.0, 0.0, 0.0])
    c = clearance(pos, [])
    assert c == float("inf")


def test_multiple_obstacles_closest():
    """Clearance returns minimum over all obstacles."""
    aabbs = [
        ((-10.0, -1.0, -1.0), (-8.0, 1.0, 1.0)),  # 8 m away in -X
        ((3.0,  -1.0, -1.0), (5.0,  1.0, 1.0)),    # 3 m away in +X
    ]
    pos = np.array([0.0, 0.0, 0.0])
    c = clearance(pos, aabbs)
    # Nearest is the +X box (3 m away in X, closest surface at x=3)
    assert abs(c - 3.0) < 1e-6, f"Expected clearance 3.0, got {c}"
