"""Obstacle registry for the narrow-aisle warehouse demo.

`get_obstacle_aabbs()` returns a list of axis-aligned bounding boxes
(min_xyz, max_xyz) tuples in world coordinates for APF obstacle avoidance.

Obstacles represented (3 entries):
  0. Primary rack aisle-face slab  — the -Y face of the primary rack, modelled as a
     thin slab extending into the aisle.  Using only the aisle-facing surface (not
     the full rack volume) prevents the APF local-minimum trap at the aisle entrance:
     a full-volume AABB creates a symmetric dead-zone at x < RACK_X_MIN where corners
     of both racks create equal-and-opposite Y forces but combined -X repulsion.
     A face-slab has NO X corners outside the rack X range, so the drone can approach
     the aisle from the -X end without any equilibrium.
  1. Second rack aisle-face slab   — the +Y face of the second rack (toward the aisle).
  2. Aisle floor obstacle           — stacked boxes near column B.

Face slabs extend from (RACK_X_MIN, face_y ± slab_thickness/2, 0) to
(RACK_X_MAX, face_y ± slab_thickness/2, RACK_TOTAL_HEIGHT).

These are used by:
  - `drone/avoidance.py`  — APF repulsive-force computation.
  - `scripts/fly_aisle_demo.py` — clearance metric.
  - `tests/test_obstacles.py`   — unit tests.

All values are derived from sim/config.py constants.
Pure-Python module — no Isaac Sim dependency.
"""
from sim.config import (
    RACK_X_MIN, RACK_X_MAX, RACK_Y_MIN, RACK_Y_MAX, RACK_TOTAL_HEIGHT,
    SECOND_RACK_Y, BOX_D,
    OBSTACLE_X, OBSTACLE_Y,
    OBSTACLE_HALF_W, OBSTACLE_HALF_D, OBSTACLE_HALF_H,
)

# Thickness of the aisle-face slab (thin enough to model a wall face, not a volume)
_SLAB_HALF_THICKNESS = 0.10   # m (20 cm total slab; enough for clearance detection)

# Primary rack front face (box face at y = -BOX_D/2 = -0.25); use RACK_Y_MIN=-0.45 as the
# structural face (includes posts). Slab centred at y=-0.45 facing into aisle.
_PRIM_FACE_Y = float(RACK_Y_MIN)   # = -0.45 m

# Second rack front face (toward aisle = +Y face of second rack).
# Second rack box centers at y=SECOND_RACK_Y=-1.80, structural half-depth=0.45 m.
# +Y face at y = SECOND_RACK_Y + 0.45 = -1.35 m.
_SEC_FACE_Y  = float(SECOND_RACK_Y) + 0.45   # = -1.35 m

# The slabs extend in X from SLAB_X_MIN to RACK_X_MAX.  By extending the slab
# well PAST the rack left boundary (x << RACK_X_MIN), we ensure that when the
# drone approaches from -X at y=aisle_center, the nearest obstacle point is
# directly to the +Y or -Y side (not a corner), so the APF force points in ±Y
# (deflecting the drone back to centerline) rather than in -X (blocking it).
_SLAB_X_MIN = -5.0   # m — far left extension so no X-corner trap
_SLAB_X_MAX = float(RACK_X_MAX)


def _make_obs_aabb():
    """Build the aisle floor obstacle AABB."""
    return (
        (
            float(OBSTACLE_X) - float(OBSTACLE_HALF_W),
            float(OBSTACLE_Y) - float(OBSTACLE_HALF_D),
            0.0,
        ),
        (
            float(OBSTACLE_X) + float(OBSTACLE_HALF_W),
            float(OBSTACLE_Y) + float(OBSTACLE_HALF_D),
            float(OBSTACLE_HALF_H) * 2,
        ),
    )


def get_obstacle_aabbs():
    """Return list of (min_xyz, max_xyz) tuples for APF force computation.

    Uses EXTENDED face slabs (x from SLAB_X_MIN to RACK_X_MAX) to avoid
    the symmetric APF corner trap at the aisle entrance.  Only the floor
    obstacle (index 2) is used for APF forces in the FlightController;
    slabs 0 and 1 are included for clearance metrics.

    Obstacles:
      index 0 — primary rack aisle-face slab (extended in -X)
      index 1 — second rack aisle-face slab (extended in -X)
      index 2 — aisle floor obstacle (stacked boxes near column B)
    """
    prim_min = (_SLAB_X_MIN, _PRIM_FACE_Y - _SLAB_HALF_THICKNESS, 0.0)
    prim_max = (_SLAB_X_MAX, _PRIM_FACE_Y + _SLAB_HALF_THICKNESS, float(RACK_TOTAL_HEIGHT))

    sec_min = (_SLAB_X_MIN, _SEC_FACE_Y - _SLAB_HALF_THICKNESS, 0.0)
    sec_max = (_SLAB_X_MAX, _SEC_FACE_Y + _SLAB_HALF_THICKNESS, float(RACK_TOTAL_HEIGHT))

    obs = _make_obs_aabb()

    return [
        (prim_min, prim_max),   # index 0: primary rack aisle face (extended)
        (sec_min,  sec_max),    # index 1: second rack aisle face (extended)
        obs,                    # index 2: aisle floor obstacle
    ]


def get_clearance_aabbs():
    """Return AABBs for clearance metric (drone must not penetrate these).

    Uses RACK-SPAN-ONLY face slabs (x from RACK_X_MIN to RACK_X_MAX) so that
    the clearance metric correctly represents the real aisle walls only where
    rack structure exists.  The drone approach path (home → pre_entry at x=-2)
    is outside the rack X span and should NOT penalise clearance.

    Obstacles:
      index 0 — primary rack aisle-face slab (within rack X span)
      index 1 — second rack aisle-face slab (within rack X span)
      index 2 — aisle floor obstacle
    """
    prim_min = (float(RACK_X_MIN), _PRIM_FACE_Y - _SLAB_HALF_THICKNESS, 0.0)
    prim_max = (float(RACK_X_MAX), _PRIM_FACE_Y + _SLAB_HALF_THICKNESS, float(RACK_TOTAL_HEIGHT))

    sec_min = (float(RACK_X_MIN), _SEC_FACE_Y - _SLAB_HALF_THICKNESS, 0.0)
    sec_max = (float(RACK_X_MAX), _SEC_FACE_Y + _SLAB_HALF_THICKNESS, float(RACK_TOTAL_HEIGHT))

    obs = _make_obs_aabb()

    return [
        (prim_min, prim_max),
        (sec_min,  sec_max),
        obs,
    ]
