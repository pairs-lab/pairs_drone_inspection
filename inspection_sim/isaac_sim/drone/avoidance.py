"""Artificial Potential Field (APF) obstacle avoidance for the drone.

Public API:
  repulsive_force(pos, aabbs, influence=0.6, gain=3.0, safety_margin=0.25)
    -> np.ndarray shape (3,)  — world-space repulsive force (N) away from obstacles.

  clearance(pos, aabbs)
    -> float  — minimum signed distance from pos to any AABB surface (m).
                Negative means the drone centre is INSIDE an obstacle.

Pure-Python / NumPy — no Isaac Sim dependency.

APF formula (per obstacle, when d < influence):
    magnitude = gain * (1/d - 1/influence) / d^2
where d = max(signed_dist - safety_margin, eps) is the effective distance.
Force direction: unit vector away from the closest point on the AABB.
"""
import numpy as np


def _closest_point_on_aabb(pos, aabb_min, aabb_max):
    """Return the closest point on the AABB surface/interior to pos."""
    return np.clip(pos, aabb_min, aabb_max)


def _signed_dist_to_aabb(pos, aabb_min, aabb_max):
    """Return signed distance from pos to the AABB.

    > 0: pos is outside the box (distance to nearest surface).
    = 0: pos is exactly on the surface.
    < 0: pos is inside the box (negative penetration depth).
    """
    mn = np.asarray(aabb_min, dtype=float)
    mx = np.asarray(aabb_max, dtype=float)
    # Per-axis: positive = outside, negative = inside
    d_pos_side = pos - mx   # dist to max face (+ve if outside on max side)
    d_neg_side = mn - pos   # dist to min face (+ve if outside on min side)
    # Component-wise max: positive means "how far outside this axis"
    outside = np.maximum(d_pos_side, d_neg_side)   # shape (3,)
    # If all axes ≤ 0 the point is inside: signed dist = max(outside) (negative)
    # If at least one axis > 0 the point is outside: signed dist = ||positive part||
    pos_outside = np.maximum(outside, 0.0)
    if np.any(outside > 0.0):
        return float(np.linalg.norm(pos_outside))
    else:
        return float(np.max(outside))   # ≤ 0


def repulsive_force(pos, aabbs, influence=0.6, gain=3.0, safety_margin=0.25):
    """Compute APF repulsive force from all obstacle AABBs.

    Args:
        pos:           np.ndarray (3,) — drone world position.
        aabbs:         list of (min_xyz, max_xyz) tuples.
        influence:     float — interaction radius beyond AABB surface (m).
        gain:          float — repulsive gain constant.
        safety_margin: float — drone radius added to effective distance.
                                (APF treats the drone as a sphere of this radius.)

    Returns:
        np.ndarray (3,) — repulsive force vector (N), world space.
                          Zero if outside influence for all obstacles.
    """
    pos = np.asarray(pos, dtype=float)
    total_force = np.zeros(3, dtype=float)
    max_magnitude = 50.0  # N — clamp so gains don't cause instability

    for (aabb_min, aabb_max) in aabbs:
        mn = np.asarray(aabb_min, dtype=float)
        mx = np.asarray(aabb_max, dtype=float)

        sd = _signed_dist_to_aabb(pos, mn, mx)

        # Effective distance: subtract drone radius; clamp to tiny positive
        eps = 1e-4
        d = max(sd - safety_margin, eps)

        if d >= influence:
            continue   # outside influence sphere — no contribution

        # Direction: away from closest surface point
        cp = _closest_point_on_aabb(pos, mn, mx)
        direction = pos - cp
        dir_norm = np.linalg.norm(direction)
        if dir_norm < eps:
            # Pos is inside the AABB — push straight up as emergency
            direction = np.array([0.0, 0.0, 1.0])
        else:
            direction = direction / dir_norm

        # APF magnitude: grows steeply as d -> 0
        magnitude = gain * (1.0 / d - 1.0 / influence) / (d ** 2)
        magnitude = min(magnitude, max_magnitude)   # clamp

        total_force += magnitude * direction

    # Final clamp on total
    total_norm = np.linalg.norm(total_force)
    if total_norm > max_magnitude:
        total_force = total_force / total_norm * max_magnitude

    return total_force


def clearance(pos, aabbs):
    """Return minimum signed distance from pos to any AABB surface.

    Positive  -> drone is outside all obstacles (clearance in metres).
    Zero      -> touching a surface.
    Negative  -> drone centre is inside an obstacle (penetration).
    """
    pos = np.asarray(pos, dtype=float)
    if not aabbs:
        return float("inf")
    return min(
        _signed_dist_to_aabb(pos, np.asarray(mn, dtype=float), np.asarray(mx, dtype=float))
        for (mn, mx) in aabbs
    )
