"""State machine that flies the drone through planned waypoints for a BIN.

States:
  IDLE      -> initial and final state (done)
  FLYING    -> moving toward the next waypoint (home->aisle->scan path)
  SCANNING  -> hovering at the scan pose, counting hold steps
  RETURNING -> moving back (scan->aisle->home)

Control: position_to_thrust() (attractive PD) + repulsive_force() (APF).
The combined 3D world-space force is applied via the inject_fn each step.

Force API (Isaac Sim 6.0): RigidPrim.apply_forces(forces_Nx3, is_global=True).
The apply_fn callable passed to FlightController must accept (force_3d, pos_3d)
and apply the force to the physics body.
"""
import numpy as np
from drone.controller import position_to_thrust
from drone.quadrotor import DRONE_MASS
from drone.waypoints import plan_waypoints
from drone.avoidance import repulsive_force, clearance

# ── Tuning constants ──────────────────────────────────────────────────────────
ARRIVE_TOL   = 0.20   # m — close enough to advance to next waypoint
SCAN_HOLD_STEPS = 120  # ~2 s at 60 Hz hold at scan pose
KP = 4.0              # position P gain
KD = 6.0              # position D gain

# APF avoidance parameters
# Influence = 0.30 m beyond obstacle surface.  This is intentionally small:
#   - Drone radius = 0.25 m, so effective_d = signed_dist - 0.25.
#   - APF activates only when drone centre is within 0.30+0.25=0.55 m of an obstacle surface.
#   - Aisle is 1.30 m wide; drone at centreline is 0.65 m from each surface → no repulsion.
#   - Only activates as "last resort" when drone drifts close to a wall or obstacle.
# This prevents APF equilibria far from obstacles (like at the aisle entrance approach).
APF_INFLUENCE   = 0.30  # m — interaction radius beyond obstacle surface
APF_GAIN        = 1.0   # repulsive gain — gentler far from obstacles, very strong near contact
DRONE_SAFETY_R  = 0.25  # m — drone radius for effective-distance calc


class FlightController:
    """State-machine flight controller with APF obstacle avoidance.

    Args:
        localization: LocalizationProvider (OracleLocalization in POC).
        apply_force_fn: callable(force_3d: np.ndarray, pos_3d: np.ndarray)
            — applies the given world-space 3D force to the drone rigid body.
        dt: physics step duration (s).
        aabbs: list of obstacle AABBs (from sim.obstacles.get_obstacle_aabbs()).
               If None, avoidance is disabled (backward-compat with fly_to_bin.py).
    """

    def __init__(self, localization, apply_force_fn, dt=1.0 / 60.0, aabbs=None,
                 clearance_aabbs=None):
        self.loc = localization
        self._apply_force = apply_force_fn
        self.dt = dt
        self.state = "IDLE"
        self._aabbs = aabbs or []
        # Separate AABB set for clearance metric (smaller; represents real wall surfaces).
        # If not provided, uses same as aabbs.
        self._clearance_aabbs = clearance_aabbs if clearance_aabbs is not None else self._aabbs

    def _step_toward(self, target_pos: np.ndarray):
        """Compute + apply total 3D force toward target with APF avoidance.

        Returns:
            (distance_error, clearance_value)
        """
        pos, _yaw = self.loc.get_pose()
        vel = self.loc.get_velocity()

        # Attractive force (PD toward waypoint)
        f_attr = position_to_thrust(
            pos, vel, target_pos, DRONE_MASS, g=9.81, kp=KP, kd=KD
        )

        # Repulsive force (APF away from obstacles).
        # Only apply APF to the AISLE OBSTACLE (index 2) — the stacked boxes on the
        # floor.  Rack walls (indices 0 and 1) are avoided structurally by the
        # aisle-centerline waypoints; adding APF repulsion from them creates
        # equilibria that prevent the drone from travelling along the aisle.
        f_rep = np.zeros(3, dtype=float)
        if self._aabbs:
            obs_only = [self._aabbs[2]] if len(self._aabbs) > 2 else self._aabbs
            f_rep = repulsive_force(
                pos, obs_only,
                influence=APF_INFLUENCE,
                gain=APF_GAIN,
                safety_margin=DRONE_SAFETY_R,
            )

        total_force = f_attr + f_rep
        self._apply_force(total_force, pos)

        err = float(np.linalg.norm(pos - target_pos))
        clr = clearance(pos, self._clearance_aabbs) if self._clearance_aabbs else float("inf")
        return err, clr

    def run(self, bin_id: str, world_step_fn, max_steps: int = 8000):
        """Fly the drone to a bin's scan pose and back to home.

        Args:
            bin_id: BIN identifier (e.g. "B3" or "C3").
            world_step_fn: callable() — advances Isaac Sim by one physics step.
            max_steps: safety cap on total simulation steps.

        Returns:
            (reached: bool, min_clearance: float, collision_steps: int)
            reached        — True if drone returned home within max_steps.
            min_clearance  — minimum clearance observed during the full flight.
            collision_steps — number of steps where clearance <= 0 (penetration).
        """
        wps = plan_waypoints(bin_id)
        wp_idx = 0
        scan_hold = 0
        self.state = "FLYING"
        min_clr = float("inf")
        collision_steps = 0

        for step in range(max_steps):
            wp = wps[wp_idx]
            target = np.array(wp.position)
            err, clr = self._step_toward(target)
            world_step_fn()

            # Track clearance metrics
            if clr < min_clr:
                min_clr = clr
            if clr <= 0.0:
                collision_steps += 1

            if wp.label == "scan":
                self.state = "SCANNING"
                if err < ARRIVE_TOL:
                    scan_hold += 1
                    if scan_hold >= SCAN_HOLD_STEPS:
                        wp_idx += 1
                        scan_hold = 0
            elif err < ARRIVE_TOL:
                wp_idx += 1
                if wp_idx >= len(wps):
                    self.state = "IDLE"
                    return True, min_clr, collision_steps
                next_label = wps[wp_idx].label
                if next_label == "home" and wp.label not in ("home", "aisle_exit"):
                    self.state = "RETURNING"
                else:
                    self.state = "FLYING"

        # Timed out
        return False, min_clr, collision_steps
