"""Pose source for navigation. POC uses OracleLocalization (ground-truth from sim).
Interface kept minimal so a VIO implementation can replace it without touching nav."""
import numpy as np

class LocalizationProvider:
    def get_pose(self):
        """Return (position np.array[3], yaw_rad float)."""
        raise NotImplementedError
    def get_velocity(self):
        """Return linear velocity np.array[3] (world)."""
        raise NotImplementedError

class OracleLocalization(LocalizationProvider):
    """Reads true pose/velocity of a RigidPrim view from the running sim.

    Isaac Sim 6.0 RigidPrim is a BATCHED view — all getter methods return
    arrays with shape (N, ...) for N matched prims.  For a single drone we
    use index [0] to extract the scalar values.

    API confirmed against Isaac Sim 6.0 binary:
      get_world_poses()        -> (positions (N,3), orientations (N,4) wxyz)
      get_linear_velocities()  -> velocities (N,3)
    """
    def __init__(self, rigid_prim):
        self._rb = rigid_prim   # isaacsim RigidPrim view wrapping /World/Drone/Body
    def get_pose(self):
        positions, orientations = self._rb.get_world_poses()
        pos = np.asarray(positions[0], dtype=float).reshape(3)
        # orientation is (w, x, y, z) in Isaac Sim
        q = np.asarray(orientations[0], dtype=float).reshape(4)
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
        return pos, yaw
    def get_velocity(self):
        vels = self._rb.get_linear_velocities()
        return np.asarray(vels[0], dtype=float).reshape(3)
