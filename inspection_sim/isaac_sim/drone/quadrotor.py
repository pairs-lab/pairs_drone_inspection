"""Create a physics rigid-body drone and apply forces for navigation.

Isaac Sim 6.0 API notes (confirmed against binary):
  - RigidPrim is a BATCHED view: prim_paths_expr can be a single path string.
    All methods use plural forms: get_world_poses(), get_linear_velocities(),
    apply_forces(forces), etc.
  - apply_forces(forces, is_global=True): forces shape (N, 3), applies COM force.
  - apply_forces_and_torques_at_pos(forces, torques, positions, is_global=True):
    useful for multi-point forces; positions shape (N, 3).
  - Force API applies to the whole rigid body at COM, so we compute the NET 3D
    world-space force from position_to_thrust() and apply it directly. This moves
    the drone in X, Y and Z (genuine rigid-body dynamics, not just altitude control).

Control strategy:
  - position_to_thrust(pos, vel, target, mass, g) returns a 3D world force vector
    that accelerates the drone toward target with gravity compensation.
  - We apply this NET force at the drone COM each physics step.
  - The rotor offsets / motor mixing are kept for optional visual torque effects;
    for the POC the net force is sufficient for stable 3D flight.
"""
import numpy as np
from pxr import UsdGeom, UsdPhysics, Gf

ROTOR_ARM = 0.12  # m, half-spacing of rotors in body frame (X config)
# rotor offsets in body frame: FL, FR, RL, RR
ROTOR_OFFSETS = np.array([
    [-ROTOR_ARM,  ROTOR_ARM, 0.0],
    [ ROTOR_ARM,  ROTOR_ARM, 0.0],
    [-ROTOR_ARM, -ROTOR_ARM, 0.0],
    [ ROTOR_ARM, -ROTOR_ARM, 0.0],
])
DRONE_MASS = 1.0  # kg


def add_physics(stage, body_path="/World/Drone/Body"):
    """Make the drone body a dynamic rigid body with mass and collision.

    The drone body must already exist as a prim (created by spawn_drone).
    Applies UsdPhysics.RigidBodyAPI, MassAPI (mass=DRONE_MASS), and
    CollisionAPI so Isaac Sim treats it as a simulated rigid body.

    Returns:
        str: the body_path (for chaining).
    """
    prim = stage.GetPrimAtPath(body_path)
    if not prim.IsValid():
        raise ValueError(
            f"Prim not found at {body_path!r}. "
            "Call spawn_drone() before add_physics()."
        )
    UsdPhysics.RigidBodyAPI.Apply(prim)
    UsdPhysics.CollisionAPI.Apply(prim)
    massapi = UsdPhysics.MassAPI.Apply(prim)
    massapi.CreateMassAttr(DRONE_MASS)
    return body_path
