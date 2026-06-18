"""Task 4 verify script: drone rigid body + PID hover test.

Spawns drone at (0,0,1), applies physics, then runs a PID position controller
for 10s applying the FULL 3D world-space net force each step.  Asserts that
the final position error from the hover target (0,0,1.5) is < 0.3 m.

Force API used (Isaac Sim 6.0 binary, confirmed):
  RigidPrim.apply_forces(forces, is_global=True)
    forces: np.ndarray shape (N, 3) — world-space force in Newtons applied at COM.

Control: position_to_thrust() computes gravity-compensated 3D world force vector,
applied directly as the net body force each step (no altitude-only approximation).
Gains kp=4.0, kd=6.0 were tuned for stability at dt=1/60 s.
"""
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0})

import omni.usd
from pxr import UsdGeom
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from sim.warehouse import build_warehouse
from sim.drone_asset import spawn_drone
from drone.quadrotor import add_physics, DRONE_MASS
from drone.controller import position_to_thrust
from drone.localization import OracleLocalization

# ── Scene setup ──────────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
build_warehouse(stage)

START = (0.0, 0.0, 1.0)
spawn_drone(stage, START)
add_physics(stage)

# RigidPrim view — Isaac 6.0: all methods batched, pass prim path expression.
rb = RigidPrim("/World/Drone/Body", name="drone_body")

world.reset()

# ── Control loop ─────────────────────────────────────────────────────────────
loc = OracleLocalization(rb)
target = np.array([0.0, 0.0, 1.5])   # hover 0.5 m above start

# Tuned gains: kp=4.0 kd=6.0 for dt=1/60s (critically damped, no oscillation)
KP = 4.0
KD = 6.0
dt = 1.0 / 60.0
N_STEPS = 600  # 10 s

for i in range(N_STEPS):
    pos, _yaw = loc.get_pose()
    vel = loc.get_velocity()
    # Full 3D world-space force (gravity-compensated PD controller)
    force = position_to_thrust(pos, vel, target, DRONE_MASS, g=9.81, kp=KP, kd=KD)
    # apply_forces: shape (N_prims, 3) — single drone → shape (1, 3)
    rb.apply_forces(force.reshape(1, 3), is_global=True)
    world.step(render=False)

# ── Verify ───────────────────────────────────────────────────────────────────
pos, _ = loc.get_pose()
err = float(np.linalg.norm(pos - target))
print(f"HOVER_OK final_pos={pos.round(3)} err={err:.3f}")
assert err < 0.3, f"Hover error too large: {err:.3f} >= 0.3 m"

app.close()
