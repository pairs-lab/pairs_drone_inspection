"""Task 5 entrypoint: fly drone to a BIN's scan pose and return home.

Usage:
    scripts/run_isaac.sh scripts/fly_to_bin.py <BIN_ID>

Example:
    scripts/run_isaac.sh scripts/fly_to_bin.py B3

Prints:
    FLY_DONE bin=<id> reached_home=<bool> final_state=<state>
    SCAN_ERR=<distance from scan pose at end of run>

Force API used (Isaac Sim 6.0 binary):
    RigidPrim.apply_forces(forces, is_global=True)
    forces shape: (N_prims, 3)  — world-space Newtons applied at COM.

Control: position_to_thrust() with kp=4.0, kd=6.0 (same as hover, stable).
ARRIVE_TOL=0.20 m, max_steps=6000 (~100 s at 60 Hz).
"""
import sys
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0})

import omni.usd
from pxr import UsdGeom
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim
from sim.warehouse import build_warehouse
from sim.rack import build_rack
from sim.drone_asset import spawn_drone
from sim.config import HOME_POSE
from drone.quadrotor import add_physics
from drone.localization import OracleLocalization
from drone.flight import FlightController
from drone.waypoints import plan_waypoints

# ── Parse CLI arg ─────────────────────────────────────────────────────────────
bin_id = sys.argv[1] if len(sys.argv) > 1 else "B3"

# ── Scene setup ───────────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
build_warehouse(stage)
build_rack(stage)

home_pos = tuple(HOME_POSE["position"])
spawn_drone(stage, home_pos)
add_physics(stage)

rb = RigidPrim("/World/Drone/Body", name="drone_body")
world.reset()

# ── Force application callback (full 3D net force at COM) ────────────────────
def apply_force_fn(force_3d: np.ndarray, pos_3d: np.ndarray):
    """Apply world-space 3D net force to the drone rigid body COM."""
    # RigidPrim.apply_forces expects shape (N_prims, 3)
    rb.apply_forces(force_3d.reshape(1, 3), is_global=True)

# ── Fly ───────────────────────────────────────────────────────────────────────
loc = OracleLocalization(rb)
fc = FlightController(loc, apply_force_fn)
reached_home, _min_clr, _col = fc.run(bin_id, lambda: world.step(render=False), max_steps=6000)

# ── Report ────────────────────────────────────────────────────────────────────
final_pos, _ = loc.get_pose()
scan_wps = [w for w in plan_waypoints(bin_id) if w.label == "scan"]
scan_pos = np.array(scan_wps[0].position)
scan_err = float(np.linalg.norm(final_pos - scan_pos))

print(f"FLY_DONE bin={bin_id} reached_home={reached_home} final_state={fc.state}")
print(f"SCAN_ERR={scan_err:.3f} final_pos={final_pos.round(3)}")

app.close()
