"""Narrow-aisle obstacle-avoidance demo.

Builds: warehouse env + primary rack (18 bins, QR labels) +
        second (mirror) rack (obstacle wall) + aisle floor obstacle +
        drone at HOME_POSE.
Flies to the requested BIN through the narrow corridor using APF avoidance.
Records clearance every step and physics collision count.

Usage:
    scripts/run_isaac.sh scripts/fly_aisle_demo.py [BIN_ID]
    (default BIN_ID = C3, which is behind the column-B aisle obstacle)

Prints at end:
    AISLE_DEMO bin=<id> reached=<bool> min_clearance=<m> collisions=<n>

Asserts:
    reached==True, min_clearance > 0, collisions == 0

Also renders:
    sim/assets/aisle_overview.png  — oblique view showing both racks + aisle + drone
    sim/assets/aisle_topdown.png   — top-down view showing narrow corridor + obstacle
"""
import sys
import os
import numpy as np

# ── Isaac Sim boot ────────────────────────────────────────────────────────────
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0,
                     "width": 1280, "height": 720})

import carb
_s = carb.settings.get_settings()
_s.set("/renderer/activeGpu", 0)
_s.set("/rtx/materialDb/syncLoads", True)
_s.set("/rtx/hydra/materialSyncLoads", True)

import omni.usd
import omni.replicator.core as rep
from pxr import UsdGeom, UsdLux, Gf
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim

from sim.warehouse import build_warehouse
from sim.rack import build_rack, build_second_rack, build_aisle_obstacle
from sim.drone_asset import spawn_drone
from sim.config import HOME_POSE, AISLE_CENTER_Y, AISLE_WIDTH
from sim.obstacles import get_obstacle_aabbs, get_clearance_aabbs
from drone.quadrotor import add_physics, DRONE_MASS
from drone.localization import OracleLocalization
from drone.flight import FlightController
from drone.waypoints import plan_waypoints
from drone.avoidance import clearance

# ── Parse CLI arg ─────────────────────────────────────────────────────────────
bin_id = sys.argv[1] if len(sys.argv) > 1 else "C3"
print(f"[fly_aisle_demo] target bin = {bin_id}")

# ── Scene setup ───────────────────────────────────────────────────────────────
world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")

build_warehouse(stage)
print("[fly_aisle_demo] building primary rack (18 bins) ...")
build_rack(stage)
print("[fly_aisle_demo] building second (mirror) rack ...")
build_second_rack(stage)
print("[fly_aisle_demo] placing aisle floor obstacle ...")
build_aisle_obstacle(stage)

# Dome fill light
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1500.0)

home_pos = tuple(HOME_POSE["position"])
spawn_drone(stage, home_pos)
add_physics(stage)

# ── Physics reset ─────────────────────────────────────────────────────────────
rb = RigidPrim("/World/Drone/Body", name="drone_body")
world.reset()

# Read contact-report API if available (Isaac 6.0 rigid body contacts)
def _get_contact_count():
    """Return the number of contact events on the drone body (best-effort)."""
    try:
        # Isaac Sim 6.0: rigid body contact reporter via physx
        contacts = rb.get_contact_force_matrix()
        if contacts is not None:
            forces = np.asarray(contacts)
            return int(np.sum(np.abs(forces) > 0.1))
    except Exception:
        pass
    return 0

# ── Obstacle AABBs ────────────────────────────────────────────────────────────
aabbs = get_obstacle_aabbs()
clr_aabbs = get_clearance_aabbs()
print(f"[fly_aisle_demo] obstacle AABBs (APF): {len(aabbs)} volumes")
for i, (mn, mx) in enumerate(aabbs):
    print(f"  [{i}] min={[f'{v:.2f}' for v in mn]} max={[f'{v:.2f}' for v in mx]}")
print(f"[fly_aisle_demo] clearance AABBs: {len(clr_aabbs)} volumes")

# ── Force application callback ────────────────────────────────────────────────
def apply_force_fn(force_3d: np.ndarray, pos_3d: np.ndarray):
    rb.apply_forces(force_3d.reshape(1, 3), is_global=True)

# ── Fly with APF avoidance ────────────────────────────────────────────────────
loc = OracleLocalization(rb)
fc = FlightController(loc, apply_force_fn, aabbs=aabbs, clearance_aabbs=clr_aabbs)

print(f"[fly_aisle_demo] starting flight to {bin_id} ...")
reached, min_clr, collision_steps = fc.run(
    bin_id, lambda: world.step(render=False), max_steps=15000
)
print(f"[fly_aisle_demo] flight done: reached={reached} "
      f"min_clearance={min_clr:.4f} collision_steps={collision_steps}")

# ── Render overview images ─────────────────────────────────────────────────────
print("[fly_aisle_demo] rendering overview images ...")
# Position drone at aisle midpoint for the render (visual confirmation)
# (drone may already be near home after the flight; the render captures the static scene)

os.makedirs("sim/assets", exist_ok=True)

# Warm up RTX
for _ in range(80):
    app.update()

# --- Oblique overview ---
# Camera placed at a high oblique angle to show both racks and the narrow aisle
oblique_cam_pos = (-3.0, -4.0, 5.0)   # above-and-behind, looking at scene centre
scene_center = (1.2, -0.9, 1.5)       # roughly between the two racks, mid-height

cam_oblique = rep.create.camera(
    position=oblique_cam_pos,
    look_at=scene_center,
    look_at_up_axis=(0.0, 0.0, 1.0),
    name="OverviewCam",
)
rp_oblique = rep.create.render_product(cam_oblique, (1280, 720), name="OverviewCapture")
annot_oblique = rep.AnnotatorRegistry.get_annotator("LdrColor")
annot_oblique.attach([rp_oblique])

# Set near clip
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))

for _ in range(10):
    rep.orchestrator.step(rt_subframes=32, wait_for_render=True)

oblique_data = annot_oblique.get_data()
if oblique_data is not None:
    import PIL.Image
    arr = np.asarray(oblique_data)
    rgb = arr[:, :, :3].astype(np.uint8) if arr.ndim == 3 and arr.shape[2] >= 3 else arr.astype(np.uint8)
    PIL.Image.fromarray(rgb).save("sim/assets/aisle_overview.png")
    print(f"[fly_aisle_demo] saved sim/assets/aisle_overview.png "
          f"mean={rgb.mean():.1f} shape={rgb.shape}")
else:
    print("[fly_aisle_demo] WARNING: oblique render returned None")

# --- Top-down view ---
topdown_cam_pos = (1.2, -0.9, 8.0)    # directly above scene centre
cam_topdown = rep.create.camera(
    position=topdown_cam_pos,
    look_at=(1.2, -0.9, 0.0),
    look_at_up_axis=(1.0, 0.0, 0.0),   # +X as "up" in image for top-down Z-up scene
    name="TopdownCam",
)
rp_topdown = rep.create.render_product(cam_topdown, (1280, 720), name="TopdownCapture")
annot_topdown = rep.AnnotatorRegistry.get_annotator("LdrColor")
annot_topdown.attach([rp_topdown])

for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))

for _ in range(10):
    rep.orchestrator.step(rt_subframes=32, wait_for_render=True)

topdown_data = annot_topdown.get_data()
if topdown_data is not None:
    import PIL.Image
    arr = np.asarray(topdown_data)
    rgb = arr[:, :, :3].astype(np.uint8) if arr.ndim == 3 and arr.shape[2] >= 3 else arr.astype(np.uint8)
    PIL.Image.fromarray(rgb).save("sim/assets/aisle_topdown.png")
    print(f"[fly_aisle_demo] saved sim/assets/aisle_topdown.png "
          f"mean={rgb.mean():.1f} shape={rgb.shape}")
else:
    print("[fly_aisle_demo] WARNING: top-down render returned None")

# ── Report + assert ───────────────────────────────────────────────────────────
print(f"\nAISLE_DEMO bin={bin_id} reached={reached} "
      f"min_clearance={min_clr:.4f} collisions={collision_steps}")

assert reached, f"Drone did not reach home! (reached={reached})"
assert min_clr > 0, (
    f"Drone penetrated an obstacle! min_clearance={min_clr:.4f} m (must be > 0)"
)
assert collision_steps == 0, (
    f"Physics collision detected: collision_steps={collision_steps}"
)

print("[fly_aisle_demo] ALL ASSERTIONS PASSED — demo DONE")
app.close()
