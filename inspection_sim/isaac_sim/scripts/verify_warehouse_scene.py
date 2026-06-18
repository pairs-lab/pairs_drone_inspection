"""Render an overview frame of the integrated warehouse + rack + drone scene.

Verifies that our 3x6 labeled rack sits inside the real warehouse backdrop and
that the combined scene renders correctly.

Success: prints `SCENE_OVERVIEW_OK mean=<N>` and saves
sim/assets/scene_overview.png.

Run from repo root:
    scripts/run_isaac.sh scripts/verify_warehouse_scene.py
"""

import os
import numpy as np

# 1. Boot Isaac Sim on the NVIDIA RTX GPU
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": True, "width": 1280, "height": 720,
    "active_gpu": 0, "physics_gpu": 0,
})

import carb
_s = carb.settings.get_settings()
_s.set("/renderer/activeGpu", 0)
_s.set("/rtx/materialDb/syncLoads", True)
_s.set("/rtx/hydra/materialSyncLoads", True)
_s.set("/omni.kit.plugin/syncUsdLoads", True)

import omni.usd
import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdLux

from sim.warehouse import use_local_assets, build_warehouse_env
from sim.rack import build_rack
from sim.drone_asset import spawn_drone
from sim.config import HOME_POSE, RACK_WORLD_OFFSET

# 2. Point Isaac at local asset mirror BEFORE stage edits
use_local_assets()

# 3. Build the integrated scene
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")

# Real warehouse backdrop
UsdGeom.Xform.Define(stage, "/World")
build_warehouse_env(stage, kind="warehouse")
print("[verify_warehouse_scene] loaded real warehouse.usd backdrop")

# Add a dome fill light (the rack labels are emissive so overview will be visible
# regardless, but the warehouse walls/floor need some light)
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1000.0)

# Rack under offset Xform (RACK_WORLD_OFFSET = 0,0,0 → no shift needed)
rack_root = UsdGeom.Xform.Define(stage, "/World/Rack")
ox, oy, oz = RACK_WORLD_OFFSET
if ox != 0.0 or oy != 0.0 or oz != 0.0:
    rack_root.AddTranslateOp().Set(Gf.Vec3d(ox, oy, oz))

n_bins = build_rack(stage)
print(f"[verify_warehouse_scene] built {n_bins} bins")
assert n_bins == 18

# Drone at home position
hp = HOME_POSE["position"]
drone_pos = (hp[0] + ox, hp[1] + oy, hp[2] + oz)
spawn_drone(stage, drone_pos)
print(f"[verify_warehouse_scene] drone at {drone_pos}")

# 4. Set up an oblique overview camera that shows the rack inside the warehouse.
# The rack spans X in [0, 2.4], Z in [0, 4.0], labels at y = -0.41.
# Rack centre ≈ (1.2, 0, 2.0).  Camera pulls back and up for overview.
OVERVIEW_CAM_POS = (-4.0, -6.0, 5.0)   # oblique: left, back, elevated
OVERVIEW_LOOK_AT = (1.2, 0.0, 2.0)     # rack centre

cam = rep.create.camera(
    position=OVERVIEW_CAM_POS,
    look_at=OVERVIEW_LOOK_AT,
    look_at_up_axis=(0.0, 0.0, 1.0),
    name="OverviewCam",
)
print(f"[verify_warehouse_scene] overview cam at {OVERVIEW_CAM_POS} looking at {OVERVIEW_LOOK_AT}")

# Widen near clip so close warehouse geometry doesn't get clipped
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))

rp = rep.create.render_product(cam, (1280, 720), name="OverviewRP")
annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
annot.attach([rp])

# 5. Warm up RTX (textures/materials compile async) then render real frames
print("[verify_warehouse_scene] warming up RTX ...")
for _ in range(120):
    simulation_app.update()
for _ in range(10):
    rep.orchestrator.step(rt_subframes=64, wait_for_render=True)

# 6. Pull rendered pixels
data = annot.get_data()
if data is None:
    raise RuntimeError("LdrColor annotator returned None")
arr = np.asarray(data)
rgb = arr[:, :, :3].astype(np.uint8) if arr.ndim == 3 and arr.shape[2] >= 3 else arr.astype(np.uint8)
print(f"[verify_warehouse_scene] rendered shape={rgb.shape} mean={rgb.mean():.1f}")

if rgb.mean() < 5 or float(rgb.std()) < 1.0:
    raise RuntimeError(
        f"render looks empty (mean={rgb.mean():.1f}, std={rgb.std():.1f}). "
        "Check that warehouse.usd and rack labels are visible from overview pose."
    )

# 7. Save the overview
out_path = os.path.abspath("sim/assets/scene_overview.png")
os.makedirs(os.path.dirname(out_path), exist_ok=True)
from PIL import Image
Image.fromarray(rgb).save(out_path)
print(f"[verify_warehouse_scene] saved overview -> {out_path}")
print(f"SCENE_OVERVIEW_OK mean={rgb.mean():.1f}")

simulation_app.close()
