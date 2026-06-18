"""Render a camera capture for EACH of the 18 bins and save to sim/assets/captures/<bin>.png.

Reuses the proven camera/render approach from scripts/verify_capture.py:
 - Isaac Sim 6.0, headless, NVIDIA RTX GPU (active_gpu=0)
 - near_clip = 0.01 m to avoid the default 1.0 m clip cutting close geometry
 - Per-bin: re-create camera, render product, and annotator (same approach as verify_capture.py)
 - LdrColor annotator, rt_subframes=64 for clean convergence

One Isaac session: build scene once, then loop 18 bins. For each bin:
  - Delete old camera/render-product prims
  - Create new camera at that bin's scan_pose pointing at the label center
  - Attach annotator, warm up, render, save

Success: prints RENDER_ALL_OK count=18
"""

import os
import math
import numpy as np

# 1. Boot Isaac on the NVIDIA RTX GPU
from isaacsim import SimulationApp
simulation_app = SimulationApp({
    "headless": True,
    "width": 1280,
    "height": 720,
    "active_gpu": 0,
    "physics_gpu": 0,
})

import carb
_s = carb.settings.get_settings()
_s.set("/renderer/activeGpu", 0)
_s.set("/rtx/materialDb/syncLoads", True)
_s.set("/rtx/hydra/materialSyncLoads", True)
_s.set("/omni.kit.plugin/syncUsdLoads", True)

import omni.usd
import omni.replicator.core as rep
from pxr import Gf, UsdGeom, UsdLux, Sdf
from PIL import Image

from sim.warehouse import use_local_assets, build_warehouse_env
from sim.rack import build_rack
from sim.bin_map import load_bin_map
from sim.config import BOX_D

# ---------------------------------------------------------------------------
# 2. Build scene once
# ---------------------------------------------------------------------------
use_local_assets()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
UsdGeom.Xform.Define(stage, "/World")
build_warehouse_env(stage, kind="warehouse")
n_bins = build_rack(stage)
print(f"[render_all_bins] built scene: {n_bins} bins (warehouse.usd + rack)")
assert n_bins == 18, f"expected 18 bins, got {n_bins}"

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1200.0)

# ---------------------------------------------------------------------------
# 3. Initial warm-up
# ---------------------------------------------------------------------------
print("[render_all_bins] initial RTX warm-up ...")
for _ in range(100):
    simulation_app.update()

RESOLUTION = (1280, 720)
OUT_DIR = os.path.abspath("sim/assets/captures")
os.makedirs(OUT_DIR, exist_ok=True)

bin_map = load_bin_map()
bin_ids = sorted(bin_map.keys())

# ---------------------------------------------------------------------------
# Helper to set near clip on ALL camera prims in /Replicator subtree
# ---------------------------------------------------------------------------
def set_near_clip(near=0.01, far=1_000_000.0):
    for p in stage.Traverse():
        if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
            UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(near, far))

# ---------------------------------------------------------------------------
# 4. Loop all 18 bins — re-create camera/render-product each time
#    (matches the proven recipe from verify_capture.py exactly)
# ---------------------------------------------------------------------------
ok_count = 0
failed_bins = []

for idx, bin_id in enumerate(bin_ids):
    b = bin_map[bin_id]
    sp = b["scan_pose"]["position"]   # [x, scan_y, box_center_z]
    cam_pos = (float(sp[0]), float(sp[1]), float(sp[2]))

    # Label center: same x, label_y = -BOX_D/2 - 0.005, same z as scan_pose
    label_y = -BOX_D / 2 - 0.005
    label_center = (float(sp[0]), label_y, float(sp[2]))

    print(f"[render_all_bins] [{idx+1}/18] {bin_id}: cam={cam_pos}, label={label_center}")

    # Create camera for this bin (unique name per bin to avoid prim conflicts)
    cam_name = f"DroneCamera_{bin_id}"
    cam = rep.create.camera(
        position=cam_pos,
        look_at=label_center,
        look_at_up_axis=(0.0, 0.0, 1.0),
        name=cam_name,
    )

    # Near clip MUST be set before render (default 1.0m clips label at 0.75m away)
    set_near_clip()

    # Create render product for this camera
    rp = rep.create.render_product(cam, RESOLUTION, name=f"DroneCapture_{bin_id}")
    annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
    annot.attach([rp])

    # Per-bin warm-up frames (scene already loaded; shorter than initial)
    for _ in range(20):
        simulation_app.update()
    for _ in range(8):
        rep.orchestrator.step(rt_subframes=64, wait_for_render=True)

    # Pull pixels
    data = annot.get_data()
    if data is None:
        print(f"[render_all_bins] WARNING: {bin_id} returned None from annotator")
        failed_bins.append(bin_id)
        continue

    arr = np.asarray(data)
    rgb = arr[:, :, :3].astype(np.uint8) if arr.ndim == 3 and arr.shape[2] >= 3 else arr.astype(np.uint8)

    if rgb.mean() < 5 or float(rgb.std()) < 1.0:
        print(f"[render_all_bins] WARNING: {bin_id} looks empty (mean={rgb.mean():.1f})")
        failed_bins.append(bin_id)
        continue

    out_path = os.path.join(OUT_DIR, f"{bin_id}.png")
    Image.fromarray(rgb).save(out_path)
    ok_count += 1
    print(f"[render_all_bins] {bin_id}: saved (mean={rgb.mean():.1f}, std={rgb.std():.1f})")

    # Detach annotator to free GPU resources before next bin
    annot.detach([rp])

print(f"\n[render_all_bins] done: {ok_count}/18 OK, failed={failed_bins}")
print(f"RENDER_ALL_OK count={ok_count}")

simulation_app.close()
