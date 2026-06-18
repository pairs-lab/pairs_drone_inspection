"""Render the drone camera at bin A1 and decode the QR from REAL rendered pixels.

Success: prints `CAPTURE_QR_OK part_no=PN-A01 qty=11` and saves
sim/assets/capture_A1.png (the genuine RTX render, NOT a composite).

Now also loads the real warehouse.usd backdrop so the capture is verified in
the integrated scene (warehouse env + rack).  The rack GR-labels are emissive
so warehouse lighting does not affect decode quality.  The warehouse is loaded
via use_local_assets() + build_warehouse_env() before build_rack() is called.

Root-cause history (see docs/superpowers/INSTALL_NOTES.md "Task 6"):
  * The label texture rendered fine all along. The frame looked uniformly gray
    because the replicator camera's DEFAULT NEAR CLIP PLANE is 1.0 m, and the
    label sits only ~0.49 m from the (0,-0.9,0) scan pose, so ALL nearby
    geometry was clipped while only the infinitely-far sky/dome rendered.
  * Fix: (a) force the NVIDIA RTX GPU (active_gpu=0; the Intel iGPU is otherwise
    considered), (b) set the camera clippingRange near plane to 0.01 m, and
    (c) stand the camera back ~0.75 m from the box face so the small 0.28m QR
    fills enough pixels to decode at 1280px width.

Scene geometry (Rack v2 — Z-up):
  * Box: 0.70 x 0.50 x 0.50 m (W x D x H).  A1 shelf_top_z=0, box_center_z=0.25.
  * Box front face at y = -BOX_D/2 = -0.25.
  * A1 label center: (0, -0.255, 0.25) — 5 mm proud of box front face, mid-height.
  * Label size: 0.28 m wide x 0.20 m tall (small sticker, NOT a giant sheet).
  * A1 scan_pose.y = -(BOX_D/2 + SCAN_STANDOFF) = -(0.25 + 0.75) = -1.0 m.
  * Camera at (0, -1.0, 0.25) looking at label center (0, -0.255, 0.25), up +Z.
    Camera is ~0.745 m from the label face — small label fills ~22% of 1280px width,
    which is sufficient for pyzbar to decode with rt_subframes=64.
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
from PIL import Image
from pyzbar.pyzbar import decode

from sim.warehouse import use_local_assets, build_warehouse_env
from sim.rack import build_rack
from sim.gr_label import decode_payload
from sim.config import BOX_D, BOX_H, SCAN_STANDOFF

# 2. Point Isaac at local asset mirror, then build integrated scene:
#    real warehouse backdrop + 3x6 rack (all 18 textured emissive labels)
use_local_assets()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
UsdGeom.Xform.Define(stage, "/World")
build_warehouse_env(stage, kind="warehouse")
n_bins = build_rack(stage)
print(f"[verify_capture] built scene: {n_bins} bins (warehouse.usd + rack)")
assert n_bins == 18, f"expected 18 bins, got {n_bins}"

# Dome fill light (the labels are emissive, so this only softens the scene)
UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1200.0)

# 3. Drone scan camera at the A1 scan pose.
# A1 geometry (Rack v2):
#   shelf_top_z = 0 (level 1), box_center_z = BOX_H/2 = 0.25
#   label center: (0, -BOX_D/2 - 0.005, box_center_z) = (0, -0.255, 0.25)
#   scan_pose.y = -(BOX_D/2 + SCAN_STANDOFF) = -(0.25 + 0.75) = -1.0
# Camera placed at scan_pose so it reads from bin_map dynamically for A1.
_A1_BOX_CENTER_Z = BOX_H / 2            # = 0.25 (A1 shelf_top=0)
_A1_LABEL_Y = -(BOX_D / 2) - 0.005     # = -0.255
LABEL_CENTER = (0.0, _A1_LABEL_Y, _A1_BOX_CENTER_Z)
# Camera at scan_pose.y with same z as label center; slight pull back for clear frame
CAM_Y = -(BOX_D / 2 + SCAN_STANDOFF)   # = -1.0
CAM_POS = (0.0, CAM_Y, _A1_BOX_CENTER_Z)
RESOLUTION = (1280, 720)

cam = rep.create.camera(position=CAM_POS, look_at=LABEL_CENTER,
                        look_at_up_axis=(0.0, 0.0, 1.0), name="DroneCamera")
print(f"[verify_capture] camera at {CAM_POS} looking at {LABEL_CENTER}")

# CRITICAL: default near clip is 1.0 m and would clip the ~1.3 m-away label edges
# / nearer geometry; widen the frustum so all rack geometry is visible.
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))

rp = rep.create.render_product(cam, RESOLUTION, name="DroneCapture")
annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
annot.attach([rp])

# 4. Warm up RTX (texture/material compile is async) then render real frames
print("[verify_capture] warming up RTX ...")
for _ in range(100):
    simulation_app.update()
for _ in range(10):
    rep.orchestrator.step(rt_subframes=64, wait_for_render=True)

# 5. Pull the genuine rendered pixels
data = annot.get_data()
if data is None:
    raise RuntimeError("LdrColor annotator returned None")
arr = np.asarray(data)
rgb = arr[:, :, :3].astype(np.uint8) if arr.ndim == 3 and arr.shape[2] >= 3 else arr.astype(np.uint8)
print(f"[verify_capture] rendered shape={rgb.shape} mean={rgb.mean():.1f}")
if rgb.mean() < 5 or float(rgb.std()) < 1.0:
    raise RuntimeError(f"render looks empty (mean={rgb.mean():.1f}, std={rgb.std():.1f})")

# 6. Save the REAL capture (no compositing)
out_path = os.path.abspath("sim/assets/capture_A1.png")
os.makedirs(os.path.dirname(out_path), exist_ok=True)
img = Image.fromarray(rgb)
img.save(out_path)
print(f"[verify_capture] saved real capture -> {out_path}")

# 7. Decode the QR from the SAVED image
decoded = decode(Image.open(out_path))
assert decoded, (
    f"No QR decoded from REAL render {out_path} (mean={rgb.mean():.1f}). "
    "Open the image to inspect."
)
pn, qty = decode_payload(decoded[0].data.decode())
print(f"CAPTURE_QR_OK part_no={pn} qty={qty}")
assert pn == "PN-A01", f"expected PN-A01, got {pn}"
assert qty == 11, f"expected 11, got {qty}"

simulation_app.close()
