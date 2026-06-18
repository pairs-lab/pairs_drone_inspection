"""Verify the locally-downloaded Isaac warehouse asset loads and find missing deps.

Points Isaac's asset root at the local mirror (~/isaacsim_assets), references
full_warehouse.usd, computes all dependencies, reports any that are missing on
disk, then renders one frame to confirm it loads.
"""
import os
import numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0})

import carb
LOCAL_ROOT = os.path.expanduser("~/isaacsim_assets")
# Make Isaac resolve {assets_root}/... against the local mirror
carb.settings.get_settings().set("/persistent/isaac/asset_root/default", LOCAL_ROOT)

import omni.usd
import omni.replicator.core as rep
from pxr import Usd, UsdGeom, UsdUtils, Gf
from PIL import Image

WH = os.path.join(LOCAL_ROOT, "Isaac/Environments/Simple_Warehouse/full_warehouse.usd")
assert os.path.exists(WH), f"missing {WH}"

# dependency audit
layers, assets, unresolved = UsdUtils.ComputeAllDependencies(WH)
missing = [a for a in assets if not os.path.exists(a)] + list(unresolved)
print(f"DEPS layers={len(layers)} assets={len(assets)} unresolved={len(unresolved)}")
for m in missing[:20]:
    print("  MISSING:", m)
print(f"MISSING_COUNT={len(missing)}")

# load into a stage + render
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
UsdGeom.Xform.Define(stage, "/World")
ref = UsdGeom.Xform.Define(stage, "/World/Warehouse")
ref.GetPrim().GetReferences().AddReference(WH)

cam = rep.create.camera(position=(-8.0, -8.0, 3.0), look_at=(2.0, 2.0, 1.2), name="AssetCam")
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1e6))
rp = rep.create.render_product(cam, (1280, 720), name="AssetRP")
annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
annot.attach([rp])
for _ in range(120):
    app.update()
for _ in range(8):
    rep.orchestrator.step(rt_subframes=32, wait_for_render=True)
arr = np.asarray(annot.get_data())[:, :, :3].astype(np.uint8)
Image.fromarray(arr).save("sim/assets/warehouse_view.png")
print(f"WAREHOUSE_RENDER mean={arr.mean():.1f} saved=sim/assets/warehouse_view.png")
app.close()
