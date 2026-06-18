"""Verify Isaac Sim 6.0 import + headless launch. Run with isaac6 python.

Usage: OMNI_KIT_ACCEPT_EULA=YES conda run -n isaac6 python scripts/verify_isaac.py
"""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd  # noqa: E402
from pxr import UsdGeom  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
print("ISAAC_OK stage_prims=", len(list(stage.Traverse())))
app.close()
