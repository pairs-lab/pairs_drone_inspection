"""Dung nen + anh sang. Goi sau khi da co stage."""
import os
from pxr import Usd, UsdGeom, UsdLux


def build_warehouse(stage: Usd.Stage):
    """Primitive fallback: flat ground plane + distant light (no asset dependency)."""
    UsdGeom.Xform.Define(stage, "/World")
    plane = UsdGeom.Mesh.Define(stage, "/World/Ground")
    plane.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000.0)
    return "/World"


def use_local_assets():
    """Point Isaac's asset resolver at the local mirror. Call before stage edits."""
    import carb
    carb.settings.get_settings().set(
        "/persistent/isaac/asset_root/default", os.path.expanduser("~/isaacsim_assets"))


def build_warehouse_env(stage, kind="warehouse", prim_path="/World/Warehouse"):
    """Reference a real Simple_Warehouse env USD as the backdrop (floor+walls+lights).

    Args:
        stage: USD stage to add prims to.
        kind: Which warehouse variant to load. Options:
            - "warehouse"  (default) — open shell, no built-in shelves, PREFERRED
            - "full_warehouse" — has built-in shelves + forklift
            - "warehouse_multiple_shelves" — multiple shelf rows
        prim_path: USD prim path for the warehouse reference.

    Returns:
        str: prim_path of the added Xform.
    """
    from pxr import UsdGeom  # noqa: F811 (re-import safe here)
    root = os.path.expanduser("~/isaacsim_assets")
    usd = f"{root}/Isaac/Environments/Simple_Warehouse/{kind}.usd"
    assert os.path.exists(usd), usd
    # Ensure /World exists as an Xform before adding children
    if not stage.GetPrimAtPath("/World"):
        UsdGeom.Xform.Define(stage, "/World")
    xf = UsdGeom.Xform.Define(stage, prim_path)
    xf.GetPrim().GetReferences().AddReference(usd)
    return prim_path
