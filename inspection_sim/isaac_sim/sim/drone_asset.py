"""Spawn a drone body (real CAD USD or primitive quadrotor fallback) + a camera prim.

CAD asset (preferred):
  ~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd
  -- Converted from ANAFI Ai SIMPLIFIED STEP via omni.kit.asset_converter / HOOPS.
  -- STEP native units: millimetres.  The converter context has use_meter_as_world_unit=True
     so the output USD is already in metres, but we still measure the actual bbox and
     scale to target ~0.45 m wingspan so it looks right in the 1-m-scale warehouse.
  -- Body is an invisible lightweight physics proxy (UsdGeom.Cube, 0.001 m) at /World/Drone/Body
     with RigidBodyAPI / MassAPI / CollisionAPI attached by drone/quadrotor.add_physics().
  -- The visual mesh is a child Xform /World/Drone/Body/Visual that references the USD.
     Keeping the proxy+visual pattern means heavy mesh geometry does NOT affect
     physics inertia / collision but the drone is fully visible.

Primitive fallback:
  If the converted USD does not exist, spawn_drone() builds a recognisable quadrotor
  from scratch: central body + 4 arms + 4 rotor disc rings + a small camera gimbal.

Camera geometry:
  USD cameras look down their local -Z axis.
  To aim along world +Y (toward the rack label face at y < camera y):
    apply RotateX(+90°) on the camera Xform.
  Proof: RX(+90) * (0,0,-1) = (0, 1, 0) = +Y world.
"""
import os
from pxr import Usd, UsdGeom, UsdShade, Sdf, Gf

# ── Target visual size of the drone in the scene ─────────────────────────────
_TARGET_WINGSPAN = 0.45    # metres; ANAFI Ai is ~0.35 m wingspan
_CAD_USD = os.path.expanduser(
    "~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd"
)

# ── Minimum meaningful USD size (bytes) — below this we treat it as empty ────
_MIN_USD_BYTES = 200_000   # 200 KB


def _cad_usd_valid() -> bool:
    """Return True if the converted CAD USD exists and is non-trivial."""
    if not os.path.exists(_CAD_USD):
        return False
    sz = os.path.getsize(_CAD_USD)
    return sz >= _MIN_USD_BYTES


def _dark_material(stage, path: str, color: Gf.Vec3f, metallic: float = 0.0):
    """Helper: simple UsdPreviewSurface material."""
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/PBR")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(color)
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.4)
    mat.CreateSurfaceOutput().ConnectToSource(
        sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat


def _spawn_primitive_quadrotor(stage, body_prim_path: str, x: float, y: float, z: float):
    """Build a recognisable quadrotor from USD primitives under body_prim_path.

    Layout (Z-up, drone facing +Y):
      /Body               — flat central chassis (0.20 x 0.20 x 0.04 m) — physics proxy
      /Body/Arm_F         — +Y arm
      /Body/Arm_B         — -Y arm
      /Body/Arm_L         — -X arm
      /Body/Arm_R         — +X arm
      /Body/Rotor_FL/Disc — front-left  rotor disc (flat cylinder approximation)
      /Body/Rotor_FR/Disc — front-right rotor disc
      /Body/Rotor_RL/Disc — rear-left   rotor disc
      /Body/Rotor_RR/Disc — rear-right  rotor disc
      /Body/Gimbal        — small gray box mimicking camera gimbal under body
    """
    arm_len  = 0.18   # half-length of each arm in its long axis
    arm_r    = 0.015  # arm radius (square cross-section half-size)
    arm_z    = 0.0    # arms sit at body mid-plane
    rotor_r  = 0.07   # rotor disc radius (half-size for Scale op on a unit Cylinder)
    rotor_t  = 0.004  # rotor disc half-thickness
    rotor_off = arm_len + rotor_r * 0.5   # center of rotor disc along arm axis

    body_prim = UsdGeom.Cube.Define(stage, body_prim_path)
    body_prim.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    # chassis: 0.20 x 0.20 x 0.04 m (unit Cube has half-extent 1; scale = half-size)
    body_prim.AddScaleOp().Set(Gf.Vec3f(0.10, 0.10, 0.02))

    body_mat = _dark_material(stage, "/World/Materials/DroneBody",
                              Gf.Vec3f(0.08, 0.08, 0.08), metallic=0.5)
    UsdShade.MaterialBindingAPI(body_prim.GetPrim()).Bind(body_mat)

    arm_mat = _dark_material(stage, "/World/Materials/DroneArm",
                             Gf.Vec3f(0.15, 0.15, 0.15), metallic=0.3)
    rotor_mat = _dark_material(stage, "/World/Materials/DroneRotor",
                               Gf.Vec3f(0.18, 0.18, 0.18), metallic=0.1)

    # 4 arms: F(+Y), B(-Y), L(-X), R(+X)
    arm_defs = [
        ("Arm_F",  ( 0.0,  arm_len, arm_z), (arm_r, arm_len, arm_r)),
        ("Arm_B",  ( 0.0, -arm_len, arm_z), (arm_r, arm_len, arm_r)),
        ("Arm_L",  (-arm_len,  0.0, arm_z), (arm_len, arm_r, arm_r)),
        ("Arm_R",  ( arm_len,  0.0, arm_z), (arm_len, arm_r, arm_r)),
    ]
    for name, (ax, ay, az_arm), (sx, sy, sz_arm) in arm_defs:
        arm = UsdGeom.Cube.Define(stage, f"{body_prim_path}/{name}")
        arm.AddTranslateOp().Set(Gf.Vec3d(ax, ay, az_arm))
        arm.AddScaleOp().Set(Gf.Vec3f(sx, sy, sz_arm))
        UsdShade.MaterialBindingAPI(arm.GetPrim()).Bind(arm_mat)

    # 4 rotor discs at arm tips: FL(-X,+Y), FR(+X,+Y), RL(-X,-Y), RR(+X,-Y)
    # Use UsdGeom.Cylinder for a disc (height = rotor thickness, radius = rotor_r)
    # Cylinder default axis = Y; we want disc flat in XZ → no extra rotation needed
    # (Z-up stage: cylinder along Y axis is vertical; we want it along Z → RotateX(90))
    rotor_defs = [
        ("Rotor_FL", (-rotor_off,  rotor_off, 0.005)),
        ("Rotor_FR", ( rotor_off,  rotor_off, 0.005)),
        ("Rotor_RL", (-rotor_off, -rotor_off, 0.005)),
        ("Rotor_RR", ( rotor_off, -rotor_off, 0.005)),
    ]
    for name, (rx, ry, rz) in rotor_defs:
        disc_xf = UsdGeom.Xform.Define(stage, f"{body_prim_path}/{name}")
        disc_xf.AddTranslateOp().Set(Gf.Vec3d(rx, ry, rz))
        # Cylinder primitive: default axis=Y, radius and height are attributes
        disc = UsdGeom.Cylinder.Define(stage, f"{body_prim_path}/{name}/Disc")
        disc.CreateRadiusAttr(float(rotor_r))
        disc.CreateHeightAttr(float(rotor_t * 2))
        # Rotate so the flat disc lies in XY plane (Z-up: cylinder axis → Z)
        disc.AddRotateXOp().Set(90.0)
        UsdShade.MaterialBindingAPI(disc.GetPrim()).Bind(rotor_mat)

    # Gimbal: small box hanging under body
    gimbal_mat = _dark_material(stage, "/World/Materials/DroneGimbal",
                                Gf.Vec3f(0.3, 0.3, 0.3), metallic=0.6)
    gimbal = UsdGeom.Cube.Define(stage, f"{body_prim_path}/Gimbal")
    gimbal.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.04, -0.03))
    gimbal.AddScaleOp().Set(Gf.Vec3f(0.025, 0.025, 0.015))
    UsdShade.MaterialBindingAPI(gimbal.GetPrim()).Bind(gimbal_mat)

    return body_prim


def _spawn_cad_drone(stage, body_prim_path: str, x: float, y: float, z: float):
    """Spawn a thin physics proxy at body_prim_path + visual CAD mesh as a child.

    The physics proxy is a near-invisible 1mm cube (mass is set by add_physics()
    externally, so physical size does not matter for dynamics).  The visual mesh
    is a child Xform+Reference that references the converted USD, scaled and
    recentered so the drone's ~0.45 m wingspan sits level at (x,y,z).

    Returns the body_prim UsdGeom.Cube.
    """
    # ── Physics proxy: tiny invisible cube ───────────────────────────────────
    body_prim = UsdGeom.Cube.Define(stage, body_prim_path)
    body_prim.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    body_prim.AddScaleOp().Set(Gf.Vec3f(0.001, 0.001, 0.001))
    # Make it essentially invisible (opacity ~ 0) via a transparent material
    invis_mat = UsdShade.Material.Define(stage, "/World/Materials/DroneProxy")
    invis_sh = UsdShade.Shader.Define(stage, "/World/Materials/DroneProxy/PBR")
    invis_sh.CreateIdAttr("UsdPreviewSurface")
    invis_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.1, 0.1, 0.1))
    invis_sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.0)
    invis_mat.CreateSurfaceOutput().ConnectToSource(
        invis_sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    UsdShade.MaterialBindingAPI(body_prim.GetPrim()).Bind(invis_mat)

    # ── Visual mesh child: Xform + Reference to CAD USD ──────────────────────
    visual_xf = UsdGeom.Xform.Define(stage, f"{body_prim_path}/Visual")
    mesh_prim = stage.DefinePrim(f"{body_prim_path}/Visual/Mesh")
    mesh_prim.GetReferences().AddReference(_CAD_USD)

    # Measure the native bounding box of the referenced asset
    # We must temporarily commit the stage so the reference is resolvable
    cache = UsdGeom.BBoxCache(
        Usd.TimeCode.Default(),
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
        useExtentsHint=True,
    )
    rng = cache.ComputeWorldBound(visual_xf.GetPrim()).ComputeAlignedRange()
    mn, mx = rng.GetMin(), rng.GetMax()
    native_size = (mx[0] - mn[0], mx[1] - mn[1], mx[2] - mn[2])
    native_ctr  = ((mx[0] + mn[0]) / 2.0, (mx[1] + mn[1]) / 2.0, (mx[2] + mn[2]) / 2.0)

    if max(native_size) < 1e-6:
        # Fallback: bbox unavailable (payload unloaded headless)
        # Use a conservative scale: STEP is in mm natively; if converter did NOT
        # apply use_meter_as_world_unit, native extents will be ~350 mm wide.
        # Apply 1/1000 scale and re-center empirically.
        print("[drone_asset] WARNING: CAD USD bbox is zero; using mm→m scale (1/1000)")
        s = 1.0 / 1000.0 * (_TARGET_WINGSPAN / 0.35)  # assume 350mm wingspan
        visual_xf.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
        visual_xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))
    else:
        max_dim = max(native_size)
        s = _TARGET_WINGSPAN / max_dim
        tx = x - native_ctr[0] * s
        ty = y - native_ctr[1] * s
        tz = z - native_ctr[2] * s
        visual_xf.AddTranslateOp().Set(Gf.Vec3d(tx, ty, tz))
        visual_xf.AddScaleOp().Set(Gf.Vec3f(s, s, s))
        print(f"[drone_asset] CAD drone: native_size={native_size[0]:.4f}x{native_size[1]:.4f}x{native_size[2]:.4f}m "
              f"scale={s:.5f} final_wingspan~{max_dim*s:.3f}m")

    return body_prim


# ── Public API ────────────────────────────────────────────────────────────────

def spawn_drone(stage, position, look_dir_deg_about_x=90.0, prim_path="/World/Drone"):
    """Create a drone body (CAD USD or primitive quadrotor) and a Camera prim.

    The body prim at `prim_path + "/Body"` is always a lightweight physics proxy
    (plain UsdGeom.Cube) so that drone/quadrotor.add_physics() and RigidPrim can
    target it without issues.  The visual geometry lives one level deeper:
      - CAD mode  : /World/Drone/Body/Visual/Mesh (AddReference to anafi_ai.usd)
      - Primitive : inline Arms/Rotors/Gimbal children of /World/Drone/Body

    Args:
        stage: USD stage to add prims to.
        position: (x, y, z) world position for the drone body centre.
        look_dir_deg_about_x: Rotation around X axis (degrees) applied to the
            camera Xform.  Default +90° maps local -Z to world +Y so the
            camera faces the rack label (label face normal is +Y).
        prim_path: Root prim path for the drone (default "/World/Drone").

    Returns:
        str: Prim path of the Camera prim.
    """
    x, y, z = position
    body_path = prim_path + "/Body"

    use_cad = _cad_usd_valid()
    if use_cad:
        print(f"[drone_asset] Using real CAD drone: {_CAD_USD} ({os.path.getsize(_CAD_USD)/1e6:.1f} MB)")
        _spawn_cad_drone(stage, body_path, x, y, z)
    else:
        print(f"[drone_asset] CAD USD not found or too small ({_CAD_USD}); using primitive quadrotor fallback.")
        _spawn_primitive_quadrotor(stage, body_path, x, y, z)

    # Camera prim — always at same position/orientation regardless of visual mode
    # Placed slightly forward+below the body centre as a cosmetic camera mount
    cam_offset_y =  0.05   # slightly forward of body centre
    cam_offset_z = -0.03   # slightly below body centre
    cam = UsdGeom.Camera.Define(stage, prim_path + "/Camera")
    cam.AddTranslateOp().Set(Gf.Vec3d(x, y + cam_offset_y, z + cam_offset_z))
    cam.AddRotateXOp().Set(look_dir_deg_about_x)

    return prim_path + "/Camera"
