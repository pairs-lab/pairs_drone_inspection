"""live_sim.py — Isaac Sim 6.0 WebSocket streaming demo.

Builds the full warehouse POC scene (warehouse.usd backdrop + two racks +
aisle obstacle + ANAFI drone) and streams the drone's onboard camera as live
JPEG frames + telemetry over WebSocket (port 8765 by default).

Also mounts an RTX LiDAR (SLAMTEC RPLIDAR_S2E config) on the drone and
streams downsampled point cloud data at ~3-5 Hz.

Usage (from repo root):
    # GUI mode (default):
    scripts/run_isaac.sh scripts/live_sim.py

    # Headless mode:
    scripts/run_isaac.sh scripts/live_sim.py -- --headless

    # Custom WS port:
    scripts/run_isaac.sh scripts/live_sim.py -- --headless --ws-port 8765

    # Finite duration for testing (default=0 = run until Ctrl-C/SIGTERM):
    scripts/run_isaac.sh scripts/live_sim.py -- --headless --duration 90

WS server (port 8765):
    Server→client JSON:
      {"type":"frame","jpeg":"<base64>","w":960,"h":540,"ts":<float>}
      {"type":"telemetry","pos":[x,y,z],"target":<bin|null>,
       "state":"IDLE|FLYING|SCANNING|RETURNING",
       "clearance":<float>,"lidar_min":<float>,
       "detections":[{"part_no":...,"qty":...,"bbox":[x1,y1,x2,y2]}]}
      {"type":"inspection","bin_id":..., "scanned_part":..., "scanned_qty":...,
       "system_part":..., "system_qty":..., "match":<bool>,
       "status":"completed"|"discrepancy","ts":<float>}
      {"type":"lidar","points":[[x,y,z],...],"frame":"world","n":<count>}

    Client→server JSON:
      {"type":"cmd","action":"inspect","bin_id":"B3"}
      {"type":"cmd","action":"home"}

The script also keeps an optional UDP/ffmpeg path (--udp) for backwards compat.

Prereq: ~/isaacsim/python.sh -m pip install websockets
"""

import argparse
import asyncio
import base64
import json
import math
import os
import queue
import subprocess
import sys
import threading
import time

# ---------------------------------------------------------------------------
# CLI flags
# ---------------------------------------------------------------------------
_argv = sys.argv[1:]
if "--" in _argv:
    _argv = _argv[_argv.index("--") + 1:]

ap = argparse.ArgumentParser()
ap.add_argument("--headless", action="store_true")
ap.add_argument("--duration", type=float, default=0.0,
                help="Stop after N seconds (0 = run forever)")
ap.add_argument("--width",  type=int, default=1280)
ap.add_argument("--height", type=int, default=720)
ap.add_argument("--fps",    type=int, default=20,
                help="Target simulation frame rate")
ap.add_argument("--ws-port", type=int, default=8765,
                help="WebSocket server port")
ap.add_argument("--no-warehouse", dest="warehouse", action="store_false",
                default=True)
ap.add_argument("--udp", action="store_true",
                help="Also pipe frames to ffmpeg UDP (legacy, optional)")
args = ap.parse_args(_argv)

# ---------------------------------------------------------------------------
# 1. Boot Isaac Sim
# ---------------------------------------------------------------------------
from isaacsim import SimulationApp  # noqa: E402

sim_cfg = {
    "headless": args.headless,
    "active_gpu": 0,
    "physics_gpu": 0,
    "width":  args.width,
    "height": args.height,
}
simulation_app = SimulationApp(sim_cfg)

import carb
_s = carb.settings.get_settings()
_s.set("/renderer/activeGpu", 0)
_s.set("/rtx/materialDb/syncLoads", True)
_s.set("/rtx/hydra/materialSyncLoads", True)
_s.set("/omni.kit.plugin/syncUsdLoads", True)

import numpy as np
import omni.usd
import omni.replicator.core as rep
from pxr import Sdf, UsdGeom, UsdLux, Gf

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    _PYZBAR_OK = True
except ImportError:
    _PYZBAR_OK = False
    print("[live_sim] WARNING: pyzbar not available — no QR overlay")

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False
    print("[live_sim] WARNING: cv2 not available — no overlay drawing")

try:
    import websockets
    _WS_OK = True
except ImportError:
    _WS_OK = False
    print("[live_sim] WARNING: websockets not available — install with: "
          "~/isaacsim/python.sh -m pip install websockets")

from sim.config import (HOME_POSE, RACK_WORLD_OFFSET,
                        BOX_D, BOX_H, SCAN_STANDOFF,
                        COLUMNS, LEVELS, COLUMN_SPACING, LEVEL_HEIGHT,
                        GROUND_DRONE_Z)

# ---------------------------------------------------------------------------
# 2. Seed SAP mock
# ---------------------------------------------------------------------------
try:
    from backend.sap_mock import seed_from_bin_map, get_inventory
    seed_from_bin_map()
    print("[live_sim] SAP mock seeded")
except Exception as _e:
    print(f"[live_sim] WARNING: SAP mock seed failed: {_e}")
    def get_inventory(bin_id):  # noqa: E302
        return None

# ---------------------------------------------------------------------------
# 3. Build the scene
# ---------------------------------------------------------------------------
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
UsdGeom.Xform.Define(stage, "/World")

if args.warehouse:
    try:
        from sim.warehouse import use_local_assets, build_warehouse_env
        use_local_assets()
        build_warehouse_env(stage, kind="warehouse")
        print("[live_sim] warehouse.usd backdrop loaded")
    except (AssertionError, Exception) as _e:
        print(f"[live_sim] warehouse.usd not available ({_e}), using primitive ground")
        from sim.warehouse import build_warehouse
        build_warehouse(stage)
else:
    from sim.warehouse import build_warehouse
    build_warehouse(stage)

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1500.0)

from sim.rack import build_rack, build_second_rack, build_aisle_obstacle
print("[live_sim] building primary rack (18 bins)...")
n_bins = build_rack(stage)
print(f"[live_sim] primary rack: {n_bins} bins")

print("[live_sim] building second (mirror) rack + aisle obstacle...")
build_second_rack(stage)
build_aisle_obstacle(stage)

ox, oy, oz = RACK_WORLD_OFFSET
hp = HOME_POSE["position"]
drone_pos = (hp[0] + ox, hp[1] + oy, hp[2] + oz)
from sim.drone_asset import spawn_drone
spawn_drone(stage, drone_pos)
print(f"[live_sim] drone spawned at {drone_pos}")

# ---------------------------------------------------------------------------
# Home landing pad — a flat marker on the floor where the drone parks/lands.
# ---------------------------------------------------------------------------
try:
    from pxr import UsdShade as _UsdShade
    _pad_path = "/World/HomePad"
    _pad = UsdGeom.Cylinder.Define(stage, _pad_path)
    _pad.CreateRadiusAttr(0.45)
    _pad.CreateHeightAttr(0.02)
    _pad.AddTranslateOp().Set(Gf.Vec3d(drone_pos[0], drone_pos[1], 0.011))
    _pad_mat = _UsdShade.Material.Define(stage, "/World/Materials/HomePad")
    _pad_sh = _UsdShade.Shader.Define(stage, "/World/Materials/HomePad/PBR")
    _pad_sh.CreateIdAttr("UsdPreviewSurface")
    _pad_sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.05, 0.35, 0.12))
    _pad_sh.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.0, 0.55, 0.18))
    _pad_sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.6)
    _pad_mat.CreateSurfaceOutput().ConnectToSource(
        _pad_sh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    _UsdShade.MaterialBindingAPI(_pad.GetPrim()).Bind(_pad_mat)
    print(f"[live_sim] home landing pad at ({drone_pos[0]:.1f},{drone_pos[1]:.1f},0)")
except Exception as _pad_e:
    print(f"[live_sim] home pad skipped: {_pad_e}")

# ---------------------------------------------------------------------------
# Drone beacon — a bright emissive sphere co-located with the drone so it is
# easy to spot in the Isaac viewport (repositioned every frame in the main loop).
# ---------------------------------------------------------------------------
# Beacon disabled — the orange sphere covered the drone. The centerline chase-cam
# shows the drone clearly without it.
_beacon_t_op = None

# ---------------------------------------------------------------------------
# Spinning rotor discs — the ANAFI CAD mesh has static props, so we overlay 4
# thin discs at the rotor positions and spin them every frame (repositioned with
# the drone in the main loop). Gives a visible "props spinning" effect.
# ---------------------------------------------------------------------------
_rotor_group_t_op = None     # translate op of the /World/DroneRotors group
_rotor_spin_ops = []         # RotateZ op of each disc (incremented each frame)
try:
    from pxr import UsdShade as _UsdShade3
    _grp = UsdGeom.Xform.Define(stage, "/World/DroneRotors")
    _rotor_group_t_op = _grp.AddTranslateOp()
    _rotor_group_t_op.Set(Gf.Vec3d(drone_pos[0], drone_pos[1], drone_pos[2] + 0.06))
    _rmat = _UsdShade3.Material.Define(stage, "/World/Materials/Rotor")
    _rsh = _UsdShade3.Shader.Define(stage, "/World/Materials/Rotor/PBR")
    _rsh.CreateIdAttr("UsdPreviewSurface")
    _rsh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.15, 0.15, 0.15))
    _rsh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(0.45)   # blur-like
    _rmat.CreateSurfaceOutput().ConnectToSource(_rsh.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    # ANAFI propeller positions derived from the mesh bbox (width ±0.14 X, length
    # ±0.22 Y): props at the 4 arm tips. NOT a symmetric square (longer front-back).
    # Tunable if slightly off.
    # _ROTOR_OFFSETS = [
    #     (0.10,  0.02),   # front-left  (front pair lowered per feedback)
    #     ( 0.10,  0.02),   # front-right
    #     (0.10, -0.05),   # rear-left
    #     ( 0.10, -0.05),   # rear-right
    # ]
    # for _i, (_ox, _oy) in enumerate(_ROTOR_OFFSETS):
    #     _disc = UsdGeom.Cylinder.Define(stage, f"/World/DroneRotors/Rotor_{_i}")
    #     _disc.CreateRadiusAttr(0.06)
    #     _disc.CreateHeightAttr(0.006)
    #     _disc.AddTranslateOp().Set(Gf.Vec3d(_ox, _oy, 0.0))   # offset within group
    #     _spin = _disc.AddRotateZOp()                          # spin op (updated in loop)
    #     _spin.Set(0.0)
    #     _rotor_spin_ops.append(_spin)
    #     _UsdShade3.MaterialBindingAPI(_disc.GetPrim()).Bind(_rmat)
    # print("[live_sim] 4 spinning rotor discs created")
except Exception as _rot_e:
    print(f"[live_sim] rotor discs skipped: {_rot_e}")
    _rotor_group_t_op = None
    _rotor_spin_ops = []
_rotor_angle = 0.0

# ---------------------------------------------------------------------------
# 3a. Set the Isaac Sim GUI viewport to look at the parked drone on startup
# ---------------------------------------------------------------------------
# When run with a window (not --headless), place the persp viewport camera just
# behind/above the drone's HOME park spot, framing the drone + the loaded rack —
# so opening Isaac Sim shows the robot where it is parked.
_set_camera_view = None   # saved so the main loop can chase-cam the drone
try:
    from isaacsim.core.utils.viewports import set_camera_view as _set_camera_view
    _dx, _dy, _dz = drone_pos
    # Wide overview from behind/above the parked drone, framing the WHOLE aisle +
    # both racks — so you see the drone parked at the aisle mouth AND watch it fly
    # down to a box when you press Check (it stays in view the whole flight).
    _set_camera_view(
        eye=[_dx - 3.0, _dy - 4.5, _dz + 3.0],     # back + to the side + above
        target=[1.0, -0.9, 1.0],                    # rack/aisle center
        camera_prim_path="/OmniverseKit_Persp",
    )
    print(f"[live_sim] GUI viewport set to wide aisle overview (drone home={drone_pos})")
except Exception as _vp_e:
    print(f"[live_sim] viewport camera set skipped: {_vp_e}")

# ---------------------------------------------------------------------------
# 3b. RTX LiDAR — mounted on drone body using the exact replicator annotator API
# ---------------------------------------------------------------------------
# API reference:
#   docs.isaacsim.omniverse.nvidia.com/4.5.0/sensors/isaacsim_sensors_rtx_lidar.html
# Strategy (Isaac Sim 6.0 binary on this machine):
#   1. Try IsaacSensorCreateRtxLidar command with config "Example_Rotary".
#      - _add_reference() tries get_assets_root_path()+config path → raises if missing.
#      - We patch get_assets_root_path to return None so _add_reference returns None,
#        allowing _call_replicator_api() to run instead (creates OmniLidar prim).
#   2. Fallback: rep.create.omni_lidar directly (OmniLidar prim, no USD asset needed).
#   3. Annotator: try "RtxSensorCpuIsaacCreateRTXLidarScanBuffer" first (4.5 docs),
#      then "IsaacCreateRTXLidarScanBuffer" (confirmed present in 6.0), then
#      "GenericModelOutput" (new recommended).
# annotator.get_data() returns a dict; keys inspected + printed at runtime.
# ---------------------------------------------------------------------------
import omni.kit.commands as _omni_kit_cmds  # noqa: E402

_lidar_annot = None      # rep.Annotator (or None if unavailable)
_lidar_rp    = None      # rep RenderProduct (kept alive to prevent GC)
_lidar_keys_printed = True  # default: skip key print

try:
    # Parent to /World/Drone (Xform) — /World/Drone/Body is a Cube (gprim) so
    # nested OmniLidar prims can't be children of it.
    # force_camera_prim approach can create children of any prim but produces a
    # non-functional sensor on Isaac 6.0; rep.functional.create.omni_lidar is
    # the recommended path and requires an Xform parent.
    _LIDAR_PATH  = "/World/Drone/Lidar"
    _PARENT_PATH = "/World/Drone"
    _lidar_config = "Example_Rotary"  # built-in rotary config

    _sensor_prim_path = None

    # --- Strategy 1: IsaacSensorCreateRtxLidar with the user-provided exact API ---
    # This uses rep.functional.create.omni_lidar internally (_call_replicator_api).
    # The _add_reference() path fails if the lidar USD isn't in local assets.
    # We detect this at import time and skip to strategy 2 if the asset is missing.
    _lidar_usd_exists = False
    try:
        import isaacsim.storage.native as _isn
        _assets_root = _isn.get_assets_root_path()
        import os as _os
        _lidar_usd_exists = (
            _assets_root is not None and
            _os.path.exists(str(_assets_root) + "/Isaac/Sensors/NVIDIA/Example_Rotary.usda")
        )
    except Exception:
        _lidar_usd_exists = False

    if _lidar_usd_exists:
        try:
            _, _lsensor = _omni_kit_cmds.execute(
                "IsaacSensorCreateRtxLidar",
                path="/Lidar",
                parent=_PARENT_PATH,
                config=_lidar_config,
                translation=(0, 0, 0.0),
                orientation=Gf.Quatd(1, 0, 0, 0),
            )
            if _lsensor is not None and _lsensor.IsValid():
                _sensor_prim_path = str(_lsensor.GetPath())
                print(f"[live_sim] IsaacSensorCreateRtxLidar OK → {_sensor_prim_path}")
        except Exception as _strat1_e:
            print(f"[live_sim] IsaacSensorCreateRtxLidar failed ({_strat1_e.__class__.__name__})")
    else:
        print(f"[live_sim] Example_Rotary.usda not in local assets → using rep.functional.create.omni_lidar")

    # --- Strategy 2: rep.functional.create.omni_lidar (OmniLidar prim, no USD needed) ---
    if _sensor_prim_path is None:
        try:
            _lidar_rep_item = rep.functional.create.omni_lidar(
                position=(0, 0, 0),
                rotation=(0, 0, 0),
                name="Lidar",
                parent=_PARENT_PATH,
            )
            # rep.functional.create returns a list of prims
            if isinstance(_lidar_rep_item, (list, tuple)) and len(_lidar_rep_item) > 0:
                _lsensor = _lidar_rep_item[0]
            else:
                _lsensor = _lidar_rep_item
            if hasattr(_lsensor, "GetPath"):
                _sensor_prim_path = str(_lsensor.GetPath())
            elif hasattr(_lsensor, "GetPrimPath"):
                _sensor_prim_path = str(_lsensor.GetPrimPath())
            else:
                # last resort: navigate stage to find OmniLidar under parent
                _sp = stage.GetPrimAtPath(_PARENT_PATH + "/Lidar")
                if _sp.IsValid():
                    _sensor_prim_path = _PARENT_PATH + "/Lidar"
                    print("[live_sim] found OmniLidar at path fallback")
            print(f"[live_sim] rep.functional.create.omni_lidar OK → {_sensor_prim_path}")
        except Exception as _strat2_e:
            print(f"[live_sim] rep.functional.create.omni_lidar failed: {_strat2_e}")

    # --- Strategy 3: IsaacSensorCreateRtxLidar with force_camera_prim ---
    # (deprecated camera prim approach - lidar sensor model won't work but annotator attaches)
    if _sensor_prim_path is None:
        try:
            _, _lsensor = _omni_kit_cmds.execute(
                "IsaacSensorCreateRtxLidar",
                path="/Lidar",
                parent=_PARENT_PATH,
                config=_lidar_config,
                translation=(0, 0, 0.0),
                orientation=Gf.Quatd(1, 0, 0, 0),
                force_camera_prim=True,
            )
            if _lsensor is not None:
                _sensor_prim_path = str(_lsensor.GetPath())
                print(f"[live_sim] IsaacSensorCreateRtxLidar force_camera_prim → {_sensor_prim_path}")
        except Exception as _strat3_e:
            print(f"[live_sim] force_camera_prim fallback failed: {_strat3_e}")

    # --- Attach annotator to render product ---
    if _sensor_prim_path is not None:
        _lidar_rp = rep.create.render_product(_sensor_prim_path, [1, 1])
        _lidar_annot_name = None
        for _aname in [
            "RtxSensorCpuIsaacCreateRTXLidarScanBuffer",  # 4.5 docs name
            "IsaacCreateRTXLidarScanBuffer",               # 6.0 confirmed present
            "GenericModelOutput",                          # newer recommended
            "RtxSensorCpu",                               # generic fallback
        ]:
            try:
                _lidar_annot = rep.AnnotatorRegistry.get_annotator(_aname)
                _lidar_annot_name = _aname
                break
            except Exception:
                _lidar_annot = None
        if _lidar_annot is not None:
            try:
                _lidar_annot.attach([_lidar_rp])
            except Exception:
                _lidar_annot.attach(_lidar_rp)
            print(f"[live_sim] LiDAR annotator attached: {_lidar_annot_name} @ {_sensor_prim_path}")
            _lidar_keys_printed = False  # will print get_data() keys once in main loop
        else:
            print("[live_sim] WARNING: no supported LiDAR annotator found — lidar disabled")
    else:
        print("[live_sim] WARNING: LiDAR sensor prim could not be created — lidar disabled")

except Exception as _lidar_init_e:
    import traceback as _tb
    print(f"[live_sim] WARNING: RTX LiDAR init error: {_lidar_init_e}")
    _tb.print_exc()
    _lidar_annot = None
    _lidar_keys_printed = True

# ---------------------------------------------------------------------------
# 4. Camera setup — use rep.create.camera() (same API as verify_capture)
# ---------------------------------------------------------------------------
# Root-cause fix: a raw UsdGeom.Camera prim attached via rep.create.render_product
# does NOT render emissive label materials correctly in Isaac 6.0 RTX — the label
# quad texture stays invisible on the box face.  rep.create.camera() creates a
# Replicator-managed camera prim (/Replicator/Camera_Xform/Camera) that IS
# correctly registered with the RTX material pipeline and reliably decodes QR.
# We replicate verify_capture's exact setup (focal=10, aperture=20.955,
# clipping near=0.01, look_at_up_axis=+Z) and update position each frame using
# rep.modify.pose() via a replicator graph.
# ---------------------------------------------------------------------------
W, H = args.width, args.height
_start_cam_pos = (0.0, -(BOX_D / 2 + SCAN_STANDOFF), BOX_H / 2)
_start_look_at = (0.0, -(BOX_D / 2) - 0.005, BOX_H / 2)

# Create the Replicator camera at the initial A1 scan pose
_rep_cam = rep.create.camera(
    position=_start_cam_pos,
    look_at=_start_look_at,
    look_at_up_axis=(0.0, 0.0, 1.0),
    name="LiveStreamCam",
)
print(f"[live_sim] rep.create.camera at {_start_cam_pos} looking at {_start_look_at}")

# Set near clip, focal length, aperture on the underlying USD camera prim
# (rep.create.camera may not expose these directly as kwargs in Isaac 6.0)
for _p in stage.Traverse():
    if _p.IsA(UsdGeom.Camera):
        _pp = _p.GetPath().pathString
        if "LiveStreamCam" in _pp or "Camera_Xform" in _pp:
            _usd_c = UsdGeom.Camera(_p)
            _usd_c.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))
            _usd_c.GetFocalLengthAttr().Set(10.0)
            _usd_c.GetHorizontalApertureAttr().Set(20.955)
            print(f"[live_sim] set clipping/focal on camera prim: {_pp}")

# Also set on any /Replicator camera prims (verify_capture pattern)
for _p in stage.Traverse():
    if _p.GetPath().pathString.startswith("/Replicator") and _p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(_p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1_000_000.0))

_rp = rep.create.render_product(_rep_cam, (W, H), name="LiveStream")
_annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
_annot.attach([_rp])


# Remember the last camera pose so we can skip redundant updates (see below).
_last_cam_pose = None


def _update_cam_xform(cam_pos: tuple, look_at: tuple):
    """Move the replicator camera to cam_pos looking at look_at (up=+Z).

    Uses rep.modify.pose() — the API that keeps the RTX material pipeline
    registered so the emissive QR labels render (a direct USD xform write is
    faster but leaves the labels black, so scans fail to decode).

    PERF: rep.modify.pose() accumulates OmniGraph state, so calling it EVERY
    frame makes its cost grow without bound (profiled 32 ms -> 260 ms+), which is
    what made the stream fps decay over time. The camera is stationary while IDLE
    (parked at home) and only moves during FLYING/SCANNING/RETURNING, so we skip
    the call when the pose is unchanged. That removes the per-frame cost during the
    long IDLE periods (no decay) while still posing correctly whenever it moves.
    """
    global _last_cam_pose
    pose = (round(float(cam_pos[0]), 4), round(float(cam_pos[1]), 4), round(float(cam_pos[2]), 4),
            round(float(look_at[0]), 4), round(float(look_at[1]), 4), round(float(look_at[2]), 4))
    # Skip redundant calls (the fps fix) — EXCEPT while SCANNING. During the scan
    # hold the pose is constant, but the repeated rep.modify.pose is what keeps the
    # emissive QR label rendered so pyzbar can decode it, so we must keep calling it
    # there. SCANNING is brief, so the bounded accumulation is harmless.
    if pose == _last_cam_pose and _drone_state != "SCANNING":
        return
    _last_cam_pose = pose
    try:
        with _rep_cam:
            rep.modify.pose(
                position=cam_pos,
                look_at=look_at,
                look_at_up_axis=(0.0, 0.0, 1.0),
            )
    except Exception as _cam_e:
        # Fallback: directly set the translate op on the underlying USD prim.
        for _p in stage.Traverse():
            if _p.IsA(UsdGeom.Camera):
                _pp = _p.GetPath().pathString
                if "LiveStreamCam" in _pp or ("Camera_Xform" in _pp and "Replicator" in _pp):
                    _xf = UsdGeom.Xformable(_p)
                    _t_op = next((o for o in _xf.GetOrderedXformOps()
                                  if o.GetOpType() == UsdGeom.XformOp.TypeTranslate), None)
                    if _t_op is not None:
                        _t_op.Set(Gf.Vec3d(float(cam_pos[0]), float(cam_pos[1]), float(cam_pos[2])))
                    break


_update_cam_xform(_start_cam_pos, _start_look_at)

# ---------------------------------------------------------------------------
# 5. Warmup
# ---------------------------------------------------------------------------
print("[live_sim] RTX warmup (shader compilation, ~2-5 min first run)...")
_warmup_start = time.time()
for i in range(120):
    simulation_app.update()
    if i % 20 == 0:
        print(f"[live_sim] warmup frame {i}/120 ({time.time()-_warmup_start:.0f}s)")

for _ in range(10):
    rep.orchestrator.step(rt_subframes=8, wait_for_render=True)
print(f"[live_sim] warmup done in {time.time()-_warmup_start:.1f}s")

# ---------------------------------------------------------------------------
# 6. Optional legacy UDP/ffmpeg
# ---------------------------------------------------------------------------
_ffmpeg_proc = None
_udp_url = "udp://127.0.0.1:5600?pkt_size=1316"
_encoder = None

if args.udp:
    _ffmpeg_nvenc = [
        "ffmpeg", "-loglevel", "warning",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{W}x{H}", "-r", str(args.fps), "-i", "-",
        "-c:v", "h264_nvenc", "-preset", "p1", "-tune", "ll",
        "-b:v", "6M", "-g", str(args.fps), "-f", "mpegts", _udp_url,
    ]
    _ffmpeg_x264 = [
        "ffmpeg", "-loglevel", "warning",
        "-f", "rawvideo", "-pix_fmt", "rgb24",
        "-s", f"{W}x{H}", "-r", str(args.fps), "-i", "-",
        "-c:v", "libx264", "-preset", "ultrafast", "-tune", "zerolatency",
        "-b:v", "4M", "-g", str(args.fps), "-f", "mpegts", _udp_url,
    ]
    print("[live_sim] starting ffmpeg subprocess (nvenc preferred)...")
    _encoder = "h264_nvenc"
    try:
        _ffmpeg_proc = subprocess.Popen(_ffmpeg_nvenc, stdin=subprocess.PIPE,
                                        stderr=subprocess.PIPE)
        time.sleep(1.5)
        if _ffmpeg_proc.poll() is not None:
            _err = _ffmpeg_proc.stderr.read().decode(errors="replace")
            print(f"[live_sim] nvenc failed: {_err[:300]}")
            raise RuntimeError("nvenc failed")
        print("[live_sim] ffmpeg started with h264_nvenc")
    except Exception as _nvenc_e:
        print(f"[live_sim] falling back to libx264: {_nvenc_e}")
        _encoder = "libx264"
        _ffmpeg_proc = subprocess.Popen(_ffmpeg_x264, stdin=subprocess.PIPE,
                                        stderr=subprocess.PIPE)
        time.sleep(0.5)
        if _ffmpeg_proc.poll() is not None:
            _err = _ffmpeg_proc.stderr.read().decode(errors="replace")
            raise RuntimeError(f"libx264 also failed: {_err[:400]}")
        print("[live_sim] ffmpeg started with libx264")

# ---------------------------------------------------------------------------
# 7. Shared state — thread-safe structures between main loop and WS thread
# ---------------------------------------------------------------------------

# Latest JPEG frame: bytes or None
_latest_frame_lock = threading.Lock()
_latest_frame: dict = {"jpeg": None, "w": 960, "h": 540, "ts": 0.0}

# Latest telemetry snapshot
_telem_lock = threading.Lock()
_telem: dict = {
    "pos": list(drone_pos),
    "target": None,
    "state": "IDLE",
    "clearance": 9.9,
    "detections": [],
}

# Incoming command queue (main loop consumes)
_cmd_queue: queue.Queue = queue.Queue()

# Latest inspection result (for broadcast once set)
_inspection_lock = threading.Lock()
_pending_inspection: dict = {}   # set by main loop; WS thread broadcasts once

# Latest LiDAR point cloud snapshot
_lidar_lock = threading.Lock()
_latest_lidar: dict = {"points": None, "n": 0, "ts": 0.0}  # points: list of [x,y,z]

# Set of active WS client queues: each client has its own asyncio Queue
# accessed from the WS asyncio thread only
_ws_clients_lock = threading.Lock()
_ws_clients: set = set()

# ---------------------------------------------------------------------------
# 8. WebSocket server (runs in a separate thread with its own event loop)
# ---------------------------------------------------------------------------

def _ws_thread_main(port: int):
    """Entry point for the WebSocket server thread."""
    if not _WS_OK:
        print("[live_sim/ws] websockets not installed — WS server disabled")
        return

    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(_ws_serve(port))


async def _ws_serve(port: int):
    """Async WS server: accept clients, broadcast frames, handle commands."""
    import websockets.server as _wss

    print(f"[live_sim/ws] starting WebSocket server on 0.0.0.0:{port}")

    # Per-client queues stored in a set so broadcaster can push to all
    _client_queues: set = set()

    async def _handle_client(ws):
        q = asyncio.Queue(maxsize=4)  # small buffer; drop oldest on overflow
        _client_queues.add(q)
        print(f"[live_sim/ws] client connected: {ws.remote_address}")
        try:
            consumer_task = asyncio.ensure_future(_consumer(ws))
            producer_task = asyncio.ensure_future(_producer(ws, q))
            done, pending = await asyncio.wait(
                [consumer_task, producer_task],
                return_when=asyncio.FIRST_COMPLETED,
            )
            for t in pending:
                t.cancel()
        except Exception as _ex:
            print(f"[live_sim/ws] client error: {_ex}")
        finally:
            _client_queues.discard(q)
            print(f"[live_sim/ws] client disconnected: {ws.remote_address}")

    async def _consumer(ws):
        """Receive commands from a single client."""
        async for msg in ws:
            try:
                obj = json.loads(msg)
                if obj.get("type") == "cmd":
                    print(f"[live_sim/ws] command received: {obj}")
                    _cmd_queue.put_nowait(obj)
            except Exception as _ex:
                print(f"[live_sim/ws] bad message: {_ex}")

    async def _producer(ws, q: asyncio.Queue):
        """Send queued messages to a single client."""
        while True:
            msg = await q.get()
            try:
                await ws.send(msg)
            except Exception:
                break

    async def _broadcast_loop():
        """Poll shared state and push to all client queues."""
        last_frame_ts = 0.0
        last_telem_ts = 0.0
        last_inspection_id = None
        last_lidar_ts = 0.0
        FRAME_INTERVAL = 1.0 / 10   # ~10 fps broadcast
        TELEM_INTERVAL = 1.0 / 5    #  ~5 Hz
        LIDAR_INTERVAL = 1.0 / 4    #  ~4 Hz (lidar is heavier)

        while True:
            now = time.time()

            # --- frame ---
            if now - last_frame_ts >= FRAME_INTERVAL:
                with _latest_frame_lock:
                    snap = dict(_latest_frame)
                if snap["jpeg"] is not None and snap["ts"] > last_frame_ts:
                    last_frame_ts = snap["ts"]
                    msg = json.dumps({
                        "type": "frame",
                        "jpeg": snap["jpeg"],
                        "w": snap["w"],
                        "h": snap["h"],
                        "ts": snap["ts"],
                    })
                    _push_to_all(_client_queues, msg)

            # --- telemetry ---
            if now - last_telem_ts >= TELEM_INTERVAL:
                last_telem_ts = now
                with _telem_lock:
                    t = dict(_telem)
                msg = json.dumps({"type": "telemetry", **t})
                _push_to_all(_client_queues, msg)

            # --- pending inspection result ---
            with _inspection_lock:
                insp = dict(_pending_inspection)
            if insp and insp.get("_id") != last_inspection_id:
                last_inspection_id = insp.get("_id")
                payload = {k: v for k, v in insp.items() if k != "_id"}
                msg = json.dumps({"type": "inspection", **payload})
                _push_to_all(_client_queues, msg)
                print(f"[live_sim/ws] broadcast inspection: {payload}")

            # --- lidar point cloud ---
            if now - last_lidar_ts >= LIDAR_INTERVAL:
                with _lidar_lock:
                    lsnap = dict(_latest_lidar)
                if lsnap["points"] is not None and lsnap["ts"] > last_lidar_ts:
                    last_lidar_ts = lsnap["ts"]
                    msg = json.dumps({
                        "type": "lidar",
                        "points": lsnap["points"],
                        "frame": "world",
                        "n": lsnap["n"],
                    })
                    _push_to_all(_client_queues, msg)

            await asyncio.sleep(0.02)  # 50 Hz poll

    def _push_to_all(queues: set, msg: str):
        for q in list(queues):
            try:
                q.put_nowait(msg)
            except asyncio.QueueFull:
                # Drop oldest, insert newest
                try:
                    q.get_nowait()
                    q.put_nowait(msg)
                except Exception:
                    pass

    async with _wss.serve(_handle_client, "0.0.0.0", port):
        print(f"LIVE_SIM ws://0.0.0.0:{port}  (connect: ws://localhost:{port})")
        await _broadcast_loop()


# Start the WS thread
_ws_thread = threading.Thread(target=_ws_thread_main, args=(args.ws_port,),
                               daemon=True, name="ws-server")
_ws_thread.start()

# ---------------------------------------------------------------------------
# 9. Drone state machine for commanded inspections
# ---------------------------------------------------------------------------

from drone.waypoints import plan_waypoints

# State: IDLE | FLYING | SCANNING | RETURNING
_drone_state = "IDLE"
_drone_target: str = None   # current commanded bin_id
_drone_pos = list(drone_pos)  # [x, y, z] — updated every frame during motion

# Parked "home" camera pose: the drone hovers at HOME and looks toward the rack.
# Used while IDLE (drone stays put — it only flies when an inspect command arrives)
# and as the start pose for the fly-to-bin pan.
_HOME_CAM  = tuple(drone_pos)                 # (-2, -2, 1) by default
_HOME_LOOK = (1.2 + ox, -0.4 + oy, 1.2 + oz)  # overview of the loaded rack/aisle

# Waypoint traversal
_waypoints = []        # list of Waypoint objects
_wp_idx = 0            # index into _waypoints
_ARRIVE_TOL = 0.15     # m

# Scanning hold
_scan_frames_left = 0
_SCAN_HOLD_FRAMES = 30  # (legacy) frame-based hold — superseded by the time-based dwell below
# Time-based scan dwell: linger at the scan pose attempting to decode for up to this
# many wall-clock SECONDS (independent of stream fps), returning early once decoded.
# A frame-count hold is fps-dependent: after the camera-pose fps fix raised fps ~5x,
# a 30-frame hold shrank from ~15 s to ~4 s — too short for the async RTX emissive
# label texture to stream in, so scans stopped decoding. Seconds fix that for good.
_SCAN_MAX_SECONDS = 8.0
_scan_start_time = 0.0

# Extra high-quality warmup frames rendered AT the scan pose BEFORE attempting
# to decode the QR.  RTX needs a few frames at the new camera position for
# materials/textures to converge — verify_capture uses 10 such frames and
# reliably decodes.  We use 8 here (faster than verify_capture's 10 because
# the scene is already loaded and shaders are compiled from the IDLE sweep).
_SCAN_WARMUP_FRAMES = 2
_scan_warmup_left = 0   # countdown; >0 means we're still warming up
_scan_emitted = False   # True once the inspection result for this scan was emitted

# QR detection result for current scan
_last_scan_dets = []

# Clearance (simple AABB check, best-effort)
try:
    from sim.obstacles import get_clearance_aabbs
    _clearance_aabbs = get_clearance_aabbs()
    def _compute_clearance(pos):
        min_c = 9999.0
        px, py, pz = pos
        for aabb in _clearance_aabbs:
            # Support both flat 6-tuples and nested ((min),(max)) tuples
            if len(aabb) == 2:
                (ax0, ay0, az0), (ax1, ay1, az1) = aabb
            else:
                ax0, ay0, az0, ax1, ay1, az1 = aabb
            dx = max(ax0 - px, 0, px - ax1)
            dy = max(ay0 - py, 0, py - ay1)
            dz = max(az0 - pz, 0, pz - az1)
            d = math.sqrt(dx*dx + dy*dy + dz*dz)
            min_c = min(min_c, d)
        return min_c
except Exception:
    def _compute_clearance(pos):
        return 9.9


def _lerp(a, b, t):
    return a + (b - a) * t


def _interp_toward(pos, target_pos, step=0.05):
    """Move pos toward target_pos by `step` metres; return (new_pos, arrived)."""
    px, py, pz = pos
    tx, ty, tz = target_pos
    dx, dy, dz = tx - px, ty - py, tz - pz
    dist = math.sqrt(dx*dx + dy*dy + dz*dz)
    if dist < _ARRIVE_TOL:
        return list(target_pos), True
    t = min(step / dist, 1.0)
    return [px + dx*t, py + dy*t, pz + dz*t], False


# ---------------------------------------------------------------------------
# 10. Scan-pose helpers and sweep camera (used in IDLE / FLYING)
# ---------------------------------------------------------------------------
_sweep_total_frames = args.fps * 30   # 30-second full cycle
_frame_count = 0
_dets_count  = 0

# Load bin_map once for scan-pose lookups
try:
    from sim.bin_map import load_bin_map as _load_bin_map
    _BIN_MAP = _load_bin_map()
except Exception as _bm_e:
    print(f"[live_sim] WARNING: could not load bin_map: {_bm_e}")
    _BIN_MAP = {}

# Pre-compute ordered list of bin scan poses for IDLE sweep
# Order: column-major, level ascending per column (A1..A6, B1..B6, C1..C6)
_SWEEP_BIN_IDS = [f"{c}{l}" for c in COLUMNS for l in LEVELS]


def _bin_scan_pose(bin_id: str) -> tuple:
    """Return (cam_pos, look_at) for the proven scan-pose framing of bin_id.

    Uses bin_map scan_pose.position as the camera position and derives look_at
    from the label face geometry (5 mm proud of box front face).  This is
    identical to verify_capture's proven geometry that reliably decodes QR.
    """
    if bin_id in _BIN_MAP:
        sp = _BIN_MAP[bin_id]["scan_pose"]["position"]
        cam_x, cam_y, cam_z = float(sp[0]), float(sp[1]), float(sp[2])
    else:
        # Fallback: compute from config constants
        col_letter = bin_id[0] if bin_id else "A"
        try:
            level = int(bin_id[1:])
        except Exception:
            level = 1
        col_idx = COLUMNS.index(col_letter) if col_letter in COLUMNS else 0
        cam_x = col_idx * COLUMN_SPACING
        cam_y = -(BOX_D / 2 + SCAN_STANDOFF)
        cam_z = (level - 1) * LEVEL_HEIGHT + BOX_H / 2

    # Look-at = label centre: same X and Z as camera, y just in front of box face
    look_at_x = cam_x
    look_at_y = -(BOX_D / 2) - 0.005
    look_at_z = cam_z
    return (cam_x, cam_y, cam_z), (look_at_x, look_at_y, look_at_z)


def _sweep_camera_pos(frame: int) -> tuple:
    """IDLE sweep: smoothly pan between consecutive bin scan poses."""
    period = _sweep_total_frames
    t = (frame % period) / period

    n_total = len(_SWEEP_BIN_IDS)
    # Which bin we're "at" in the sweep and sub-t within that bin dwell
    bin_float = t * n_total
    bin_idx_a = int(bin_float) % n_total
    bin_idx_b = (bin_idx_a + 1) % n_total
    sub_t = bin_float - int(bin_float)  # 0..1

    bid_a = _SWEEP_BIN_IDS[bin_idx_a]
    bid_b = _SWEEP_BIN_IDS[bin_idx_b]

    cam_a, lat_a = _bin_scan_pose(bid_a)
    cam_b, lat_b = _bin_scan_pose(bid_b)

    # Smooth ease in/out: slow at start/end, faster in middle
    alpha = sub_t * sub_t * (3.0 - 2.0 * sub_t)  # smoothstep

    cam_pos = (
        _lerp(cam_a[0], cam_b[0], alpha),
        _lerp(cam_a[1], cam_b[1], alpha),
        _lerp(cam_a[2], cam_b[2], alpha),
    )
    look_at = (
        _lerp(lat_a[0], lat_b[0], alpha),
        _lerp(lat_a[1], lat_b[1], alpha),
        _lerp(lat_a[2], lat_b[2], alpha),
    )
    return cam_pos, look_at


# ---------------------------------------------------------------------------
# 11. Overlay helper
# ---------------------------------------------------------------------------

def _draw_overlay(rgb: np.ndarray, frame: int, sim_time: float,
                  cam_pos: tuple, dets: list, state: str, target) -> np.ndarray:
    if not _CV2_OK:
        return rgb

    bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    h, w = bgr.shape[:2]

    for d in dets:
        pts = d.polygon
        if pts and len(pts) >= 4:
            poly = np.array([[p.x, p.y] for p in pts], dtype=np.int32).reshape(-1, 1, 2)
            cv2.polylines(bgr, [poly], isClosed=True, color=(0, 255, 0), thickness=3)
            r = d.rect
            tx, ty = r.left, r.top - 12
            if ty < 20:
                ty = r.top + r.height + 22
            try:
                txt = d.data.decode("utf-8", errors="replace")
            except Exception:
                txt = str(d.data)
            cv2.putText(bgr, txt, (tx, ty),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65,
                        (0, 255, 0), 2, cv2.LINE_AA)
        else:
            r = d.rect
            cv2.rectangle(bgr, (r.left, r.top),
                          (r.left + r.width, r.top + r.height),
                          (0, 255, 0), 3)

    bar_h = 36
    overlay = bgr.copy()
    cv2.rectangle(overlay, (0, 0), (w, bar_h), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.7, bgr, 0.3, 0, bgr)

    cx, cy, cz = cam_pos
    tgt_str = f"→{target}" if target else "IDLE"
    hud = (f"DRONE LIVE [{state} {tgt_str}]  |  frame={frame}  |  "
           f"t={sim_time:.1f}s  |  pos=({cx:.2f},{cy:.2f},{cz:.2f})  |  "
           f"QR={len(dets)}")
    cv2.putText(bgr, hud, (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX, 0.52,
                (0, 255, 180), 1, cv2.LINE_AA)

    rgb[:] = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    return rgb


# ---------------------------------------------------------------------------
# 12. Main loop
# ---------------------------------------------------------------------------
_streaming_announced = False
_start_time  = time.time()
_frame_t0    = _start_time
_fps_window  = []

# WS broadcast counters
_WS_FRAME_INTERVAL = 1.0 / 10   # ~10 fps
_last_ws_frame_t = 0.0

# LiDAR read counter (throttled to every 6th render frame → ~3-5 Hz)
_LIDAR_FRAME_STRIDE = 6
_lidar_warmup_frames = 30  # skip first N frames while RTX warms up

# Inspection serial counter (so WS thread detects "new" inspection)
_inspection_serial = 0

print("[live_sim] entering main loop...")

try:
    while True:
        now     = time.time()
        elapsed = now - _start_time
        if args.duration > 0 and elapsed >= args.duration:
            print(f"[live_sim] duration {args.duration}s reached, stopping.")
            break

        # ---- Consume commands ----
        while not _cmd_queue.empty():
            try:
                cmd = _cmd_queue.get_nowait()
            except queue.Empty:
                break
            action = cmd.get("action", "")
            if action == "inspect":
                bin_id = cmd.get("bin_id", "")
                try:
                    wps = plan_waypoints(bin_id)
                    _waypoints = wps
                    _wp_idx = 1  # skip index 0 (home) since we're already there
                    _drone_target = bin_id
                    _drone_state = "FLYING"
                    _scan_frames_left = 0
                    _scan_emitted = False   # allow one emit for this new inspection
                    print(f"[live_sim] cmd: inspect {bin_id} — {len(wps)} waypoints")
                except Exception as _wp_e:
                    print(f"[live_sim] plan_waypoints failed for {bin_id}: {_wp_e}")
            elif action == "home":
                _drone_target = None
                _drone_state = "IDLE"
                _waypoints = []
                _wp_idx = 0
                _drone_pos[:] = drone_pos
                print("[live_sim] cmd: home")

        # ---- State machine: update drone position ----
        if _drone_state == "FLYING" and _waypoints and _wp_idx < len(_waypoints):
            wp = _waypoints[_wp_idx]
            new_pos, arrived = _interp_toward(_drone_pos, wp.position, step=0.35)
            _drone_pos[:] = new_pos
            if arrived:
                if wp.label == "scan":
                    _drone_state = "SCANNING"
                    _scan_frames_left = _SCAN_HOLD_FRAMES
                    _scan_warmup_left = _SCAN_WARMUP_FRAMES
                    _scan_start_time = time.time()
                    print(f"[live_sim] reached scan pose for {_drone_target} — decoding up to {_SCAN_MAX_SECONDS:.0f}s")
                else:
                    _wp_idx += 1
                    if _wp_idx >= len(_waypoints):
                        _drone_state = "IDLE"
                        _drone_target = None
                        print("[live_sim] waypoint traversal complete")

        elif _drone_state == "SCANNING":
            # Warmup phase: render a couple frames at the scan pose before decoding.
            if _scan_warmup_left > 0:
                _scan_warmup_left -= 1
            else:
                # Time-based dwell: keep rendering + decoding at the scan pose until we
                # decode (the emit path sets _scan_emitted) OR the wall-clock budget
                # elapses. fps-independent, so the async label texture has time to load.
                if _scan_emitted or (time.time() - _scan_start_time) >= _SCAN_MAX_SECONDS:
                    _drone_state = "RETURNING"
                    _wp_idx += 1  # move past scan wp to the return path
                    print(f"[live_sim] scan {'decoded' if _scan_emitted else 'timed out'}, returning home")

        elif _drone_state == "RETURNING" and _waypoints and _wp_idx < len(_waypoints):
            wp = _waypoints[_wp_idx]
            new_pos, arrived = _interp_toward(_drone_pos, wp.position, step=0.35)
            _drone_pos[:] = new_pos
            if arrived:
                _wp_idx += 1
                if _wp_idx >= len(_waypoints):
                    _drone_state = "IDLE"
                    _drone_target = None
                    print("[live_sim] returned home")

        # ---- Determine camera pose (always a scan-pose view of a label) ----
        if _drone_state == "IDLE":
            # Parked at home — the drone does NOT wander; it only flies on an
            # inspect command. Stationary overview of the rack from HOME.
            cam_pos, look_at = _HOME_CAM, _HOME_LOOK
        elif _drone_state == "SCANNING":
            # Hold exactly at the target bin's scan pose
            cam_pos, look_at = _bin_scan_pose(_drone_target)
        else:
            # FLYING or RETURNING: interpolate camera between last-known bin
            # scan pose (the sweep bin at current frame) and target bin scan pose.
            # This gives a smooth pan along the rack rather than a 3rd-person drone shot.
            sweep_cam, sweep_look = _HOME_CAM, _HOME_LOOK
            if _drone_target and _drone_target in _BIN_MAP:
                tgt_cam, tgt_look = _bin_scan_pose(_drone_target)
                # Progress 0→1 based on waypoint index along outbound route
                # waypoints: [home, pre_entry, entry, aisle_col, scan, ...]
                # We're at _wp_idx; scan waypoint is index 4 (label "scan")
                # Map _wp_idx 1..3 → 0..1 for outbound, 5..8 → 1..0 for return
                if _drone_state == "FLYING":
                    max_fly_idx = 4  # scan waypoint index
                    alpha = min(_wp_idx / max(max_fly_idx, 1), 1.0)
                else:  # RETURNING
                    # Returning: scan=4, home=8 → alpha goes 1..0
                    scan_idx = 4
                    home_idx = 8
                    alpha = 1.0 - min(
                        max(_wp_idx - scan_idx, 0) / max(home_idx - scan_idx, 1),
                        1.0
                    )
                # Smoothstep
                alpha = alpha * alpha * (3.0 - 2.0 * alpha)
                cam_pos = (
                    _lerp(sweep_cam[0], tgt_cam[0], alpha),
                    _lerp(sweep_cam[1], tgt_cam[1], alpha),
                    _lerp(sweep_cam[2], tgt_cam[2], alpha),
                )
                look_at = (
                    _lerp(sweep_look[0], tgt_look[0], alpha),
                    _lerp(sweep_look[1], tgt_look[1], alpha),
                    _lerp(sweep_look[2], tgt_look[2], alpha),
                )
            else:
                cam_pos, look_at = sweep_cam, sweep_look

        _update_cam_xform(cam_pos, look_at)

        # ---- Move the ACTUAL drone (so it visibly flies in the Isaac viewport) ----
        # spawn_drone() bakes the position onto /World/Drone/Body (the proxy that the
        # ANAFI visual is parented to), so we must move THAT prim — not the /World/Drone
        # ancestor (which would double-offset) — by SETTING its existing translate op.
        # Place it behind+below the onboard camera so the drone body never occludes the
        # label, and clamp Z to the ground rest height so it never sinks below the floor.
        # Drone body sits directly BELOW the onboard camera (not behind it in -Y):
        # keeps the drone centered in the aisle (~0.55 m clearance from each rack)
        # instead of hugging the 2nd rack, and the camera looks +Y so the body
        # (0.35 m below) stays out of the onboard frame.
        _drone_world = (
            float(cam_pos[0]),
            float(cam_pos[1]),
            max(float(cam_pos[2]) - 0.35, GROUND_DRONE_Z),
        )
        try:
            _body_prim = stage.GetPrimAtPath("/World/Drone/Body")
            if _body_prim.IsValid():
                _bxf = UsdGeom.Xformable(_body_prim)
                _bt_op = None
                for _op in _bxf.GetOrderedXformOps():
                    if _op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                        _bt_op = _op
                        break
                if _bt_op is None:
                    _bt_op = _bxf.AddTranslateOp()
                _bt_op.Set(Gf.Vec3d(*_drone_world))
            else:
                print("[live_sim] WARN: /World/Drone/Body not found — drone won't move")
        except Exception as _dm_e:
            print(f"[live_sim] drone move error: {_dm_e}")

        # ---- Move the beacon with the drone (slightly above body, below camera) ----
        if _beacon_t_op is not None:
            try:
                _beacon_t_op.Set(Gf.Vec3d(_drone_world[0], _drone_world[1], _drone_world[2] + 0.12))
            except Exception:
                pass

        # ---- Spinning rotors: follow the drone + spin every frame ----
        if _rotor_group_t_op is not None:
            try:
                _rotor_group_t_op.Set(Gf.Vec3d(_drone_world[0], _drone_world[1], _drone_world[2] + 0.06))
                _rotor_angle = (_rotor_angle + 45.0) % 360.0   # spin step per frame
                for _si, _sop in enumerate(_rotor_spin_ops):
                    # alternate spin direction per diagonal pair for realism
                    _dir = 1.0 if _si in (0, 3) else -1.0
                    _sop.Set(_dir * _rotor_angle)
            except Exception:
                pass

        # ---- Chase-cam: make the Isaac GUI viewport FOLLOW the drone while flying ----
        # While the drone is moving (FLYING/SCANNING/RETURNING) the persp viewport
        # tracks it (eye behind + above the drone, looking at it) so you can watch it
        # navigate. While IDLE the viewport is left alone (wide overview / your orbit).
        if _set_camera_view is not None and not args.headless and _drone_state != "IDLE":
            try:
                _cdx, _cdy, _cdz = _drone_world
                # Chase from BEHIND the drone along the aisle (-X), ON the aisle
                # centerline, slightly above — the aisle centre has no boxes so the
                # line of sight to the drone is clear (verified). The bright beacon
                # makes the drone easy to spot in the corridor.
                _set_camera_view(
                    eye=[_cdx - 2.2, _cdy, _cdz + 0.6],   # behind along aisle, just above
                    target=[_cdx, _cdy, _cdz + 0.15],     # the drone + beacon
                    camera_prim_path="/OmniverseKit_Persp",
                )
            except Exception:
                pass  # never let viewport tracking break the sim loop

        # Periodic debug: confirm the drone is actually being repositioned in-sim
        if _frame_count % 60 == 0:
            print(f"[live_sim] state={_drone_state} target={_drone_target} "
                  f"drone_world=({_drone_world[0]:.2f},{_drone_world[1]:.2f},{_drone_world[2]:.2f})")

        # ---- Step simulation ----
        # Warmup phase at scan pose: render multiple high-quality steps so RTX
        # fully converges textures/materials at the new camera position.
        # Normal streaming: low subframes for smooth fps.
        # SCANNING (post-warmup): high subframes for crisp QR pixels.
        simulation_app.update()
        if _drone_state == "SCANNING" and _scan_warmup_left > 0:
            # Brief warmup at modestly-higher quality (QR decodes fine at sf~12;
            # the IDLE stream already decodes at sf=6, so no sf=64 needed).
            rep.orchestrator.step(rt_subframes=16, wait_for_render=True)
        elif _drone_state == "SCANNING":
            rep.orchestrator.step(rt_subframes=12, wait_for_render=True)
        else:
            rep.orchestrator.step(rt_subframes=6, wait_for_render=True)

        # ---- Pull rendered frame ----
        data = _annot.get_data()
        if data is None:
            _frame_count += 1
            continue

        arr = np.asarray(data)
        if arr.ndim == 3 and arr.shape[2] >= 3:
            rgb = np.ascontiguousarray(arr[:, :, :3], dtype=np.uint8)
        elif arr.ndim == 3:
            rgb = np.ascontiguousarray(arr, dtype=np.uint8)
        else:
            _frame_count += 1
            continue

        if rgb.shape[0] != H or rgb.shape[1] != W:
            _frame_count += 1
            continue

        # ---- QR detection ----
        dets = []
        if _PYZBAR_OK:
            try:
                from PIL import Image as _PIL_Image
                _pil = _PIL_Image.fromarray(rgb)
                raw_dets = pyzbar_decode(_pil)
                dets = raw_dets
                _dets_count += len(dets)
            except Exception:
                pass

        # Note: the per-frame sf=64/96 "retry" renders were removed — each blocks the
        # render loop ~1-2 s (starving the async WebSocket server → dropped clients)
        # and is unnecessary now: the time-based scan dwell attempts a decode every
        # frame at sf=12 over ~8 s, which is plenty once the emissive label texture
        # has streamed in (the label is a flat texture, so it decodes at low subframes).

        # ---- If SCANNING (post-warmup), try to decode QR and emit inspection result ----
        if _drone_state == "SCANNING" and _scan_warmup_left == 0 and _drone_target and dets and not _scan_emitted:
            # Use first QR detection
            try:
                qr_text = dets[0].data.decode("utf-8", errors="replace").strip()
                # QR payload format: JSON {"part_no": "PN-A01", "qty": 11}
                # (generated by sim.gr_label.encode_payload).
                # Fallback: legacy "PN-A01:11" or "PN-A01,11" formats.
                scanned_part = qr_text
                scanned_qty = None
                try:
                    import json as _json_mod
                    _qr_obj = _json_mod.loads(qr_text)
                    scanned_part = str(_qr_obj.get("part_no", qr_text))
                    scanned_qty = int(_qr_obj.get("qty", 0)) if "qty" in _qr_obj else None
                except Exception:
                    # Legacy separator formats
                    for sep in (":", ",", " "):
                        if sep in qr_text:
                            parts = qr_text.split(sep, 1)
                            scanned_part = parts[0].strip()
                            try:
                                scanned_qty = int(parts[1].strip())
                            except Exception:
                                pass
                            break

                sap = get_inventory(_drone_target)
                system_part = sap["part_no"] if sap else None
                system_qty  = sap["qty"] if sap else None

                match = (scanned_part == system_part) and (scanned_qty == system_qty)
                status = "completed" if match else "discrepancy"

                _inspection_serial += 1
                with _inspection_lock:
                    _pending_inspection.update({
                        "_id": _inspection_serial,
                        "bin_id": _drone_target,
                        "scanned_part": scanned_part,
                        "scanned_qty": scanned_qty,
                        "system_part": system_part,
                        "system_qty": system_qty,
                        "match": match,
                        "status": status,
                        "ts": time.time(),
                    })
                print(f"[live_sim] inspection result: bin={_drone_target} "
                      f"scanned={scanned_part}/{scanned_qty} "
                      f"sap={system_part}/{system_qty} match={match} status={status}")
                _scan_emitted = True       # emit only once per inspection
                _scan_frames_left = 0      # done scanning → return home promptly
            except Exception as _insp_e:
                print(f"[live_sim] inspection decode error: {_insp_e}")

        # ---- Detection list for telemetry ----
        det_list = []
        for d in dets:
            try:
                txt = d.data.decode("utf-8", errors="replace").strip()
                r = d.rect
                scanned_part = txt
                scanned_qty = None
                try:
                    import json as _json_mod
                    _obj = _json_mod.loads(txt)
                    scanned_part = str(_obj.get("part_no", txt))
                    scanned_qty = int(_obj.get("qty", 0)) if "qty" in _obj else None
                except Exception:
                    for sep in (":", ",", " "):
                        if sep in txt:
                            parts_sp = txt.split(sep, 1)
                            scanned_part = parts_sp[0].strip()
                            try:
                                scanned_qty = int(parts_sp[1].strip())
                            except Exception:
                                pass
                            break
                det_list.append({
                    "part_no": scanned_part,
                    "qty": scanned_qty,
                    "bbox": [r.left, r.top, r.left + r.width, r.top + r.height],
                })
            except Exception:
                pass

        # ---- Read RTX LiDAR point cloud (throttled, non-blocking) ----
        _lidar_min_dist = None
        if (_lidar_annot is not None
                and _frame_count >= _lidar_warmup_frames
                and _frame_count % _LIDAR_FRAME_STRIDE == 0):
            try:
                _lidar_raw = _lidar_annot.get_data()
                # Print dict keys once so we know the exact schema at runtime
                if not _lidar_keys_printed and _lidar_raw is not None:
                    _k = list(_lidar_raw.keys()) if isinstance(_lidar_raw, dict) else type(_lidar_raw).__name__
                    _s = {k: (getattr(v, 'shape', None) or (len(v) if hasattr(v, '__len__') else type(v).__name__))
                          for k, v in (_lidar_raw.items() if isinstance(_lidar_raw, dict) else [])}
                    print(f"[live_sim] lidar get_data() keys={_k} shapes={_s}")
                    _lidar_keys_printed = True

                # Extract XYZ from the annotator dict.
                # Primary key "data" → Nx3 float32.  Also handle "point_cloud",
                # "hitPointNormals" (older), and per-channel x/y/z arrays.
                _pts_raw = None
                if isinstance(_lidar_raw, dict):
                    if "data" in _lidar_raw and _lidar_raw["data"] is not None:
                        _pts_raw = np.asarray(_lidar_raw["data"], dtype=np.float32)
                        # Might be Nx3 or flat N*3; reshape if flat
                        if _pts_raw.ndim == 1 and len(_pts_raw) % 3 == 0:
                            _pts_raw = _pts_raw.reshape(-1, 3)
                    elif "point_cloud" in _lidar_raw and _lidar_raw["point_cloud"] is not None:
                        _pts_raw = np.asarray(_lidar_raw["point_cloud"], dtype=np.float32)
                        if _pts_raw.ndim == 1 and len(_pts_raw) % 3 == 0:
                            _pts_raw = _pts_raw.reshape(-1, 3)
                    elif all(k in _lidar_raw for k in ("x", "y", "z")):
                        _x = np.asarray(_lidar_raw["x"], dtype=np.float32).ravel()
                        _y = np.asarray(_lidar_raw["y"], dtype=np.float32).ravel()
                        _z = np.asarray(_lidar_raw["z"], dtype=np.float32).ravel()
                        if len(_x) > 0:
                            _pts_raw = np.stack([_x, _y, _z], axis=1)
                elif _lidar_raw is not None:
                    # Bare array path (annotator returned ndarray directly)
                    _pts_raw = np.asarray(_lidar_raw, dtype=np.float32)
                    if _pts_raw.ndim == 1 and len(_pts_raw) % 3 == 0:
                        _pts_raw = _pts_raw.reshape(-1, 3)

                if (_pts_raw is not None
                        and _pts_raw.ndim == 2
                        and _pts_raw.shape[1] >= 3
                        and len(_pts_raw) > 0):
                    # Filter out zero/invalid points
                    valid = np.any(_pts_raw[:, :3] != 0, axis=1)
                    pts = _pts_raw[valid, :3]
                    if len(pts) > 0:
                        # Downsample to <= 1500 points via stride
                        n_raw = len(pts)
                        stride = max(1, n_raw // 1500)
                        pts_ds = pts[::stride]
                        # Closest point distance
                        dists = np.linalg.norm(pts_ds, axis=1)
                        nonzero = dists[dists > 0.05]
                        if len(nonzero) > 0:
                            _lidar_min_dist = float(np.min(nonzero))
                        # Round to cm for compact JSON
                        pts_list = [[round(float(p[0]), 2),
                                     round(float(p[1]), 2),
                                     round(float(p[2]), 2)]
                                    for p in pts_ds]
                        with _lidar_lock:
                            _latest_lidar["points"] = pts_list
                            _latest_lidar["n"] = len(pts_list)
                            _latest_lidar["ts"] = time.time()
            except Exception as _lidar_e:
                # Never let lidar failure crash camera stream
                pass

        # ---- Update telemetry ----
        # pos = camera position so the 3D-map drone marker tracks the inspection path
        clearance = _compute_clearance(list(cam_pos))
        with _telem_lock:
            _telem.update({
                "pos": list(cam_pos),
                "target": _drone_target,
                "state": _drone_state,
                "clearance": round(clearance, 3),
                "lidar_min": round(_lidar_min_dist, 3) if _lidar_min_dist is not None else None,
                "detections": det_list,
            })

        # ---- Draw overlay ----
        if _CV2_OK:
            _draw_overlay(rgb, _frame_count, elapsed, cam_pos, dets,
                          _drone_state, _drone_target)

        # ---- Encode to JPEG and push to WS frame holder ----
        now2 = time.time()
        if now2 - _last_ws_frame_t >= _WS_FRAME_INTERVAL:
            _last_ws_frame_t = now2
            if _CV2_OK:
                small = cv2.resize(
                    cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR),
                    (960, 540), interpolation=cv2.INTER_LINEAR
                )
                ok, enc = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 70])
                if ok:
                    b64 = base64.b64encode(enc.tobytes()).decode("ascii")
                    with _latest_frame_lock:
                        _latest_frame["jpeg"] = b64
                        _latest_frame["w"] = 960
                        _latest_frame["h"] = 540
                        _latest_frame["ts"] = now2

        # ---- Optional UDP write ----
        if _ffmpeg_proc is not None:
            if _ffmpeg_proc.poll() is not None:
                _err = _ffmpeg_proc.stderr.read().decode(errors="replace")
                print(f"[live_sim] ffmpeg exited rc={_ffmpeg_proc.returncode}: {_err[:300]}")
                _ffmpeg_proc = None
            else:
                try:
                    _ffmpeg_proc.stdin.write(rgb.tobytes())
                    _ffmpeg_proc.stdin.flush()
                except BrokenPipeError:
                    print("[live_sim] ffmpeg stdin broken pipe")
                    _ffmpeg_proc = None

        # ---- Announce (once) ----
        if not _streaming_announced:
            _streaming_announced = True
            if args.udp and _encoder:
                print(f"LIVE_SIM streaming {_udp_url.split('?')[0]}  [encoder={_encoder}]")

        # ---- Periodic log ----
        if _frame_count > 0 and _frame_count % 30 == 0:
            ft = time.time()
            _fps_window.append(ft - _frame_t0)
            _frame_t0 = ft
            recent_fps = 30.0 / (_fps_window[-1] if _fps_window[-1] > 0 else 1.0)
            n_dets_recent = _dets_count
            _dets_count = 0
            print(f"[live_sim] frame={_frame_count} dets={n_dets_recent} "
                  f"fps={recent_fps:.1f} elapsed={elapsed:.0f}s "
                  f"state={_drone_state} target={_drone_target}")

        _frame_count += 1

except KeyboardInterrupt:
    print("[live_sim] KeyboardInterrupt — shutting down...")

# ---------------------------------------------------------------------------
# 13. Shutdown
# ---------------------------------------------------------------------------
print(f"[live_sim] total frames: {_frame_count}")
if _ffmpeg_proc is not None:
    try:
        _ffmpeg_proc.stdin.close()
    except Exception:
        pass
    try:
        _ffmpeg_proc.wait(timeout=5)
    except Exception:
        _ffmpeg_proc.kill()

simulation_app.close()
print("[live_sim] done.")
