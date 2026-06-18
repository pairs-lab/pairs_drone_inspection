"""Mission flythrough recorder — drone camera video with live QR detection overlay.

Flies the camera kinematically down the warehouse aisle visiting a sequence of bins
(A1 → A3 → B2 → B5 → C1 → C4 → C6) with smooth interpolation between waypoints.

At EACH rendered frame:
  - Real RTX render from the drone onboard camera (near-clip 0.01)
  - pyzbar QR decode: green rectangle + decoded text over each detection
  - HUD overlay: target bin, drone XYZ, "DRONE AI CAMERA — LIVE", frame counter,
    min-clearance to obstacles

Output:
  sim/assets/mission.mp4          — MP4 at 15 fps (imageio/ffmpeg)
  sim/assets/mission_frames/      — individual PNGs fallback
  backend/mission_trajectory.json — trajectory + detections per frame

Prints: MISSION_OK frames=<n> video=sim/assets/mission.mp4

Run with:
    scripts/run_isaac.sh scripts/record_mission.py 2>&1 | tail -30
"""

import os
import sys
import json
import math
import time
import numpy as np

# ── Isaac Sim boot ────────────────────────────────────────────────────────────
from isaacsim import SimulationApp

app = SimulationApp({
    "headless": True,
    "active_gpu": 0,
    "physics_gpu": 0,
    "width": 1280,
    "height": 720,
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
from PIL import Image, ImageDraw, ImageFont

from sim.warehouse import use_local_assets, build_warehouse_env
from sim.rack import build_rack, build_second_rack, build_aisle_obstacle
from sim.bin_map import load_bin_map
from sim.config import (
    BOX_D, BOX_H, AISLE_CENTER_Y, AISLE_WIDTH,
    RACK_X_MIN, RACK_X_MAX
)
from sim.obstacles import get_clearance_aabbs
from sim.gr_label import decode_payload

try:
    from pyzbar.pyzbar import decode as pyzbar_decode
    PYZBAR_OK = True
except Exception as e:
    print(f"[record_mission] WARNING: pyzbar unavailable: {e}")
    PYZBAR_OK = False

# ── Output paths ──────────────────────────────────────────────────────────────
REPO_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
FRAMES_DIR = os.path.join(REPO_ROOT, "sim", "assets", "mission_frames")
VIDEO_PATH = os.path.join(REPO_ROOT, "sim", "assets", "mission.mp4")
TRAJ_PATH  = os.path.join(REPO_ROOT, "backend", "mission_trajectory.json")
os.makedirs(FRAMES_DIR, exist_ok=True)
os.makedirs(os.path.join(REPO_ROOT, "sim", "assets"), exist_ok=True)

# ── Scene build ───────────────────────────────────────────────────────────────
use_local_assets()
stage = omni.usd.get_context().get_stage()
UsdGeom.SetStageUpAxis(stage, "Z")
UsdGeom.Xform.Define(stage, "/World")
build_warehouse_env(stage, kind="warehouse")
n_bins = build_rack(stage)
build_second_rack(stage)
build_aisle_obstacle(stage)
print(f"[record_mission] scene built: {n_bins} bins + second rack + obstacle")
assert n_bins == 18

UsdLux.DomeLight.Define(stage, "/World/Dome").CreateIntensityAttr(1200.0)

bin_map = load_bin_map()
aabbs   = get_clearance_aabbs()

# ── Mission waypoint plan ─────────────────────────────────────────────────────
# Each waypoint is a dict: pos=[x,y,z], look=[x,y,z], bin_id, hold_frames
# We visit 7 bins across different columns / levels for a compelling ~30s clip.
MISSION_BINS = ["A1", "A3", "B2", "B5", "C1", "C4", "C6"]
# Approach positions: fly in aisle at AISLE_CENTER_Y=-0.90, then step to scan_pose
AISLE_CRUISE_Z  = 1.2   # m — cruise height (above 0.6m obstacle)
ENTRY_X         = -1.5  # m — aisle entry X
EXIT_X          =  3.5  # m — aisle exit X

def _scan_pos(bid):
    sp = bin_map[bid]["scan_pose"]["position"]
    return [float(sp[0]), float(sp[1]), float(sp[2])]

def _label_center(bid):
    sp = bin_map[bid]["scan_pose"]["position"]
    lx = float(sp[0])
    ly = -BOX_D / 2 - 0.005
    lz = float(sp[2])
    return [lx, ly, lz]

def build_waypoints(mission_bins):
    """Return list of (cam_pos, look_at, bin_id_label, hold_frames)."""
    wps = []
    # -- Intro: wide oblique shot looking down the aisle entrance
    wps.append({
        "pos":   [-2.0, AISLE_CENTER_Y - 0.3, 2.5],
        "look":  [1.2,  AISLE_CENTER_Y,       1.0],
        "label": "ENTRY",
        "hold":  4,
    })
    # -- Enter aisle at low altitude
    wps.append({
        "pos":   [ENTRY_X, AISLE_CENTER_Y, AISLE_CRUISE_Z],
        "look":  [2.0, AISLE_CENTER_Y, AISLE_CRUISE_Z],
        "label": "AISLE_ENTER",
        "hold":  3,
    })
    for bid in mission_bins:
        sp = _scan_pos(bid)
        lc = _label_center(bid)
        bin_x = sp[0]
        # -- aisle cruise to column
        wps.append({
            "pos":   [bin_x, AISLE_CENTER_Y, AISLE_CRUISE_Z],
            "look":  [bin_x, -0.45,           sp[2]],
            "label": f"APPROACH_{bid}",
            "hold":  3,
        })
        # -- pull in to scan pose (from aisle, angle to label)
        wps.append({
            "pos":   sp,
            "look":  lc,
            "label": bid,
            "hold":  8,   # linger at scan pose for QR detection
        })
    # -- Exit aisle
    wps.append({
        "pos":   [EXIT_X, AISLE_CENTER_Y, AISLE_CRUISE_Z],
        "look":  [EXIT_X + 1.0, AISLE_CENTER_Y, 1.5],
        "label": "AISLE_EXIT",
        "hold":  3,
    })
    return wps

WAYPOINTS = build_waypoints(MISSION_BINS)

# ── Interpolation helpers ─────────────────────────────────────────────────────
INTERP_FRAMES_PER_WP = 10   # frames to interpolate between waypoints

def lerp(a, b, t):
    a, b = np.array(a, dtype=float), np.array(b, dtype=float)
    return (a + (b - a) * t).tolist()

def build_camera_path():
    """Expand waypoints into per-frame (pos, look, bin_label) list."""
    frames = []
    for i, wp in enumerate(WAYPOINTS):
        # Hold frames at this waypoint
        for _ in range(wp["hold"]):
            frames.append((wp["pos"], wp["look"], wp["label"]))
        # Interpolate to next
        if i + 1 < len(WAYPOINTS):
            nxt = WAYPOINTS[i + 1]
            for k in range(1, INTERP_FRAMES_PER_WP + 1):
                t = k / INTERP_FRAMES_PER_WP
                frames.append((
                    lerp(wp["pos"],  nxt["pos"],  t),
                    lerp(wp["look"], nxt["look"], t),
                    wp["label"],
                ))
    return frames

CAM_PATH = build_camera_path()
print(f"[record_mission] camera path: {len(CAM_PATH)} frames")

# ── USD camera prim (will be repositioned per frame) ─────────────────────────
cam_pos0, look0, _ = CAM_PATH[0]
cam = rep.create.camera(
    position=cam_pos0,
    look_at=look0,
    look_at_up_axis=(0.0, 0.0, 1.0),
    name="MissionCamera",
)
rp = rep.create.render_product(cam, (1280, 720), name="MissionCapture")
annot = rep.AnnotatorRegistry.get_annotator("LdrColor")
annot.attach([rp])

# near clip: set on all Replicator cameras
def _set_near_clip(near=0.01):
    for p in stage.Traverse():
        if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
            UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(near, 1_000_000.0))

_set_near_clip()

# ── Helper: move the named camera prim ───────────────────────────────────────
def _move_camera(cam_path_prim, pos, look_at, up=(0.0, 0.0, 1.0)):
    """Reposition camera by rebuilding ops (clear + set translate/orient)."""
    import math as _math
    prim = stage.GetPrimAtPath(cam_path_prim)
    if not prim.IsValid():
        return
    xf = UsdGeom.Xformable(prim)
    xf.ClearXformOpOrder()
    px, py, pz = pos
    lx, ly, lz = look_at
    ux, uy, uz = up
    # Build rotation: camera looks down -Z local; we need R such that R*(0,0,-1)= normalized(look-pos)
    fwd = np.array([lx - px, ly - py, lz - pz], dtype=float)
    n = np.linalg.norm(fwd)
    if n < 1e-6:
        fwd = np.array([0.0, 1.0, 0.0])
    else:
        fwd /= n
    up_v = np.array([ux, uy, uz], dtype=float)
    right = np.cross(fwd, up_v)
    rn = np.linalg.norm(right)
    if rn < 1e-6:
        right = np.array([1.0, 0.0, 0.0])
    else:
        right /= rn
    up_v = np.cross(right, fwd)
    # Camera local -Z = fwd, local X = right, local Y = up_v (right-hand cam)
    # Column-major Gf.Matrix4d: rows are basis vectors
    # USD: column 0 = right, column 1 = up, column 2 = -fwd (camera -Z = fwd)
    mat = Gf.Matrix4d(
        right[0],  right[1],  right[2],  0.0,
        up_v[0],   up_v[1],   up_v[2],   0.0,
        -fwd[0],  -fwd[1],   -fwd[2],   0.0,
        px,        py,        pz,        1.0,
    )
    xf.MakeMatrixXform().Set(mat)

# ── Clearance helper ──────────────────────────────────────────────────────────
def _point_clearance(pos):
    """Min distance from pos to any obstacle AABB surface."""
    min_d = float("inf")
    px, py, pz = pos
    for (mn, mx) in aabbs:
        dx = max(mn[0] - px, 0.0, px - mx[0])
        dy = max(mn[1] - py, 0.0, py - mx[1])
        dz = max(mn[2] - pz, 0.0, pz - mx[2])
        d = math.sqrt(dx*dx + dy*dy + dz*dz)
        if d < min_d:
            min_d = d
    return min_d

# ── HUD + QR overlay drawing ─────────────────────────────────────────────────
try:
    _FONT = ImageFont.truetype("DejaVuSans-Bold.ttf", 20)
    _FONT_SM = ImageFont.truetype("DejaVuSans.ttf", 16)
except OSError:
    _FONT = ImageFont.load_default()
    _FONT_SM = _FONT

def draw_overlay(rgb_np, bin_label, cam_pos, frame_idx, min_clr, detections):
    """Draw HUD bar + QR boxes on rgb_np (H,W,3 uint8). Returns annotated PIL Image."""
    img = Image.fromarray(rgb_np)
    draw = ImageDraw.Draw(img)

    # --- QR detection boxes ---
    det_info = []
    for det in detections:
        poly = getattr(det, "polygon", None)
        rect = det.rect
        if poly and len(poly) >= 4:
            pts = [(p.x, p.y) for p in poly]
            draw.polygon(pts, outline=(0, 255, 0))
            draw.line(pts + [pts[0]], fill=(0, 255, 0), width=3)
        else:
            l, t, w, h = rect.left, rect.top, rect.width, rect.height
            draw.rectangle([l, t, l + w, t + h], outline=(0, 255, 0), width=3)

        try:
            pn, qty = decode_payload(det.data.decode())
            txt = f"{pn} / Qty:{qty}"
        except Exception:
            txt = det.data.decode("utf-8", errors="replace")[:30]

        tx = rect.left
        ty = max(0, rect.top - 24)
        draw.rectangle([tx, ty, tx + len(txt) * 10 + 4, ty + 22], fill=(0, 180, 0))
        draw.text((tx + 2, ty + 2), txt, fill=(255, 255, 255), font=_FONT_SM)
        det_info.append({"text": txt, "bbox": [rect.left, rect.top, rect.width, rect.height]})

    # --- HUD top bar ---
    bar_h = 52
    draw.rectangle([0, 0, img.width, bar_h], fill=(0, 0, 0, 200))

    # Left: DRONE AI CAMERA label
    draw.text((8, 6),  "DRONE AI CAMERA — LIVE", fill=(0, 220, 255), font=_FONT)
    # Center: target bin
    bin_txt = f"TARGET: {bin_label}"
    bw, _ = (len(bin_txt) * 12, 20)
    draw.text((img.width // 2 - bw // 2, 6), bin_txt, fill=(255, 220, 0), font=_FONT)
    # Right: frame counter + clearance
    rt_txt = f"FRAME {frame_idx:04d}   CLR {min_clr:.2f}m"
    draw.text((img.width - len(rt_txt) * 10 - 6, 6), rt_txt, fill=(180, 255, 180), font=_FONT_SM)
    # Second row: XYZ
    xyz_txt = f"XYZ: ({cam_pos[0]:+.2f}, {cam_pos[1]:+.2f}, {cam_pos[2]:+.2f})"
    draw.text((8, 30), xyz_txt, fill=(200, 200, 200), font=_FONT_SM)

    return img, det_info

# ── Initial RTX warmup ────────────────────────────────────────────────────────
print("[record_mission] initial RTX warmup ...")
for _ in range(80):
    app.update()
_set_near_clip()

# ── Main render loop ─────────────────────────────────────────────────────────
trajectory_frames = []
saved_frames = []

print(f"[record_mission] recording {len(CAM_PATH)} frames ...")
for frame_idx, (cam_pos, look_at, bin_label) in enumerate(CAM_PATH):
    # Move camera to this pose
    # Get the actual camera prim path from the rep camera node
    cam_prim_path = None
    for p in stage.Traverse():
        if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
            cam_prim_path = p.GetPath().pathString
            break
    if cam_prim_path:
        _move_camera(cam_prim_path, cam_pos, look_at)

    # Render
    for _ in range(3):
        rep.orchestrator.step(rt_subframes=16, wait_for_render=True)

    data = annot.get_data()
    if data is None:
        print(f"[record_mission] WARNING: frame {frame_idx} returned None, skipping")
        continue

    arr = np.asarray(data)
    if arr.ndim == 3 and arr.shape[2] >= 3:
        rgb = arr[:, :, :3].astype(np.uint8)
    else:
        rgb = arr.astype(np.uint8)

    if rgb.mean() < 2.0:
        # still dark, skip
        continue

    # QR decode
    detections = []
    if PYZBAR_OK:
        try:
            detections = pyzbar_decode(Image.fromarray(rgb))
        except Exception as e:
            pass

    # Clearance
    min_clr = _point_clearance(cam_pos)

    # Draw overlay
    annotated_img, det_info = draw_overlay(rgb, bin_label, cam_pos, frame_idx, min_clr, detections)

    # Save frame PNG
    frame_path = os.path.join(FRAMES_DIR, f"{len(saved_frames):04d}.png")
    annotated_img.save(frame_path)
    saved_frames.append(frame_path)

    # Trajectory record
    trajectory_frames.append({
        "t":            len(saved_frames) - 1,
        "frame":        len(saved_frames) - 1,
        "bin":          bin_label,
        "pos":          [round(float(cam_pos[0]), 4),
                         round(float(cam_pos[1]), 4),
                         round(float(cam_pos[2]), 4)],
        "detected":     det_info,
        "min_clearance": round(float(min_clr), 4),
    })

    if frame_idx % 20 == 0:
        n_det = len(detections)
        print(f"[record_mission] frame {frame_idx:03d}/{len(CAM_PATH)} "
              f"bin={bin_label} mean={rgb.mean():.1f} det={n_det} clr={min_clr:.2f}m")

print(f"[record_mission] captured {len(saved_frames)} frames")

# ── Encode MP4 ───────────────────────────────────────────────────────────────
video_ok = False
try:
    import imageio
    import imageio_ffmpeg  # ensure plugin is available
    fps = 15
    writer = imageio.get_writer(VIDEO_PATH, fps=fps, codec="libx264",
                                quality=7, macro_block_size=1)
    for fp in saved_frames:
        frame_arr = np.asarray(Image.open(fp))
        writer.append_data(frame_arr)
    writer.close()
    vsz = os.path.getsize(VIDEO_PATH)
    print(f"[record_mission] encoded MP4: {VIDEO_PATH} ({vsz/1024:.0f} KB)")
    video_ok = True
except Exception as e:
    print(f"[record_mission] imageio MP4 failed: {e}; frame sequence fallback at {FRAMES_DIR}")

    # cv2 fallback
    try:
        import cv2
        sample = np.asarray(Image.open(saved_frames[0]))
        h, w = sample.shape[:2]
        fourcc = cv2.VideoWriter_fourcc(*"mp4v")
        vw = cv2.VideoWriter(VIDEO_PATH, fourcc, 15, (w, h))
        for fp in saved_frames:
            bgr = cv2.cvtColor(np.asarray(Image.open(fp)), cv2.COLOR_RGB2BGR)
            vw.write(bgr)
        vw.release()
        vsz = os.path.getsize(VIDEO_PATH)
        print(f"[record_mission] encoded MP4 (cv2): {VIDEO_PATH} ({vsz/1024:.0f} KB)")
        video_ok = True
    except Exception as e2:
        print(f"[record_mission] cv2 MP4 also failed: {e2}; frames only saved in {FRAMES_DIR}")

# ── Write trajectory JSON ─────────────────────────────────────────────────────
from sim.config import (COLUMNS, LEVELS, COLUMN_SPACING, LEVEL_HEIGHT,
                        RACK_X_MIN, RACK_X_MAX, RACK_Y_MIN, RACK_Y_MAX, RACK_TOTAL_HEIGHT,
                        BOX_W, BOX_D, BOX_H, AISLE_WIDTH, AISLE_CENTER_Y,
                        SECOND_RACK_Y, OBSTACLE_X, OBSTACLE_Y, OBSTACLE_HALF_W,
                        OBSTACLE_HALF_D, OBSTACLE_HALF_H)

traj = {
    "schema_version": "1.0",
    "scene": {
        "warehouse": {"x_min": -5.0, "x_max": 15.0, "y_min": -5.0, "y_max": 5.0, "z": 0.0},
        "primary_rack": {
            "x_min": RACK_X_MIN, "x_max": RACK_X_MAX,
            "y_min": RACK_Y_MIN, "y_max": RACK_Y_MAX,
            "z_min": 0.0, "z_max": RACK_TOTAL_HEIGHT,
        },
        "second_rack": {
            "x_min": RACK_X_MIN, "x_max": RACK_X_MAX,
            "y_min": SECOND_RACK_Y - 0.45, "y_max": SECOND_RACK_Y + 0.45,
            "z_min": 0.0, "z_max": RACK_TOTAL_HEIGHT,
        },
        "aisle": {
            "center_y": AISLE_CENTER_Y,
            "width": AISLE_WIDTH,
            "x_min": RACK_X_MIN, "x_max": RACK_X_MAX,
        },
        "obstacle": {
            "center": [OBSTACLE_X, OBSTACLE_Y, OBSTACLE_HALF_H],
            "half_extents": [OBSTACLE_HALF_W, OBSTACLE_HALF_D, OBSTACLE_HALF_H],
        },
    },
    "mission_bins": MISSION_BINS,
    "total_frames": len(saved_frames),
    "fps": 15,
    "frames": trajectory_frames,
}

with open(TRAJ_PATH, "w") as f:
    json.dump(traj, f, indent=2)
print(f"[record_mission] trajectory written: {TRAJ_PATH} "
      f"({os.path.getsize(TRAJ_PATH)//1024} KB, {len(trajectory_frames)} frames)")

# ── Verify: open 3 frames and check ─────────────────────────────────────────
print("\n[record_mission] === FRAME VERIFICATION ===")
check_indices = [0, len(saved_frames)//3, 2*len(saved_frames)//3]
for ci in check_indices:
    fp = saved_frames[ci]
    img = Image.open(fp)
    arr = np.asarray(img)
    mean_px = arr.mean()
    h, w = arr.shape[:2]
    # Count green pixels (QR box overlay indicator)
    green_mask = (arr[:,:,1] > 180) & (arr[:,:,0] < 80) & (arr[:,:,2] < 80)
    n_green = int(green_mask.sum())
    print(f"  Frame {ci:04d}: {os.path.basename(fp)} shape={w}x{h} "
          f"mean={mean_px:.1f} green_pixels={n_green}")
    if mean_px < 5.0:
        print(f"  WARNING: frame {ci} looks blank!")
    else:
        print(f"  OK — scene rendered (non-blank)")

# Summary
print(f"\nMISSION_OK frames={len(saved_frames)} video={'sim/assets/mission.mp4' if video_ok else 'N/A (frames only)'}")
print(f"Trajectory: {len(trajectory_frames)} frames, "
      f"detections_total={sum(len(f['detected']) for f in trajectory_frames)}")

app.close()
