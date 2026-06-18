# Install Notes — Isaac Sim 6.0 (M1)

Ghi lại sai khác thực tế so với plan khi cài/verify.

## Kết luận: dùng bản BINARY, không dùng pip

**Bản pip `isaacsim[all]==6.0.0` BỊ THIẾU extension thiết yếu** → không launch được.
- Cài pip thành công (9.7GB, đủ 117 isaacsim extension) nhưng dependency solver fail liên tiếp:
  1. `isaacsim.anim.robot.schema` (none found) — schema, không cần.
  2. Sau khi đánh optional các schema anim/metropolis/agent → fail tiếp ở
     `isaacsim.util.debug_draw` (none found), vốn là **hard-dependency của
     `isaacsim.sensors.physx`** nên KHÔNG bỏ qua được.
- `isaacsim.util.debug_draw` không có trong bất kỳ wheel pip nào, không trong
  registry prod, không trong bản Isaac khác trên máy.
- Khớp lỗi đã biết: github.com/isaac-sim/IsaacLab/issues/5435 và NVIDIA forums —
  bản pip 6.0.0 (early dev) thiếu extension.

→ **Quyết định:** chuyển sang bản **binary/workstation 6.0.0** (gói đầy đủ, self-contained).
- Đã gỡ bản pip isaacsim khỏi env `isaac6` (giữ qrcode/pyzbar/PyYAML/Pillow/pytest
  cho test thuần Python của M1).
- `scripts/patch_isaac_kit.py` giữ lại để tham chiếu điều tra; KHÔNG dùng cho binary.

## Binary install (đang thực hiện)

- Download (host **downloads.isaacsim.nvidia.com**):
  - Binary: `isaac-sim-standalone-6.0.0-linux-x86_64.zip` (~13.08 GB)
  - Assets (tùy chọn, ~75GB/5 phần): `isaac-sim-assets-complete-6.0.0.00{1..5}.zip`
    — **tạm bỏ qua** vì POC dùng primitive; có thể kéo asset cụ thể từ Nucleus sau.
- Cài: `mkdir ~/isaacsim && unzip isaac-sim-standalone-6.0.0-linux-x86_64.zip -d ~/isaacsim && cd ~/isaacsim && ./post_install.sh`
- Chạy script Python: dùng `~/isaacsim/python.sh <script>.py` (interpreter bundled),
  KHÔNG dùng conda `isaac6` cho phần Isaac.
- Cần cài deps cho python.sh: `~/isaacsim/python.sh -m pip install qrcode pyzbar PyYAML Pillow`
  (zbar lib: dùng libzbar từ conda hoặc apt; xác nhận khi chạy Task 6).

## ✅ Binary launch THÀNH CÔNG (Task 2 pass)

- `~/isaacsim/python.sh scripts/verify_isaac.py` → `ISAAC_OK stage_prims= 12`, exit 0.
- Binary python: **3.12.13**. `isaacsim.util.debug_draw-3.2.2` có sẵn (extscache + extsInternal) — bản đầy đủ, không thiếu extension.
- Deps thuần Python đã cài vào binary python: `qrcode PyYAML Pillow pytest` (qua `~/isaacsim/python.sh -m pip install`).
- `pyzbar` cần libzbar: symlink libzbar (từ conda isaac6) vào `~/.local/isaac_extra_libs/`, đưa vào LD_LIBRARY_PATH.
- **Cách chạy chuẩn mọi script Isaac:** `scripts/run_isaac.sh <script.py | -m module>` (set EULA + LD_LIBRARY_PATH zbar + PYTHONPATH=repo root). Chạy từ repo root để `import sim.*` hoạt động.

## Assets nhà máy/kho (local mirror)

- Bộ asset đầy đủ (`isaac-sim-assets-complete` 5 phần) ~**75GB** → KHÔNG tải full (đĩa hạn chế).
- Tải **chọn lọc** từ S3 public root `https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0` vào local mirror `~/isaacsim_assets` (~**807MB**):
  - `Isaac/Environments/Simple_Warehouse/` (515MB) — có `full_warehouse.usd`, `warehouse_multiple_shelves.usd`, `warehouse.usd`, `warehouse_with_forklifts.usd`.
  - `Isaac/Props/`: Pallet, KLT_Bin (thùng bin), PackingTable (crate/box), Forklift (~370MB).
- Script tái lập: `scripts/download_assets.sh` (chỉnh mảng `PREFIXES` để thêm props).
- **Trỏ Isaac vào local mirror** trong script: `carb.settings.get_settings().set("/persistent/isaac/asset_root/default", "~/isaacsim_assets")` rồi reference `{root}/Isaac/Environments/Simple_Warehouse/full_warehouse.usd`.
- Verified `scripts/verify_assets.py`: `MISSING_COUNT=0` (kho tự chứa local), render ra kho thật (rack+hàng, xe nâng, cửa cuốn) → `sim/assets/warehouse_view.png`.
- `~/isaacsim_assets` KHÔNG commit vào git (binary, lớn) — tái tạo bằng `download_assets.sh`.

## Môi trường chung

- Python: bản pip 6.0.0 yêu cầu 3.12; bản binary tự mang interpreter riêng.
- GPU RTX 5060 Ti (Blackwell), driver 580.159.03.
- EULA: chạy lần đầu set `OMNI_KIT_ACCEPT_EULA=YES`.
- Lần chạy đầu Isaac Sim build shader cache → có thể 5-10 phút.

## Task 6 capture notes

### Vấn đề RTX texture headless

RTX renderer trong headless mode **không load được local file texture** cho OmniPBR MDL shader
trong vòng vài chục frame. Nguyên nhân:

- `OmniPBR.mdl` shader compilation diễn ra async — cần nhiều frame hơn;
- `file:///` URI đúng cú pháp nhưng RTX vẫn trả về black quad cho cả diffuse và emissive;
- Tested: `UsdPreviewSurface + UsdUVTexture`, `OmniPbrMaterial.set_input_values`, và
  `omni.kit.commands.CreateMdlMaterialPrim` + manual `CreateInput` — đều cho label màu đen.

### Giải pháp: render + composite

`scripts/verify_capture.py` dùng **render + 2D composite**:

1. Render cảnh 3D bình thường qua RTX (`rep.create.camera` với `look_at`, `rep.orchestrator.step`).
2. Tính 3D→2D projection giải tích từ tham số camera đã biết:
   - `focal_px = W * focal_length / horiz_aperture` (= 610.8 px cho 1280×720).
   - Chiếu 4 góc label quad xuống pixel coordinate.
3. Paste ảnh QR PNG (`sim/assets/labels/A1.png`) vào vùng pixel tương ứng.
4. Decode QR bằng pyzbar từ ảnh composite.

### Thông số camera A1

- Camera: `(0, -0.9, 0)`, `look_at=(0, -0.41, 0)`, Z-up.
- `focal_length=10`, `horizontal_aperture=20.955` → hFOV ≈ 72°.
- Label projected corners (1280×720): `(328,672)`, `(952,672)`, `(952,48)`, `(328,48)`.
- Label region in image: 624×624 px (center of frame, ~87% width fill).
- Kết quả: `CAPTURE_QR_OK part_no=PN-A01 qty=11` ✓

### Camera orientation math

USD camera looks down local **-Z**. Để nhìn theo +Y world từ vị trí `(0,-0.9,0)`:
```
RotateX(+90°) * (0,0,-1) = (0, 1, 0)  ← +Y world ✓
```
→ `cam.AddRotateXOp().Set(90.0)` trong `spawn_drone()`.

### render product naming

Dùng `rep.create.render_product(cam, res, name="DroneCapture")` để tạo
`/Render/OmniverseKit/HydraTextures/DroneCapture` — tránh reuse viewport mặc định
`/Render/OmniverseKit/HydraTextures/Replicator` (hay trả về None).

## Warehouse integration (M1 Task 7)

### Warehouse kind used

`warehouse.usd` — open shell (floor + walls + lights, no built-in shelving).
Chosen so our 3×6 rack can sit on open floor at the origin without clipping into
any built-in geometry.

### RACK_WORLD_OFFSET

`RACK_WORLD_OFFSET = (0.0, 0.0, 0.0)` in `sim/config.py` — no translation needed
because `warehouse.usd` has open floor at origin.  To reposition the rack (e.g. to
avoid collision with future props), edit `RACK_WORLD_OFFSET` in `sim/config.py`;
`scene_builder.py`, `verify_warehouse_scene.py`, and `verify_capture.py` all read it
and apply it to the `/World/Rack` Xform and the drone/camera spawn position.

### How to toggle

| Mode | Command |
|------|---------|
| Real warehouse backdrop (default) | `scripts/run_isaac.sh -m sim.scene_builder` |
| Real warehouse + GUI window | `scripts/run_isaac.sh -m sim.scene_builder -- --gui` |
| Primitive ground (offline/CI) | `scripts/run_isaac.sh -m sim.scene_builder -- --no-warehouse` |

### Verification results

- `SCENE_OVERVIEW_OK mean=152.2` — oblique camera shows all 18 labeled bins standing
  inside warehouse walls/floor (visual confirmed: `sim/assets/scene_overview.png`).
- `CAPTURE_QR_OK part_no=PN-A01 qty=11` — genuine RTX render of A1 label with
  warehouse background visible (confirmed: `sim/assets/capture_A1.png`).
- Pure-Python tests: 7 passed.

## Rack shelving + boxes

Rebuilt `sim/rack.py build_rack()` to produce a realistic multi-level shelving rack.

**Structure (from primitives):**
- 4 vertical steel posts (gray UsdGeom.Cube beams, 0.08 x 0.08 m cross-section, full rack height 4.8 m) at the 4 corners of the footprint (x: -0.5 to 2.9 m, y: -0.45 to 0.45 m).
- 6 horizontal shelf boards (thin gray slabs, 3.4 m wide x 0.90 m deep x 0.05 m thick) with their TOP surface at z = (L-1)*0.8 m for each level L — exactly matching the bin grid origin so boxes rest flush.

**Box asset used:**
`~/isaacsim_assets/Isaac/Environments/Simple_Warehouse/Props/SM_CardBoxC_01_1052.usd`
(fallback: SM_CardBoxA_01_301.usd). Referenced as USD payload into each Bin Xform.

**Box size and placement:**
- Scale: 0.5 x 0.5 x 0.4 m (W x D x H).
- Box center z = (L-1)*0.8 + 0.2 m (box bottom sits on shelf top = bin reference z).
- Box center x = ci * 1.2 m (same column as bin reference), y = 0 m.
- Bin reference coordinates (pallet_pose) UNCHANGED — navigation and perception unaffected.

**Label offset:**
GR-label quad kept at (px, py-0.41, pz) — same world position as before.  
pz = (L-1)*0.8 is the shelf-top / bin reference z; the label appears on the lower front face of the box (slightly in front at y=-0.41).  
verify_capture.py LABEL_CENTER = (0,-0.41,0) unchanged; CAPTURE_QR_OK confirmed.

**Verification results:**
- `SCENE_OVERVIEW_OK mean=158.1` — oblique view shows 18 labeled boxes resting on shelves inside warehouse.
- `CAPTURE_QR_OK part_no=PN-A01 qty=11` — genuine RTX render, QR decoded from real pixels.
- Pure-Python tests: 7 passed (unchanged).

## Rack v2 (box+label sizing)

Addressed three visual complaints: boxes too small/floating, label too large (poster), boxes not big enough.

**Final constants (sim/config.py):**
- `BOX_W = 0.70 m`, `BOX_D = 0.50 m`, `BOX_H = 0.50 m` — big cardboard carton, occupies most of the 0.8 m shelf cell.
- `LABEL_W = 0.28 m`, `LABEL_H = 0.20 m` — small shipping-label sticker on front face.
- `SCAN_STANDOFF = 0.75 m` — drone hovers 0.75 m in front of the box face (total scan_pose.y = -1.0 m from rack origin).

**Box placement formula:**
- Shelf top surface z = `(L-1) * LEVEL_HEIGHT` (same as pallet_pose.z, bin reference — UNCHANGED).
- Box center z = `shelf_top_z + BOX_H/2` (box BOTTOM rests flush on shelf).
- Box center y = 0; front face at `y = -BOX_D/2 = -0.25 m`.

**Label placement formula:**
- Label center z = `box_center_z` (vertically centered on carton).
- Label center y = `-BOX_D/2 - 0.005 = -0.255 m` (5 mm proud of box face, avoids z-fighting).
- Label is a rectangular quad (XZ plane in local space, normal -Y toward drone).

**scan_pose formula (bin_map.py):**
- `scan_pose.y = -(BOX_D/2 + SCAN_STANDOFF) = -1.0 m`
- `scan_pose.z = box_center_z = (L-1)*0.8 + BOX_H/2` (camera height tracks label center).

**Verification results:**
- `SCENE_OVERVIEW_OK mean=153.7` — oblique view shows 18 big cartons sitting ON each shelf level, each with a small white label sticker on its front face (no giant floating sheets, no floating boxes).
- `CAPTURE_QR_OK part_no=PN-A01 qty=11` — genuine RTX render; small label (0.28×0.20 m) at (0, -0.255, 0.25); camera at (0, -1.0, 0.25); QR decoded from real pixels.
- Pure-Python tests: 7 passed.

## M2 physics API

### RigidPrim — Isaac Sim 6.0 (confirmed against binary 6.0.0)

`RigidPrim` in Isaac Sim 6.0 is a **batched view** (not a per-prim handle).
All getter/setter methods use plural names and return/accept arrays.

**Constructor:**
```python
from isaacsim.core.prims import RigidPrim
rb = RigidPrim("/World/Drone/Body", name="drone_body")
# Must be called after World.reset() to initialize the physics handle.
```

**Pose / velocity (read):**
```python
positions, orientations = rb.get_world_poses()
# positions:    np.ndarray shape (N, 3)
# orientations: np.ndarray shape (N, 4)  — (w, x, y, z) quaternion

velocities = rb.get_linear_velocities()
# velocities: np.ndarray shape (N, 3)

# For a single drone, extract index 0:
pos  = np.asarray(positions[0],    dtype=float).reshape(3)
quat = np.asarray(orientations[0], dtype=float).reshape(4)
vel  = np.asarray(velocities[0],   dtype=float).reshape(3)
```

**Force application (per physics step):**
```python
rb.apply_forces(force_Nx3, is_global=True)
# force_Nx3: np.ndarray shape (N, 3) — world-space Newtons at COM
# For a single drone:
rb.apply_forces(force_3d.reshape(1, 3), is_global=True)
```

`apply_forces_and_torques_at_pos(forces, torques, positions, is_global=True)`
is also available for multi-point force application (confirmed signature).

**Note:** there is NO singular `get_world_pose()` or `get_linear_velocity()` —
these method names do not exist in Isaac Sim 6.0.  Always use the plural forms.

### Control strategy (M2 POC)

**Full 3D net force** is applied each physics step — NOT altitude-only.
`position_to_thrust(pos, vel, target, mass=1.0, g=9.81, kp=4.0, kd=6.0)`
returns a gravity-compensated 3D world-space force vector (N) that is applied
directly at the body COM via `apply_forces`. This moves the drone in X, Y and Z
toward the target waypoint simultaneously — genuine rigid-body dynamics.

**Gains used (tuned for dt=1/60 s, DRONE_MASS=1.0 kg):**
- `kp = 4.0` (position P gain)
- `kd = 6.0` (velocity D gain, provides critical damping)

These gains give near-zero steady-state error with no oscillation at the hover
target.  The higher kd (relative to kp) was necessary to damp the initial
velocity overshoot when the drone starts from rest with a large position error.

**Verified results:**
- `HOVER_OK final_pos=[0. 0. 1.5] err=0.000` (10 s hover at (0,0,1.5), started at (0,0,1))
- `FLY_DONE bin=B3 reached_home=True final_state=IDLE` (home→approach→scan→approach→home)
- Pure-Python tests: 16 passed (unchanged).

**Flight state machine (drone/flight.py):**
- `ARRIVE_TOL = 0.20 m` — waypoint arrival threshold
- `SCAN_HOLD_STEPS = 120` — hold at scan pose for 2 s (120 steps at 60 Hz)
- `max_steps = 6000` — safety cap (~100 s)

## M3 perception notes

### Semantics API (Task 3 — Isaac 6.0)

`add_update_semantics` does NOT exist in Isaac 6.0.
Use `from isaacsim.core.utils.semantics import add_labels`:
```python
add_labels(prim, ["label"], instance_name="class")
# applies SemanticsLabelsAPI:class schema
```
Equivalent: `UsdSemantics.LabelsAPI.Apply(prim, "class").CreateLabelsAttr().Set(["label"])`

### bbox_2d_tight annotator (Task 3)

The annotator MUST be created with `init_params={'semanticTypes': ['class']}`,
otherwise `idToLabels` stays empty and the data array has shape (0,):
```python
bbox = rep.AnnotatorRegistry.get_annotator(
    "bounding_box_2d_tight",
    init_params={"semanticTypes": ["class"]}
)
```
`bbox.get_data()` returns:
```python
{
  "data": np.ndarray(dtype=[('semanticId','u4'),('x_min','i4'),('y_min','i4'),
                             ('x_max','i4'),('y_max','i4'),('occlusionRatio','f4')]),
  "info": {"idToLabels": {"0": {"class": "label"}}, "bboxIds": [...], "primPaths": [...]}
}
```
Field access: `row["x_min"]`, `row["y_min"]`, `row["x_max"]`, `row["y_max"]` (structured array).

### Dataset results

- 144 images (18 bins × 8 jittered poses), 105 non-empty label files (39 empty = bin off-screen)
- `DATASET_OK images=144`

### PaddleOCR install (Task 4)

Working combination: `paddleocr==2.7.3` + `paddlepaddle==2.6.2` + `numpy<2` (1.26.4)

**Why not newer versions:**
- `paddlepaddle==3.x` fails with `NotImplementedError: ConvertPirAttribute2RuntimeAttribute`
  at `onednn_instruction.cc:116` — Intel OneDNN backend incompatibility on this machine
- `paddleocr==3.6.0` + `paddlepaddle==2.6.2` → `AttributeError: set_optimization_level`
- `paddleocr==2.7.3` + `paddlepaddle==3.x` → numpy ABI mismatch (`0x1000009 vs 0x2000000`)

**HOME override:** `~/.paddleocr` is root-owned on this machine. LabelOCR sets
`os.environ["HOME"] = "/tmp/paddle_home"` to allow model download.

**PaddleOCR result structure (v2.7.3):**
```python
result = ocr.ocr(img, cls=True)
# result: list-of-pages (len=1); each page: list of blocks or None
# Each block: [[[x,y],...4 corners], (text_str, conf_float)]
# Access: block[1][0] → text, block[1][1] → confidence
```

### YOLO training results (Task 5)

- Model: YOLOv8n (pretrained)
- Dataset: 144 synthetic images, 1 class (label)
- Epochs: 40, imgsz: 960, batch: 16, device: RTX 5060 Ti
- **mAP50 = 0.9948, mAP50-95 = 0.9748** (excellent for single-class synthetic)
- Training time: < 5 minutes

### End-to-end verify (Task 6)

`conda run -n perception python scripts/verify_perception.py` on `capture_A1.png`:
- YOLO detected 6 label regions (conf range 0.12–0.94)
- Best detection (conf=0.943) decoded QR directly → `PERCEPTION_OK source=qr part_no=PN-A01 qty=11 conf=0.94`
- fuse() whole-image QR fallback also tested (works even with 0 detections)

## Drone model (CAD->USD)

### Source
**Parrot ANAFI Ai** — genuine product CAD, ASCII STEP AP214.
- Source file: `ANAFI_Ai.step_/ANAFI Ai_SIMPLIFIED_20211213.STEP` (469 MB, not committed to git)
- Verified authentic: white body + green accent trim + "Parrot" branding + 4 propeller arms + front gimbal camera.

### Converter
`omni.kit.converter.hoops_core` (HOOPS Exchange SDK) via the `omni.services.convert.cad` subprocess approach:
- Launched as a separate `kit` subprocess with `--enable omni.kit.converter.hoops_core --exec hoops_main.py`
- Config: `use_meter_as_world_unit=True` (but STEP exports in mm; native bbox = 307×500×134 mm)
- Conversion time: ~60 seconds. Script: `scripts/convert_drone_cad.py`
- **Note:** `omni.kit.asset_converter.get_instance()` does NOT support STEP (only FBX/OBJ/glTF). Must use hoops_core subprocess.

### Output USD
`~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd` — **36.3 MB**, 425 prims, lives outside repo.
- Rebuild command: `scripts/run_isaac.sh scripts/convert_drone_cad.py`

### Scale
- Native bbox: 307.4 × 500.4 × 134.0 mm (STEP units are mm even with use_meter_as_world_unit=True)
- Target wingspan: 0.45 m → scale factor applied = 0.45 / 500.4 = 0.000899
  (max_dim is fuselage length ~500mm; yields ~276mm wingspan × 0.000899 = 0.277m wide, 0.45m long)
- Recentered via bbox so visual center lands at spawn position.

### How spawn_drone references it
`sim/drone_asset.py spawn_drone()` pattern (proxy + visual child):
1. Creates a 1mm invisible physics proxy cube at `/World/Drone/Body` — `add_physics()` attaches RigidBodyAPI/MassAPI here; physics stays light.
2. Adds `/World/Drone/Body/Visual/Mesh` as a USD Reference to `anafi_ai.usd`.
3. Measures native bbox via `UsdGeom.BBoxCache`, computes uniform scale + recenter translate; applies to the `/World/Drone/Body/Visual` Xform.
4. Fallback: if USD missing/too small, builds a primitive quadrotor (body + 4 arms + 4 rotor discs + gimbal).
5. Camera at `/World/Drone/Camera` placed slightly forward of body center, `RotateX(+90°)` to aim along +Y toward rack labels.

### Render confirmed
`sim/assets/drone_cad_render.png` — visibly the Parrot ANAFI Ai: white/green body, "Parrot" logo, 4 propeller blades, front camera gimbal.

### Verification results
- `SCENE_OVERVIEW_OK mean=143.8` — drone + rack + warehouse scene renders correctly.
- `CAPTURE_QR_OK part_no=PN-A01 qty=11` — scan pipeline unaffected (physics proxy body unchanged).
- `FLY_DONE bin=B3 reached_home=True` — flight controller / RigidPrim physics unaffected.
- `51 passed` — all pure-Python tests green.

## Narrow aisle + obstacle avoidance

### Layout

- **Primary rack** (existing): 3×6 shelving, boxes face -Y, QR labels at y≈-0.255.
- **Second (mirror) rack**: same 3×6 structure, primitive brown boxes face +Y, centers at y=SECOND_RACK_Y=-1.80 m. Visual-only (no CollisionAPI — APF handles avoidance).
- **Aisle width**: 1.30 m (AISLE_CENTER_Y=-0.90 m, between rack faces at y=-0.45 and y=-1.35).
- **Aisle floor obstacle**: stacked brown box cube at (1.2, -0.9, 0.3 m center) ≈ 0.7×0.7×0.6 m near column B.
- All constants in `sim/config.py`: AISLE_WIDTH, SECOND_RACK_Y, AISLE_CENTER_Y, OBSTACLE_X/Y/Z, DRONE_RADIUS.

### Obstacle AABB design

`sim/obstacles.py` provides two AABB sets:
- `get_obstacle_aabbs()`: APF force computation. Uses face-slab representation (extended to x=-5) to avoid symmetric APF corner trap at aisle entrance.
- `get_clearance_aabbs()`: collision-detection metric. Slabs only within rack X span [-0.5..2.9].
  
Face slabs are 0.20 m thick, centred at the actual rack aisle faces (y=-0.45 primary, y=-1.35 secondary).

### APF parameters (drone/flight.py)

- `APF_INFLUENCE = 0.30 m` — only activates when drone is within 0.30 m of obstacle surface.
- `APF_GAIN = 1.0` — gentle far from surfaces, strong near contact (1/d² divergence).
- `DRONE_SAFETY_R = 0.25 m` — drone radius subtracted from distance.
- APF forces applied ONLY to the floor obstacle (index 2); rack wall APF causes equilibria when aisle-traversing in +X.

### Waypoint route (drone/waypoints.py)

```
home(-2,-2,1) → pre_entry(-2,-0.9,1.2) → aisle_entry(-1,-0.9,1.2)
  → aisle_col(col_x,-0.9,1.2) → scan(bin scan_pose)
  → aisle_col(col_x,-0.9,1.2) → aisle_exit(3.4,-0.9,1.2)
  → post_exit(3.4,-2,1.2) → home
```

Key design decisions:
1. `pre_entry` at HOME_X moves drone to aisle Y first (avoids APF symmetric trap at x=-0.5 corners).
2. `post_exit` at HOME_Y before flying home (prevents clearance violation on diagonal return path).
3. `AISLE_CRUISE_Z = 1.2 m` — above 0.6 m floor obstacle.

### AISLE_DEMO metrics achieved

```
AISLE_DEMO bin=C3 reached=True min_clearance=0.2564 collisions=0
```

- `reached=True` — drone flew through narrow aisle, scanned C3, returned home.
- `min_clearance=0.2564 m` — 25.6 cm minimum distance to any real obstacle surface (never penetrated).
- `collisions=0` — zero steps with clearance ≤ 0.

### Verification results

- `BUILT bins=18` — scene_builder builds both racks + aisle obstacle.
- `CAPTURE_QR_OK part_no=PN-A01 qty=11` — QR capture unaffected by second rack.
- `73 passed` — all pure-Python tests green (new: test_obstacles.py + test_avoidance.py + test_waypoints.py updated).
- Render: `sim/assets/aisle_overview.png` — oblique view showing primary rack with labeled boxes and corridor.
- Render: `sim/assets/aisle_topdown.png` — top-down view showing both racks, narrow corridor, obstacle block in aisle.

## M3 update: pre-trained QR detector (no training)

Per user: KHONG tu train YOLO. Dung model YOLOv8 QR detect co san: pip `qrdet`
(tu tai weights). `perception/detector.py QRCodeDetector` thay cho LabelDetector.
Bo `scripts/train_yolo.py`, `scripts/gen_perception_dataset.py`, `data/perception`,
`perception/weights`. verify_perception.py: detect qrdet -> pyzbar decode -> PaddleOCR
fallback. Real result: detections=1 conf=0.91, PERCEPTION_OK part_no=PN-A01 qty=11.

## Live WebSocket camera stream (live_sim.py)

`scripts/live_sim.py` — long-running Isaac Sim 6.0 interactive demo.

Builds the full warehouse POC scene (warehouse.usd backdrop + primary 3×6 rack + second mirror rack + aisle floor obstacle + ANAFI drone). Runs **continuously** (no timeout by default) and exposes a **WebSocket server** on port 8765.

### Prereq

```bash
~/isaacsim/python.sh -m pip install websockets
```

### How to run (continuously)

```bash
# GUI mode (opens Isaac window):
scripts/run_isaac.sh scripts/live_sim.py

# Headless (background, runs until Ctrl-C or SIGTERM):
scripts/run_isaac.sh scripts/live_sim.py -- --headless &

# Custom WS port:
scripts/run_isaac.sh scripts/live_sim.py -- --headless --ws-port 8765

# Finite duration for testing:
scripts/run_isaac.sh scripts/live_sim.py -- --headless --duration 90

# Also enable legacy UDP/ffmpeg output (optional):
scripts/run_isaac.sh scripts/live_sim.py -- --headless --udp
```

Wait ~60–90 s for Isaac boot + shader warmup. The script prints:

```
LIVE_SIM ws://0.0.0.0:8765  (connect: ws://localhost:8765)
```

This is the "ready" line to poll for before connecting clients.

### WebSocket message schema

**Server → client** (JSON):

| Message type | Shape |
|---|---|
| `frame` | `{"type":"frame","jpeg":"<base64 JPEG>","w":960,"h":540,"ts":<float>}` — annotated camera frame, ~10 fps, JPEG quality 70, 960×540 |
| `telemetry` | `{"type":"telemetry","pos":[x,y,z],"target":<bin_id or null>,"state":"IDLE\|FLYING\|SCANNING\|RETURNING","clearance":<float>,"detections":[{"part_no":...,"qty":...,"bbox":[x1,y1,x2,y2]}]}` — ~5 Hz |
| `inspection` | `{"type":"inspection","bin_id":...,"scanned_part":...,"scanned_qty":...,"system_part":...,"system_qty":...,"match":<bool>,"status":"completed"\|"discrepancy","ts":<float>}` — emitted once per commanded inspection on scan completion |

**Client → server** (JSON):

| Command | Shape |
|---|---|
| Inspect bin | `{"type":"cmd","action":"inspect","bin_id":"B3"}` |
| Return home | `{"type":"cmd","action":"home"}` |

### Drone behaviour on `inspect` command

1. State → `FLYING`: drone interpolates kinematically through `plan_waypoints(bin_id)` (home → pre-entry → aisle centreline → column X → scan pose), visibly threading the narrow aisle.
2. State → `SCANNING`: drone holds at scan pose for ~1.5 s, runs pyzbar on the rendered frame.
3. Emits `inspection` message with scanned vs SAP data, `match` flag, `status`.
4. State → `RETURNING`: drone retraces aisle path to home. State → `IDLE`.
5. Telemetry (`target`, `state`, `pos`) updates in real time throughout.
6. `IDLE` state: camera serpentine-sweeps all 18 bins so the feed is always live.

SAP mock is seeded on startup (`seed_from_bin_map()`). Discrepancy bins: **B2** (qty mismatch), **C4** (wrong part_no), **A5** (qty off).

### Minimal Python client example

```python
import asyncio, websockets, json, base64

async def demo():
    async with websockets.connect("ws://localhost:8765") as ws:
        # Read a few messages then send inspect B2
        for _ in range(5):
            msg = json.loads(await ws.recv())
            print(msg["type"], msg.get("state",""), msg.get("ts",""))
        await ws.send(json.dumps({"type":"cmd","action":"inspect","bin_id":"B2"}))
        async for raw in ws:
            msg = json.loads(raw)
            if msg["type"] == "inspection":
                print("INSPECTION:", msg)
                break

asyncio.run(demo())
```

### Legacy UDP output (optional `--udp` flag)

Passing `--udp` also pipes raw frames to ffmpeg → `udp://127.0.0.1:5600` (H.264/mpegts, nvenc preferred). View with `ffplay udp://127.0.0.1:5600`.

### FPS / performance

- WS broadcast: ~10 fps (frames), ~5 Hz (telemetry).
- Isaac render: ~8–9 fps at 1280×720, `rt_subframes=4` (RTX 5060 Ti headless).
- JPEG encode: 960×540 @ quality 70 ≈ 25–40 KB per frame.

### Onboard camera: rep.create.camera() — CRITICAL (not UsdGeom.Camera)

**Root cause (Isaac Sim 6.0):** A raw `UsdGeom.Camera` prim attached to a render product via `rep.create.render_product(path, ...)` does NOT render emissive label materials in headless RTX mode — the label quad texture stays invisible (black) on the box face even after 120+ warmup frames.

**Fix:** Use `rep.create.camera()` (Replicator-managed camera prim). This matches the verified `verify_capture.py` approach and IS correctly registered with the RTX material/texture pipeline.

Proven by `/tmp/debug_scan_render.py` comparing both approaches in the same scene:
```
UsdGeom.Camera attach → QR detections: 0   (label texture invisible)
rep.create.camera    → QR: 1 ['{"part_no": "PN-A01", "qty": 11}']
```

**Update camera pose per-frame** with:
```python
with _rep_cam:
    rep.modify.pose(position=cam_pos, look_at=look_at, look_at_up_axis=(0.0, 0.0, 1.0))
```

### Onboard scan-pose framing

All camera views (IDLE sweep, FLYING interpolation, SCANNING) are locked to **bin scan poses** — never a third-person view.

Scan-pose geometry (same as `verify_capture.py`):
- Camera at `(col_x, -(BOX_D/2 + SCAN_STANDOFF), label_z)` = `(col_x, -1.0, (level-1)*0.8 + 0.25)`
- Look-at at `(col_x, -(BOX_D/2) - 0.005, label_z)` = `(col_x, -0.255, label_z)`
- `up = (0, 0, 1)` — no tilt
- Camera settings: `focal_length=10`, `horizontal_aperture=20.955` → hFOV ≈ 72°, near-clip=0.01

IDLE state: smoothstep pan between consecutive bin scan poses (full 18-bin cycle in 30 s).

FLYING / RETURNING: camera interpolates (smoothstep) between current sweep position and target bin scan pose, progressing as `_wp_idx` advances along the waypoint route.

### High-subframe scan decode

QR decode requires multiple high-quality RTX frames at the new camera position before textures converge. Pattern:

```python
_SCAN_WARMUP_FRAMES = 8   # hold at pose, rendering 4×rt_subframes=64 per frame
_scan_warmup_left = 0     # countdown

# In main loop:
if _drone_state == "SCANNING" and _scan_warmup_left > 0:
    for _ in range(4):
        rep.orchestrator.step(rt_subframes=64, wait_for_render=True)
elif _drone_state == "SCANNING":
    rep.orchestrator.step(rt_subframes=48, wait_for_render=True)
else:
    rep.orchestrator.step(rt_subframes=6, wait_for_render=True)

# If still no decode after warmup, retry at higher quality:
for _retry_sf in (64, 96):
    rep.orchestrator.step(rt_subframes=_retry_sf, wait_for_render=True)
    # re-read and attempt pyzbar decode
```

### Drone mesh behind camera

The ANAFI drone mesh (`/World/Drone`) is repositioned every frame so it stays **behind** the onboard camera and never enters the frame:

```python
_dm_t_op.Set(Gf.Vec3d(
    float(cam_pos[0]),
    float(cam_pos[1]) - 0.5,   # further from rack (-Y), behind camera
    float(cam_pos[2]) - 0.15,  # slightly below camera
))
```

Camera looks in +Y direction; drone mesh is placed at `cam_y - 0.5` so it is always 0.5 m behind the camera focal point.

### Known issues

- `SdRenderVarPtr missing valid input renderVar LdrColorhost` — harmless warning, frames still render.
- QR labels appear after ~20–30 warmup frames (async RTX texture load); pyzbar decodes correctly once visible.

---

## UDP→MJPEG browser bridge

Bridges the live Isaac drone-camera UDP stream (`udp://127.0.0.1:5600`) into
a browser-displayable `multipart/x-mixed-replace` MJPEG stream so any `<img>`
tag can show the live view with no plugins.

### New endpoints (backend/app.py)

| Endpoint | Description |
|---|---|
| `GET /api/live_stream` | Streams `multipart/x-mixed-replace; boundary=ffmpeg` (MJPEG). Drop into `<img src="/api/live_stream">`. |
| `GET /api/live_status` | Returns `{"streaming": true}` if the UDP source is active, `{"streaming": false}` otherwise. |

### How it relates to live_sim.py

`scripts/live_sim.py` renders the Isaac Sim drone-camera frames in real time and
pipes them to ffmpeg, which encodes H.264/mpegts and sends UDP datagrams to
`udp://127.0.0.1:5600`.  The backend bridge (`/api/live_stream`) spawns a second
ffmpeg subprocess that reads that UDP stream and re-encodes to multipart JPEG,
forwarding the output as a `StreamingResponse` to the browser client.

```
Isaac Sim → ffmpeg (h264_nvenc/libx264) → UDP:5600
                                              ↓
                          Backend /api/live_stream
                          ffmpeg (mpjpeg decoder) → HTTP multipart/x-mixed-replace
                                              ↓
                          Browser <img src="/api/live_stream">
```

### ffmpeg bridge command

```bash
ffmpeg \
  -loglevel warning \
  -analyzeduration 2000000 -probesize 2000000 \
  -fflags nobuffer -flags low_delay \
  -i "udp://127.0.0.1:5600?fifo_size=1000000&overrun_nonfatal=1&timeout=10000000" \
  -f mpjpeg -q:v 5 -r 15 \
  -
```

`-analyzeduration 2000000 -probesize 2000000` is required: without it the H.264
decoder fails with `non-existing PPS referenced` on mid-stream UDP connect.
ffmpeg buffers 2 s of stream data to locate an IDR keyframe before decoding.

### Configuration

```bash
LIVE_UDP_HOST=127.0.0.1   # default
LIVE_UDP_PORT=5600         # default
```

Override via environment variables before starting uvicorn.

### Startup sequence

```bash
# 1. Start Isaac Sim (takes ~20 s to announce streaming)
scripts/run_isaac.sh scripts/live_sim.py -- --headless --duration 1800 &

# 2. Start backend (perception env)
conda run -n perception python -m uvicorn backend.app:app --port 8080

# 3. Check stream is live (takes ~10 s to probe)
curl -s localhost:8080/api/live_status   # → {"streaming": true}

# 4. View in browser — or pull raw MJPEG to verify frames
curl -s -m 6 localhost:8080/api/live_stream -o /tmp/mjpeg_dump.bin
python3 -c "d=open('/tmp/mjpeg_dump.bin','rb').read(); print('jpeg_frames=',d.count(b'\\xff\\xd8\\xff'),'bytes=',len(d))"
```

### Verification (confirmed)

- `/api/live_status` → `{"streaming": true}` while `live_sim.py` is running.
- `/api/live_stream` pulled 5 MB in 30 s, containing **114 JPEG frames**.
- Extracted frame (51 671 bytes, 1280×720): shows the live Isaac warehouse scene —
  cardboard pallet face (front of BIN), orange metal rack uprights, metal shelf
  brackets, warehouse walls; HUD reads:
  `DRONE AI CAMERA - LIVE | frame=5900 | t=855.4s | pos=(2.40,-1.00,4.25) | QR dets=0`
- `87 passed` — all pure-Python tests green (+2 new `test_udp_bridge.py` tests
  vs. previous `85 passed`).

### Concerns / latency notes

- **First-byte latency** is ~12–14 s (2 s analyzeduration + IDR hunt in 7 fps stream).
  After the first frame, subsequent frames arrive at the requested 15 fps cadence.
- **When live_sim is not running**: `/api/live_stream` returns `503 Service Unavailable`
  with `{"error": "live_sim not running", "hint": "start scripts/live_sim.py first"}`
  after a 15 s timeout. `/api/live_status` returns `{"streaming": false}`.
- **Client disconnect**: the backend kills the ffmpeg subprocess immediately in the
  generator `finally` block, so no zombie processes accumulate.
- **UDP keyframe gap**: if the client reconnects before the next IDR keyframe, the
  first few frames may be garbled; the browser renders them as partial images.
  Increasing the `-g` GOP size in `live_sim.py` (currently `fps` frames ≈ 1 s) would
  help; this is a known limitation of live H.264/UDP bridging.

## Operator dashboard (live cam WS + 3D map)

### Panels

| Panel | Description |
|---|---|
| **Live Camera** | `<img>` element whose `src` is set to `"data:image/jpeg;base64," + msg.jpeg` on every `frame` message from ws://localhost:8765. Shows the drone's onboard RTX camera with QR-box + HUD overlay rendered server-side. Displays state/pos/clearance in a telemetry HUD strip above the frame and a detections strip below. Red "LIVE" indicator pulses when frames arrive; "waiting for Isaac live_sim…" placeholder until first frame. |
| **3D Map / Digital Twin** | Three.js (CDN `three@0.160.0`). Fetches `/api/scene` once on load: builds floor grid, primary rack (blue), second rack (indigo), floor obstacle (amber), 18 bin boxes coloured by inspection status (gray→scanning→green/red), bin ID labels, and a cone drone marker with a fading trail. OrbitControls for rotate/zoom. Drone position updates from `telemetry.pos`; bin colours update from `inspection` messages. Isaac Z-up coords used directly (camera.up=(0,0,1)). |
| **BIN Grid 3×6** | Columns A/B/C × levels 1–6. Click a cell → sends `{"type":"cmd","action":"inspect","bin_id":…}` to WS 8765 (drives the real drone). Disabled for Viewer role. "Inspect All" sequences through all 18 bins (4 s spacing when Isaac is connected). "Return Home" sends `{"type":"cmd","action":"home"}`. |
| **Alert banner** | Dismissible red banner on `inspection` with `status:"discrepancy"`. Auto-dismiss after 30 s. |
| **History table** | Appends each `inspection` message (bin, time, scanned vs system, status badge). Newest first, capped at 200 rows. |

### WebSocket schema consumed (ws://localhost:8765)

| Type | UI action |
|---|---|
| `frame` | Sets `img.src = "data:image/jpeg;base64," + msg.jpeg` (live camera) |
| `telemetry` | Updates HUD (state/pos/target/clearance), moves 3D drone marker + trail, marks targeted bin scanning |
| `inspection` | Updates bin cell colour, 3D bin colour, history table, alert if discrepancy |

Client→server commands sent: `{"type":"cmd","action":"inspect","bin_id":"B3"}` and `{"type":"cmd","action":"home"}`.

### Backend endpoint: GET /api/scene

Returns the full 3D layout JSON: bins (18 × id/col/level/world_pos/box_size/part_no/qty), primary_rack, second_rack, obstacle, aisle geometry. Reads `sim/bin_map.yaml` directly (no Isaac import). Example snippet:

```json
{
  "bins": [{"bin_id":"A1","column":"A","level":1,"world_pos":[0.0,0.0,0.25],"box_size":[0.7,0.5,0.5],"part_no":"PN-A01","qty":11}, ...],
  "primary_rack": {"x_min":-0.5,"x_max":2.9,"y_min":-0.45,"y_max":0.45,"z_min":0.0,"z_max":4.8, ...},
  "second_rack":  {"x_min":-0.5,"x_max":2.9,"y_min":-2.25,"y_max":-1.35, ...},
  "obstacle":     {"center":[1.2,-0.9,0.3],"half_extents":[0.35,0.35,0.30], ...},
  "aisle":        {"center_y":-0.9,"width":1.3, ...}
}
```

### Connection indicators (header)

Two dots: **Backend** (ws /ws, port 8080) and **Isaac** (ws://localhost:8765). Each shows green/amber/red with reconnect label.

### Run order

```bash
# 1. One-command live demo (starts both Isaac + backend):
scripts/run_live_demo.sh
# Isaac boots in ~60-90 s; open http://localhost:8080 immediately — UI shows "waiting…"

# OR start manually:

# Terminal 1 — Isaac Sim (takes ~60-90 s to boot):
scripts/run_isaac.sh scripts/live_sim.py -- --headless

# Terminal 2 — Backend + UI (perception env):
conda run -n perception python -m uvicorn backend.app:app --host 0.0.0.0 --port 8080

# 2. Open browser:
#    http://localhost:8080/
#    Camera panel: "waiting for Isaac live_sim…" until WS 8765 ready
#    Once Isaac ready: live drone feed appears, 3D map loads, bin grid is clickable
```

### Files changed

- `ui/index.html` — full dashboard layout (live cam + 3D map + bin grid + history)
- `ui/app.js`    — WS 8765 (frame/telemetry/inspection), Three.js scene, bin-click commands
- `ui/style.css` — dark-theme dashboard styles, dual connection dots, tele HUD
- `scripts/run_live_demo.sh` — starts both Isaac live_sim (background) + backend
- `backend/app.py` — `/api/scene` endpoint (already present, unchanged)

## RTX LiDAR + point cloud

### LiDAR config used

**Example_Rotary** — built-in NVIDIA rotary lidar config (Isaac Sim 6.0 bundled).
USD asset: `~/isaacsim_assets/Isaac/Sensors/NVIDIA/Example_Rotary.usda`
(not present in this deployment's local asset mirror — see creation path below).

### API / annotator used (Isaac Sim 6.0 confirmed)

```python
import omni.replicator.core as rep
import omni.kit.commands
from pxr import Gf

# 1. Create OmniLidar prim under /World/Drone (Xform — must NOT be a gprim parent)
#    Note: /World/Drone/Body is a UsdGeom.Cube (gprim); OmniLidar cannot be parented there.
#    Use rep.functional.create.omni_lidar when Example_Rotary.usda is not in local assets.
_lidar_rep_item = rep.functional.create.omni_lidar(
    position=(0, 0, 0),
    rotation=(0, 0, 0),
    name="Lidar",
    parent="/World/Drone",
)
_sensor_prim_path = str(_lidar_rep_item[0].GetPath())  # "/World/Drone/Lidar"

# 2. Attach Replicator render product + annotator
_lidar_rp    = rep.create.render_product(_sensor_prim_path, [1, 1])
_lidar_annot = rep.AnnotatorRegistry.get_annotator("IsaacCreateRTXLidarScanBuffer")
_lidar_annot.attach([_lidar_rp])

# 3. Per-frame read (inside main loop, every 6th frame)
_data = _lidar_annot.get_data()   # dict
pts   = _data["data"]              # np.ndarray shape (N, 3) — sensor-frame XYZ float32
```

### get_data() keys found at runtime (Isaac Sim 6.0)

```
keys = ['azimuth', 'channelId', 'data', 'distance', 'echoId', 'elevation',
        'emitterId', 'hitNormal', 'intensity', 'materialId', 'objectId',
        'radialVelocityMS', 'tickId', 'tickState', 'timestamp', 'velocity', 'info']

shape of 'data' key: (N, 3) — Nx3 float32 XYZ in sensor frame
all other numeric keys: (N,) arrays (empty if no hit or not produced by this config)
```

Point cloud key is `"data"` (not `"point_cloud"` or similar). N ≈ 18 000 raw
per sweep; downsampled to ≤ 1500 via stride.

### WS `lidar` message schema

```json
{
  "type": "lidar",
  "points": [[x, y, z], ...],
  "frame": "world",
  "n": <count>
}
```

Points are rounded to 2 decimal places (cm precision), downsampled to ≤ 1500/sweep.
`lidar_min` (nearest non-zero point distance) is injected into `telemetry`.

### Annotator name (Isaac Sim 6.0)

Registered name on this machine: **`IsaacCreateRTXLidarScanBuffer`**
(not `RtxSensorCpuIsaacCreateRTXLidarScanBuffer` — that name is from 4.5 docs and
is NOT registered in 6.0). Isaac prints a deprecation warning advising to use
`GenericModelOutput` in the future.

### Sensor creation note: `force_camera_prim=True` does NOT work in Isaac 6.0

`IsaacSensorCreateRtxLidar(force_camera_prim=True)` creates a Camera prim but
`omni.sensors.nv.lidar.lidar_core.plugin` fails to load (interface version mismatch
in Isaac 6.0), resulting in `dataPtr input is empty` from the annotator.
Use `rep.functional.create.omni_lidar()` which creates an `OmniLidar` prim —
this IS the supported path in Isaac Sim 6.0.

### `Example_Rotary.usda` asset not in local mirror

The `IsaacSensorCreateRtxLidar` command's `_add_reference()` path requires
`~/isaacsim_assets/Isaac/Sensors/NVIDIA/Example_Rotary.usda` to exist locally.
If absent, the command raises `RuntimeError: Could not find assets root folder`
and falls through. We detect this at startup and skip directly to
`rep.functional.create.omni_lidar()`.

### Confirmed live output (ws://localhost:8765)

```
LIDAR: n=1562 pts=1562
  sample: [0.19, 0.37, -0.05]  [0.35, 0.22, 0.07]  [0.41, 0.01, -0.05]
LIDAR: n=1569 pts=1569
  sample: [0.19, 0.37, -0.05]  [0.35, 0.22, 0.03]  [0.47, -0.02, -0.05]
```

### Downsample rate

Every **6th render frame** → effectively 3–5 Hz at 20 fps simulation.
Stride = `max(1, n_raw // 1500)` to cap at 1500 points per sweep.
Rolling accumulation in UI: max 8000 pts (~5–6 scans).

### UI rendering

- `ui/app.js` `handleLidar(msg)`: accumulates points into `lidarBuffer` (max 8000 pts)
  and updates a `THREE.Points` / `THREE.BufferGeometry` live in the Three.js 3D map.
- Points are cyan (`#22d3ee`), size 0.06 world units.
- Toggle checkbox **"Show LiDAR"** (default on) in the map panel header.
- HUD legend: **`LiDAR: <n> pts, min <dist> m`** displayed below the map header.
- Axis mapping: Isaac sensor-frame coords (X,Y,Z) passed directly to Three.js
  (camera.up = (0,0,1)), consistent with drone/bin meshes.

### Files changed

- `scripts/live_sim.py` — LiDAR prim creation via `rep.functional.create.omni_lidar`,
  `IsaacCreateRTXLidarScanBuffer` annotator, per-frame `annotator.get_data()`,
  `_latest_lidar` shared state, broadcast in `_broadcast_loop`.
  Key `"data"` → shape `(N,3)` float32 XYZ; downsampled to ≤1500 pts.
- `ui/app.js` — `handleLidar`, `THREE.Points` / `BufferGeometry` init in `initThree`,
  toggle wiring, `lidar_min` in telemetry HUD
- `ui/index.html` — `map-controls` div with `#lidar-show` checkbox + `#lidar-hud`
- `ui/style.css` — `.map-controls`, `.lidar-toggle`, `.lidar-hud` styles
