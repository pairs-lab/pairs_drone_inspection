# M1 — Sim Environment Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Cài đặt và verify Isaac Sim 6.0, rồi sinh tự động (bằng script Python) scene kho mô phỏng gồm rack 3×6 = 18 BIN với pallet + nhãn GR (QR+text), drone quadrotor + camera, và file BIN Location Map.

**Architecture:** conda env riêng `isaac6` chạy Isaac Sim 6.0. Phần thuần Python (sinh `bin_map.yaml`, sinh texture nhãn GR, QR roundtrip) tách rời để test bằng pytest. Phần phụ thuộc Isaac Sim (dựng stage USD, render, chụp camera) verify bằng script chạy thật vì không unit-test được. Mọi đường dẫn asset/import Isaac được xác nhận ở Task 1–2 trước khi dùng ở các task sau.

**Tech Stack:** Isaac Sim 6.0 (`isaacsim[all]==6.0.0`), Python 3.11, `pxr` (USD), `qrcode`, `pyzbar`, `Pillow`, `PyYAML`, `pytest`.

---

## File Structure

- `sim/scene_builder.py` — entrypoint sinh scene (chạy bằng python của isaac6). Một trách nhiệm: lắp scene từ các thành phần.
- `sim/warehouse.py` — dựng kho (nền, ánh sáng, warehouse asset).
- `sim/rack.py` — dựng rack 3×6 + đặt pallet/box theo từng BIN.
- `sim/gr_label.py` — **thuần Python**: sinh ảnh texture nhãn GR (QR + text). Testable.
- `sim/bin_map.py` — **thuần Python**: sinh/đọc/validate `bin_map.yaml`. Testable.
- `sim/drone_asset.py` — spawn quadrotor + camera tại home pose.
- `sim/bin_map.yaml` — output: 18 BIN → pose + scan pose + ground-truth part_no/qty.
- `sim/assets/labels/` — output texture nhãn GR (PNG).
- `sim/config.py` — hằng số layout (số cột=3, tầng=6, kích thước rack, offset scan pose).
- `tests/test_bin_map.py`, `tests/test_gr_label.py` — pytest cho phần thuần Python.
- `scripts/verify_isaac.py`, `scripts/verify_scene.py` — verify phần Isaac.
- `environment.isaac6.yml`, `requirements-sim.txt` — môi trường.

---

## Task 1: Khởi tạo repo + conda env `isaac6` + cài Isaac Sim 6.0

**Files:**
- Create: `.gitignore`
- Create: `environment.isaac6.yml`
- Create: `requirements-sim.txt`

- [ ] **Step 1: Khởi tạo git repo (chưa phải repo)**

Run:
```bash
cd /home/cuongtdm/Desktop/Drone_poc && git init && git add CLAUDE.md docs && git commit -m "chore: init repo with spec and plans"
```
Expected: tạo repo, commit đầu tiên thành công.

- [ ] **Step 2: Viết `.gitignore`**

```
__pycache__/
*.pyc
sim/assets/labels/*.png
backend/*.db
.isaac_cache/
*.log
```

- [ ] **Step 3: Viết `requirements-sim.txt` (phần thuần Python, cài trong isaac6)**

```
qrcode==7.4.2
pyzbar==0.1.9
Pillow>=10.0
PyYAML>=6.0
pytest>=8.0
```

- [ ] **Step 4: Tạo conda env `isaac6` (Python 3.11)**

Run:
```bash
conda create -y -n isaac6 python=3.11 && conda run -n isaac6 python --version
```
Expected: `Python 3.11.x`. Lưu `environment.isaac6.yml`:
```yaml
name: isaac6
channels: [conda-forge]
dependencies:
  - python=3.11
  - pip
```

- [ ] **Step 5: Cài Isaac Sim 6.0 qua pip**

Run:
```bash
conda run -n isaac6 pip install "isaacsim[all]==6.0.0" --extra-index-url https://pypi.nvidia.com
```
Expected: cài thành công. **Nếu lỗi** (pip 6.0.0 chưa có / Blackwell không tương thích): fallback theo `docs.isaacsim.omniverse.nvidia.com/6.0.0/installation` — bản binary workstation hoặc container — và ghi lại cách đã dùng vào `docs/superpowers/INSTALL_NOTES.md`. Không tiếp tục cho tới khi import được (Task 2).

- [ ] **Step 6: Cài deps thuần Python + thư viện hệ thống cho pyzbar**

Run:
```bash
sudo apt-get install -y libzbar0 && conda run -n isaac6 pip install -r requirements-sim.txt
```
Expected: cài thành công (`libzbar0` cần cho `pyzbar`).

- [ ] **Step 7: Commit**

```bash
git add .gitignore environment.isaac6.yml requirements-sim.txt && git commit -m "chore: add isaac6 env and sim deps"
```

## Task 2: Verify Isaac Sim 6.0 chạy được (gate cho mọi task Isaac)

**Files:**
- Create: `scripts/verify_isaac.py`

- [ ] **Step 1: Viết script verify mở SimulationApp headless**

```python
# scripts/verify_isaac.py
"""Verify Isaac Sim 6.0 import + headless launch. Run with isaac6 python."""
from isaacsim import SimulationApp

app = SimulationApp({"headless": True})
import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom  # noqa: E402

stage = omni.usd.get_context().get_stage()
UsdGeom.Xform.Define(stage, "/World")
print("ISAAC_OK stage_prims=", len(list(stage.Traverse())))
app.close()
```

- [ ] **Step 2: Chạy verify**

Run:
```bash
conda run -n isaac6 python scripts/verify_isaac.py 2>&1 | tail -5
```
Expected: in `ISAAC_OK stage_prims= ...`, không traceback. **Nếu import path khác** (vd `isaacsim.core` namespace thay đổi ở 6.0): sửa script theo lỗi thật, ghi import path đúng vào `docs/superpowers/INSTALL_NOTES.md` để các task sau dùng.

- [ ] **Step 3: Commit**

```bash
git add scripts/verify_isaac.py docs/superpowers/INSTALL_NOTES.md && git commit -m "test: verify Isaac Sim 6.0 headless launch"
```

## Task 3: Layout config + sinh `bin_map.yaml` (thuần Python, TDD)

**Files:**
- Create: `sim/config.py`
- Create: `sim/bin_map.py`
- Test: `tests/test_bin_map.py`

- [ ] **Step 1: Viết failing test**

```python
# tests/test_bin_map.py
from sim.bin_map import generate_bin_map, validate_bin_map, BIN_IDS

def test_has_18_bins():
    m = generate_bin_map()
    assert len(m) == 18
    assert set(m.keys()) == set(BIN_IDS)

def test_bin_id_format():
    # 3 cột A,B,C × 6 tầng 1..6
    assert "A1" in BIN_IDS and "C6" in BIN_IDS

def test_each_bin_has_required_fields():
    m = generate_bin_map()
    for b in m.values():
        assert {"pallet_pose", "scan_pose", "part_no", "qty"} <= b.keys()
        assert len(b["scan_pose"]["position"]) == 3
        assert isinstance(b["qty"], int)

def test_validate_accepts_generated():
    assert validate_bin_map(generate_bin_map()) is True

def test_validate_rejects_wrong_count():
    bad = generate_bin_map(); bad.pop("A1")
    import pytest
    with pytest.raises(ValueError):
        validate_bin_map(bad)
```

- [ ] **Step 2: Chạy test để thấy fail**

Run: `conda run -n isaac6 python -m pytest tests/test_bin_map.py -v`
Expected: FAIL `ModuleNotFoundError: No module named 'sim.bin_map'`.

- [ ] **Step 3: Viết `sim/config.py`**

```python
# sim/config.py
COLUMNS = ["A", "B", "C"]      # 3 cột
LEVELS = [1, 2, 3, 4, 5, 6]    # 6 tầng
COLUMN_SPACING = 1.2           # m, khoảng cách giữa cột
LEVEL_HEIGHT = 0.8             # m, chiều cao mỗi tầng
RACK_ORIGIN = (0.0, 0.0, 0.0)  # gốc rack trong world (m)
SCAN_STANDOFF = 1.5            # m, drone đứng cách mặt nhãn bao xa
HOME_POSE = {"position": [-2.0, -2.0, 1.0], "yaw_deg": 0.0}
```

- [ ] **Step 4: Viết `sim/bin_map.py`**

```python
# sim/bin_map.py
import yaml
from sim.config import (COLUMNS, LEVELS, COLUMN_SPACING, LEVEL_HEIGHT,
                        RACK_ORIGIN, SCAN_STANDOFF)

BIN_IDS = [f"{c}{l}" for c in COLUMNS for l in LEVELS]
REQUIRED = {"pallet_pose", "scan_pose", "part_no", "qty"}

def _pallet_position(col_idx, level):
    ox, oy, oz = RACK_ORIGIN
    return [ox + col_idx * COLUMN_SPACING, oy, oz + (level - 1) * LEVEL_HEIGHT]

def generate_bin_map():
    m = {}
    for ci, col in enumerate(COLUMNS):
        for level in LEVELS:
            bid = f"{col}{level}"
            px, py, pz = _pallet_position(ci, level)
            m[bid] = {
                "pallet_pose": {"position": [px, py, pz], "yaw_deg": 0.0},
                "scan_pose": {"position": [px, py - SCAN_STANDOFF, pz], "yaw_deg": 90.0},
                "part_no": f"PN-{col}{level:02d}",
                "qty": 10 + ci * 6 + level,
            }
    return m

def validate_bin_map(m):
    if len(m) != len(BIN_IDS) or set(m) != set(BIN_IDS):
        raise ValueError(f"bin_map must have exactly {len(BIN_IDS)} bins: {BIN_IDS}")
    for bid, b in m.items():
        if not REQUIRED <= b.keys():
            raise ValueError(f"bin {bid} missing fields {REQUIRED - b.keys()}")
        if len(b["scan_pose"]["position"]) != 3:
            raise ValueError(f"bin {bid} scan_pose.position must be xyz")
    return True

def write_bin_map(path="sim/bin_map.yaml"):
    m = generate_bin_map(); validate_bin_map(m)
    with open(path, "w") as f:
        yaml.safe_dump(m, f, sort_keys=True)
    return path

def load_bin_map(path="sim/bin_map.yaml"):
    with open(path) as f:
        m = yaml.safe_load(f)
    validate_bin_map(m)
    return m

if __name__ == "__main__":
    print("wrote", write_bin_map())
```

- [ ] **Step 5: Chạy test để thấy pass**

Run: `conda run -n isaac6 python -m pytest tests/test_bin_map.py -v`
Expected: 5 passed.

- [ ] **Step 6: Sinh file `bin_map.yaml` và commit**

Run: `conda run -n isaac6 python -m sim.bin_map`
Expected: `wrote sim/bin_map.yaml`.
```bash
git add sim/config.py sim/bin_map.py tests/test_bin_map.py sim/bin_map.yaml && git commit -m "feat: bin map generator with 18-bin schema validation"
```

## Task 4: Sinh texture nhãn GR (QR + text) + QR roundtrip (thuần Python, TDD)

**Files:**
- Create: `sim/gr_label.py`
- Test: `tests/test_gr_label.py`

- [ ] **Step 1: Viết failing test (QR encode→decode roundtrip)**

```python
# tests/test_gr_label.py
from pathlib import Path
from pyzbar.pyzbar import decode
from PIL import Image
from sim.gr_label import make_label_image, encode_payload, decode_payload

def test_payload_roundtrip():
    p = encode_payload("PN-A01", 11)
    pn, qty = decode_payload(p)
    assert pn == "PN-A01" and qty == 11

def test_label_qr_decodes(tmp_path):
    out = tmp_path / "A1.png"
    make_label_image("PN-A01", 11, str(out))
    assert out.exists()
    decoded = decode(Image.open(out))
    assert decoded, "no QR found in rendered label"
    pn, qty = decode_payload(decoded[0].data.decode())
    assert pn == "PN-A01" and qty == 11
```

- [ ] **Step 2: Chạy test để thấy fail**

Run: `conda run -n isaac6 python -m pytest tests/test_gr_label.py -v`
Expected: FAIL `ModuleNotFoundError: No module named 'sim.gr_label'`.

- [ ] **Step 3: Viết `sim/gr_label.py`**

```python
# sim/gr_label.py
import json
import qrcode
from PIL import Image, ImageDraw, ImageFont

def encode_payload(part_no, qty):
    return json.dumps({"part_no": part_no, "qty": int(qty)})

def decode_payload(s):
    d = json.loads(s)
    return d["part_no"], int(d["qty"])

def make_label_image(part_no, qty, out_path, size=512):
    canvas = Image.new("RGB", (size, size), "white")
    qr = qrcode.make(encode_payload(part_no, qty)).resize((size, size // 2))
    canvas.paste(qr, (size // 4, 0))
    draw = ImageDraw.Draw(canvas)
    try:
        font = ImageFont.truetype("DejaVuSans-Bold.ttf", 40)
    except OSError:
        font = ImageFont.load_default()
    draw.text((20, size // 2 + 20), f"GR LABEL", fill="black", font=font)
    draw.text((20, size // 2 + 80), f"Part No: {part_no}", fill="black", font=font)
    draw.text((20, size // 2 + 140), f"Qty: {qty}", fill="black", font=font)
    canvas.save(out_path)
    return out_path
```

- [ ] **Step 4: Chạy test để thấy pass**

Run: `conda run -n isaac6 python -m pytest tests/test_gr_label.py -v`
Expected: 2 passed.

- [ ] **Step 5: Sinh toàn bộ 18 nhãn từ bin_map**

Create thêm hàm trong `sim/gr_label.py`:
```python
def generate_all_labels(out_dir="sim/assets/labels"):
    import os
    from sim.bin_map import load_bin_map
    os.makedirs(out_dir, exist_ok=True)
    paths = {}
    for bid, b in load_bin_map().items():
        paths[bid] = make_label_image(b["part_no"], b["qty"], f"{out_dir}/{bid}.png")
    return paths

if __name__ == "__main__":
    print("labels:", len(generate_all_labels()))
```
Run: `conda run -n isaac6 python -m sim.gr_label`
Expected: `labels: 18`.

- [ ] **Step 6: Commit**

```bash
git add sim/gr_label.py tests/test_gr_label.py && git commit -m "feat: GR label texture generator with QR roundtrip"
```

## Task 5: Dựng kho + rack 3×6 trong USD stage (Isaac, verify script)

**Files:**
- Create: `sim/warehouse.py`
- Create: `sim/rack.py`
- Create: `scripts/verify_scene.py`

- [ ] **Step 1: Viết `sim/warehouse.py`**

```python
# sim/warehouse.py
"""Dựng nền + ánh sáng + (tùy chọn) warehouse asset. Gọi sau khi đã có stage."""
from pxr import Usd, UsdGeom, UsdLux, Gf

def build_warehouse(stage: Usd.Stage):
    UsdGeom.Xform.Define(stage, "/World")
    # nền
    plane = UsdGeom.Mesh.Define(stage, "/World/Ground")
    plane.CreatePointsAttr([(-10, -10, 0), (10, -10, 0), (10, 10, 0), (-10, 10, 0)])
    plane.CreateFaceVertexCountsAttr([4])
    plane.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    # ánh sáng
    light = UsdLux.DistantLight.Define(stage, "/World/Light")
    light.CreateIntensityAttr(3000.0)
    return "/World"
```
Note: warehouse asset SimReady (Warehouse01) sẽ thêm ở Task 5 bước 4 sau khi xác nhận đường dẫn asset thật.

- [ ] **Step 2: Viết `sim/rack.py`**

```python
# sim/rack.py
"""Dựng rack 3×6: mỗi BIN là 1 box (pallet) + 1 quad mang texture nhãn GR."""
from pxr import UsdGeom, UsdShade, Sdf, Gf
from sim.bin_map import load_bin_map

def _make_label_material(stage, bid, tex_path):
    mat = UsdShade.Material.Define(stage, f"/World/Materials/Label_{bid}")
    shader = UsdShade.Shader.Define(stage, f"/World/Materials/Label_{bid}/PBR")
    shader.CreateIdAttr("UsdPreviewSurface")
    tex = UsdShade.Shader.Define(stage, f"/World/Materials/Label_{bid}/Tex")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(tex_path)
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(
        tex.CreateOutput("rgb", Sdf.ValueTypeNames.Float3))
    mat.CreateSurfaceOutput().ConnectToSource(
        shader.CreateOutput("surface", Sdf.ValueTypeNames.Token))
    return mat

def build_rack(stage):
    bins = load_bin_map()
    for bid, b in bins.items():
        px, py, pz = b["pallet_pose"]["position"]
        # pallet box
        cube = UsdGeom.Cube.Define(stage, f"/World/Rack/Bin_{bid}/Pallet")
        cube.AddTranslateOp().Set(Gf.Vec3d(px, py, pz))
        cube.AddScaleOp().Set(Gf.Vec3f(0.4, 0.4, 0.3))
        # quad nhãn GR ở mặt trước pallet
        quad = UsdGeom.Mesh.Define(stage, f"/World/Rack/Bin_{bid}/Label")
        s = 0.3
        quad.CreatePointsAttr([(-s, 0, -s), (s, 0, -s), (s, 0, s), (-s, 0, s)])
        quad.CreateFaceVertexCountsAttr([4]); quad.CreateFaceVertexIndicesAttr([0,1,2,3])
        quad.CreateExtentAttr([(-s,0,-s),(s,0,s)])
        quad.AddTranslateOp().Set(Gf.Vec3d(px, py - 0.41, pz))
        tex = f"sim/assets/labels/{bid}.png"
        mat = _make_label_material(stage, bid, tex)
        UsdShade.MaterialBindingAPI(quad.GetPrim()).Bind(mat)
    return len(bins)
```

- [ ] **Step 3: Viết `scripts/verify_scene.py` (render + đếm prim)**

```python
# scripts/verify_scene.py
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import omni.usd  # noqa
from sim.warehouse import build_warehouse
from sim.rack import build_rack

stage = omni.usd.get_context().get_stage()
build_warehouse(stage)
n = build_rack(stage)
prims = len(list(stage.Traverse()))
print(f"SCENE_OK bins={n} prims={prims}")
assert n == 18, "expected 18 bins"
app.close()
```

- [ ] **Step 4: Chạy verify**

Run: `conda run -n isaac6 python scripts/verify_scene.py 2>&1 | tail -5`
Expected: `SCENE_OK bins=18 prims=...`. **Nếu API USD khác ở 6.0**, sửa theo lỗi thật và ghi chú `INSTALL_NOTES.md`. Sau khi pass, (tùy chọn) thêm warehouse asset SimReady qua `Window > Browsers > NVIDIA Assets` path đã xác nhận và reference vào `/World/Warehouse`.

- [ ] **Step 5: Commit**

```bash
git add sim/warehouse.py sim/rack.py scripts/verify_scene.py && git commit -m "feat: build warehouse + 3x6 rack with GR label quads"
```

## Task 6: Spawn drone + camera, chụp ảnh, decode QR end-to-end (Isaac, verify script)

**Files:**
- Create: `sim/drone_asset.py`
- Modify: `scripts/verify_scene.py` (thêm drone + capture)

- [ ] **Step 1: Viết `sim/drone_asset.py`**

```python
# sim/drone_asset.py
"""Spawn 1 drone body (quad placeholder) + camera tại home pose.
Physics/controller chi tiết thuộc plan M2; ở M1 chỉ cần body + camera tĩnh."""
from pxr import UsdGeom, Gf
from sim.config import HOME_POSE

def spawn_drone(stage, prim_path="/World/Drone"):
    body = UsdGeom.Cube.Define(stage, prim_path + "/Body")
    x, y, z = HOME_POSE["position"]
    body.AddTranslateOp().Set(Gf.Vec3d(x, y, z))
    body.AddScaleOp().Set(Gf.Vec3f(0.15, 0.15, 0.05))
    cam = UsdGeom.Camera.Define(stage, prim_path + "/Camera")
    cam.AddTranslateOp().Set(Gf.Vec3d(x, y, z - 0.1))
    return prim_path + "/Camera"
```

- [ ] **Step 2: Thêm capture + QR decode vào `scripts/verify_scene.py`**

Thêm trước `app.close()`:
```python
from sim.drone_asset import spawn_drone
from sim.bin_map import load_bin_map
import omni.replicator.core as rep
from pyzbar.pyzbar import decode
from PIL import Image

cam_path = spawn_drone(stage)
# Đặt camera tới scan_pose của A1 để chụp nhãn
a1 = load_bin_map()["A1"]["scan_pose"]["position"]
cam_prim = stage.GetPrimAtPath(cam_path)
from pxr import UsdGeom, Gf
UsdGeom.XformCommonAPI(cam_prim).SetTranslate(Gf.Vec3d(*a1))

rp = rep.create.render_product(cam_path, (1280, 720))
out = "sim/assets/capture_A1.png"
writer = rep.WriterRegistry.get("BasicWriter")
writer.initialize(output_dir="sim/assets", rgb=True)
writer.attach([rp])
for _ in range(5):
    rep.orchestrator.step()
print("CAPTURE_DONE")
```
Note: API replicator/capture có thể khác ở 6.0; nếu vậy dùng `isaacsim.sensors.camera` hoặc `Camera` helper, sửa theo lỗi thật và ghi `INSTALL_NOTES.md`. Mục tiêu: ra 1 file PNG chụp từ camera drone.

- [ ] **Step 3: Chạy verify scene đầy đủ**

Run: `conda run -n isaac6 python scripts/verify_scene.py 2>&1 | tail -10`
Expected: `SCENE_OK bins=18 ...` và `CAPTURE_DONE`, có file PNG trong `sim/assets/`.

- [ ] **Step 4: Verify ảnh chụp decode được QR của A1**

Create `scripts/verify_capture.py`:
```python
from pyzbar.pyzbar import decode
from PIL import Image
from sim.gr_label import decode_payload
import glob
f = sorted(glob.glob("sim/assets/rgb*.png") + glob.glob("sim/assets/capture_A1.png"))[-1]
res = decode(Image.open(f))
assert res, f"no QR decoded from {f}"
pn, qty = decode_payload(res[0].data.decode())
print(f"CAPTURE_QR_OK file={f} part_no={pn} qty={qty}")
assert pn == "PN-A01"
```
Run: `conda run -n isaac6 python scripts/verify_capture.py`
Expected: `CAPTURE_QR_OK ... part_no=PN-A01`. **Nếu không decode được**: tăng độ phân giải camera / giảm `SCAN_STANDOFF` trong `config.py` / phóng to quad nhãn cho tới khi decode ổn — đây là tham số M3/M2 sẽ tái dùng.

- [ ] **Step 5: Commit**

```bash
git add sim/drone_asset.py scripts/verify_scene.py scripts/verify_capture.py && git commit -m "feat: spawn drone+camera, capture and decode GR label end-to-end"
```

## Task 7: Entrypoint hợp nhất + README chạy M1

**Files:**
- Create: `sim/scene_builder.py`
- Create: `sim/README.md`

- [ ] **Step 1: Viết `sim/scene_builder.py` (gom mọi bước, có cờ `--gui`)**

```python
# sim/scene_builder.py
"""Entrypoint sinh scene M1. Mặc định headless; --gui để mở cửa sổ Isaac Sim."""
import argparse
from isaacsim import SimulationApp

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--gui", action="store_true")
    args = ap.parse_args()
    app = SimulationApp({"headless": not args.gui})
    import omni.usd
    from sim.warehouse import build_warehouse
    from sim.rack import build_rack
    from sim.drone_asset import spawn_drone
    stage = omni.usd.get_context().get_stage()
    build_warehouse(stage)
    n = build_rack(stage)
    spawn_drone(stage)
    omni.usd.get_context().save_as_stage("sim/warehouse_poc.usd", None)
    print(f"BUILT bins={n} saved=sim/warehouse_poc.usd")
    if args.gui:
        while app.is_running():
            app.update()
    app.close()

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Chạy build, lưu USD**

Run: `conda run -n isaac6 python -m sim.scene_builder 2>&1 | tail -3`
Expected: `BUILT bins=18 saved=sim/warehouse_poc.usd` và file `sim/warehouse_poc.usd` tồn tại.

- [ ] **Step 3: Viết `sim/README.md`**

```markdown
# M1 — Sim Environment

## Setup
conda activate isaac6  # đã cài isaacsim[all]==6.0.0 + requirements-sim.txt

## Chạy
python -m sim.bin_map            # sinh sim/bin_map.yaml (18 BIN)
python -m sim.gr_label           # sinh 18 nhãn GR vào sim/assets/labels/
python -m sim.scene_builder      # dựng scene headless, lưu warehouse_poc.usd
python -m sim.scene_builder --gui  # mở cửa sổ Isaac Sim để xem

## Test
python -m pytest tests/ -v

## Verify Isaac
python scripts/verify_isaac.py
python scripts/verify_scene.py
python scripts/verify_capture.py
```

- [ ] **Step 4: Chạy toàn bộ test lần cuối**

Run: `conda run -n isaac6 python -m pytest tests/ -v`
Expected: tất cả pass.

- [ ] **Step 5: Commit**

```bash
git add sim/scene_builder.py sim/README.md && git commit -m "feat: unified scene builder entrypoint + M1 README"
```

---

## Self-Review

**Spec coverage (M1 trong spec mục 5):**
- Cài Isaac Sim 6.0 env riêng → Task 1–2 ✓
- Sinh scene bằng script Python procedural → Task 5,7 ✓
- Kho + ánh sáng + scale → Task 5 (`warehouse.py`) ✓ (warehouse SimReady asset là tùy chọn, ghi rõ trong Task 5 Step 4)
- Rack 3×6 = 18 BIN + pallet/box → Task 5 (`rack.py`) ✓
- Nhãn GR (QR+text) sinh tự động, QR encode đúng Part No. → Task 4 ✓
- Semantic label cho prim → **GAP**: chưa có task. (Quyết định: hoãn sang khi tích hợp M3 perception nếu dùng SDG; QR decode đã đủ cho perception POC. Ghi nhận, không chặn M1.)
- BIN map yaml: 18 BIN → pose + scan pose + ground-truth → Task 3 ✓
- Drone + camera tại home → Task 6 ✓
- Kiểm thử M1 (render frame, ảnh thấy rack+nhãn, bin_map đủ 18, QR decode) → Task 2/5/6 verify scripts ✓

**Placeholder scan:** không có "TBD/TODO"; các nhánh fallback (install lỗi, API khác ở 6.0, QR không decode) đều có hành động cụ thể + nơi ghi chú. ✓

**Type consistency:** `generate_bin_map`/`load_bin_map`/`validate_bin_map`, `BIN_IDS`, `make_label_image`/`encode_payload`/`decode_payload`, `build_warehouse`/`build_rack`/`spawn_drone`, `HOME_POSE`, `SCAN_STANDOFF` dùng nhất quán giữa các task. ✓

**Lưu ý rủi ro (đã nêu trong spec mục 9):** exact import path Isaac 6.0, API replicator/camera capture, và đường dẫn SimReady asset chỉ chốt được khi chạy Task 2/5/6 — plan đã chỉ rõ sửa-theo-lỗi-thật + ghi `INSTALL_NOTES.md`.
