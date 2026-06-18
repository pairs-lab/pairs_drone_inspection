# M3 — AI Camera Perception (YOLO + pyzbar + PaddleOCR) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Từ ảnh camera drone (render RTX thật từ M1), phát hiện vùng QR/nhãn bằng **YOLO**, giải mã QR bằng **pyzbar**, đọc text nhãn bằng **PaddleOCR**, hợp nhất thành `PerceptionResult{bin_id?, part_no, qty, confidence, bbox}` cho backend.

**Architecture:** Perception KHÔNG cần Isaac runtime — chạy trên ảnh đã lưu, trong một conda env riêng `perception` (tách khỏi python của Isaac). Tập train/eval YOLO được sinh tự động trong Isaac bằng replicator bounding-box annotator (ground-truth bbox từ semantic label trên prim QR/nhãn) — không gán nhãn tay. Logic hợp nhất + decode là hàm thuần Python, test bằng pytest. Inference end-to-end verify trên ảnh thật `sim/assets/capture_A1.png`.

**Tech Stack:** conda env `perception` (Python 3.11) với `ultralytics` (YOLOv8), `paddleocr` + `paddlepaddle`, `pyzbar`, `opencv-python`, `numpy`, `pytest`. Dataset-gen chạy trong Isaac binary qua `scripts/run_isaac.sh`.

---

## Bối cảnh (đã có từ M1)

- `sim/rack.py build_rack(stage)`: mỗi BIN có quad nhãn `/World/Rack/Bin_<id>/Label` (emissive, texture QR+text) và pallet. Nhãn render thật (đã verify).
- `sim/gr_label.py`: `decode_payload(s)->(part_no,qty)`; QR encode JSON `{"part_no","qty"}`.
- `sim/bin_map.py load_bin_map()`: ground-truth part_no/qty từng BIN.
- Ảnh mẫu thật: `sim/assets/capture_A1.png` (QR "PN-A01", qty 11) decode được bằng pyzbar.
- Chạy Isaac: `scripts/run_isaac.sh <script.py>`.

## File Structure

- `perception/__init__.py`
- `perception/qr.py` — **thuần Python**: crop + pyzbar decode → part_no/qty. Testable.
- `perception/ocr.py` — PaddleOCR wrapper: đọc text nhãn, parse "Part No: X" / "Qty: N". Testable phần parse.
- `perception/detector.py` — YOLO wrapper: load weights, `detect(image)` → list `Detection{cls, bbox, conf}`.
- `perception/pipeline.py` — **thuần logic hợp nhất**: ghép YOLO+QR+OCR → `PerceptionResult`. Testable.
- `perception/types.py` — dataclasses `Detection`, `PerceptionResult`.
- `scripts/gen_perception_dataset.py` — Isaac: render N view/bin + bbox annotator → `data/perception/{images,labels}` (YOLO format) + `data.yaml`.
- `scripts/train_yolo.py` — train YOLOv8n trên dataset, lưu `perception/weights/best.pt`.
- `scripts/verify_perception.py` — chạy pipeline trên `sim/assets/capture_A1.png`, in `PERCEPTION_OK`.
- `tests/test_qr.py`, `tests/test_ocr_parse.py`, `tests/test_pipeline.py` — pytest (env perception).

---

## Task 1: Perception env + types (thuần Python, TDD)

**Files:**
- Create: `perception/__init__.py`, `perception/types.py`
- Create: `environment.perception.yml`
- Test: `tests/test_pipeline.py` (chỉ phần types ở task này)

- [ ] **Step 1: Tạo conda env `perception`**

Run:
```bash
conda create -y -n perception python=3.11 && conda install -y -n perception -c conda-forge zbar && conda run -n perception pip install ultralytics opencv-python pyzbar PyYAML pytest && conda run -n perception python -c "import ultralytics, cv2, pyzbar.pyzbar; print('PERCEPTION_DEPS_OK')"
```
Expected: `PERCEPTION_DEPS_OK`. (PaddleOCR cài ở Task 4 để cô lập rủi ro.) Lưu `environment.perception.yml`:
```yaml
name: perception
channels: [conda-forge]
dependencies: [python=3.11, pip, zbar]
```

- [ ] **Step 2: Viết failing test cho types**

```python
# tests/test_pipeline.py
from perception.types import Detection, PerceptionResult

def test_detection_fields():
    d = Detection(cls="qr", bbox=(10, 20, 100, 120), conf=0.9)
    assert d.cls == "qr" and d.conf == 0.9 and len(d.bbox) == 4

def test_result_defaults():
    r = PerceptionResult(part_no="PN-A01", qty=11, confidence=0.95,
                         bbox=(0, 0, 1, 1), source="qr")
    assert r.part_no == "PN-A01" and r.qty == 11 and r.source == "qr"
```

- [ ] **Step 3: Run → FAIL** `conda run -n perception python -m pytest tests/test_pipeline.py -v` (module missing).

- [ ] **Step 4: Implement `perception/types.py`**

```python
from dataclasses import dataclass
from typing import Optional, Tuple

@dataclass(frozen=True)
class Detection:
    cls: str                 # "qr" | "label"
    bbox: Tuple[int, int, int, int]   # x1,y1,x2,y2
    conf: float

@dataclass(frozen=True)
class PerceptionResult:
    part_no: Optional[str]
    qty: Optional[int]
    confidence: float
    bbox: Tuple[int, int, int, int]
    source: str              # "qr" | "ocr" | "qr+ocr"
```

- [ ] **Step 5: Run → PASS**, then commit:
```bash
git add perception/__init__.py perception/types.py environment.perception.yml tests/test_pipeline.py && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M3): perception env + Detection/PerceptionResult types

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 2: QR decode module (thuần Python, TDD)

**Files:**
- Create: `perception/qr.py`
- Test: `tests/test_qr.py`

- [ ] **Step 1: Viết failing test** (dùng nhãn thật từ M1)

```python
# tests/test_qr.py
import os
from perception.qr import decode_qr_in_image, decode_qr_crop

LABEL = "sim/assets/labels/A1.png"

def test_decode_full_label():
    assert os.path.exists(LABEL), "run `conda run -n isaac6 python -m sim.gr_label` first"
    res = decode_qr_in_image(LABEL)
    assert res is not None
    assert res[0] == "PN-A01" and res[1] == 11

def test_decode_crop_bbox():
    # whole image as bbox still decodes
    from PIL import Image
    import numpy as np
    arr = np.array(Image.open(LABEL).convert("RGB"))
    h, w = arr.shape[:2]
    res = decode_qr_crop(arr, (0, 0, w, h))
    assert res == ("PN-A01", 11)
```

- [ ] **Step 2: Run → FAIL.**

- [ ] **Step 3: Implement `perception/qr.py`**

```python
import numpy as np
from PIL import Image
from pyzbar.pyzbar import decode
from sim.gr_label import decode_payload  # reuse JSON payload parser

def decode_qr_crop(image_rgb, bbox):
    """image_rgb: HxWx3 ndarray; bbox: x1,y1,x2,y2. Returns (part_no, qty) or None."""
    x1, y1, x2, y2 = [int(v) for v in bbox]
    crop = image_rgb[max(0, y1):y2, max(0, x1):x2]
    if crop.size == 0:
        return None
    found = decode(Image.fromarray(crop))
    if not found:
        return None
    try:
        return decode_payload(found[0].data.decode())
    except Exception:
        return None

def decode_qr_in_image(path):
    arr = np.array(Image.open(path).convert("RGB"))
    h, w = arr.shape[:2]
    return decode_qr_crop(arr, (0, 0, w, h))
```
NOTE: `tests/` run từ repo root; perception env cần thấy `sim` package → đảm bảo chạy pytest từ repo root (sim/ trên path). Nếu import `sim.gr_label` lỗi trong env perception (thiếu qrcode), copy chỉ hàm `decode_payload` vào perception/qr.py thay vì import (nó chỉ parse JSON, không cần qrcode).

- [ ] **Step 4: Run → PASS.** (Nếu import sim lỗi do thiếu deps, inline `decode_payload` như note.)
- [ ] **Step 5: Commit** `git add perception/qr.py tests/test_qr.py && git commit -m "feat(M3): QR decode module (pyzbar) with bbox crop"`.

## Task 3: Auto-generate YOLO dataset trong Isaac (verify script)

**Files:**
- Modify: `sim/rack.py` — thêm semantic label "qr"/"label" cho prim nhãn (để bbox annotator xuất ground-truth).
- Create: `scripts/gen_perception_dataset.py`

- [ ] **Step 1: Thêm semantic label vào `sim/rack.py`**

Trong `build_rack`, sau khi tạo quad nhãn, gán semantic class:
```python
from semantics.schema.editor import add_update_semantics  # Isaac semantics helper
# ... trong vòng lặp, sau khi tạo quad:
add_update_semantics(quad.GetPrim(), semantic_label="label", type_label="class")
```
NOTE: API semantics có thể là `isaacsim.core.utils.semantics.add_update_semantics(prim, "label")` ở 6.0 — verify và dùng cái import được; ghi vào INSTALL_NOTES. Giữ thay đổi tối thiểu, không ảnh hưởng render.

- [ ] **Step 2: Implement `scripts/gen_perception_dataset.py`**

```python
"""Render N views per bin from jittered camera poses around each scan pose, and
write YOLO-format dataset using replicator's 2D bbox annotator (ground-truth)."""
import os, numpy as np
from isaacsim import SimulationApp
app = SimulationApp({"headless": True, "active_gpu": 0, "physics_gpu": 0})
import omni.usd, omni.replicator.core as rep
from pxr import UsdGeom, Gf
from sim.warehouse import build_warehouse
from sim.rack import build_rack
from sim.bin_map import load_bin_map
from PIL import Image

OUT = "data/perception"; os.makedirs(f"{OUT}/images", exist_ok=True); os.makedirs(f"{OUT}/labels", exist_ok=True)
VIEWS_PER_BIN = 8
W, H = 1280, 720

stage = omni.usd.get_context().get_stage(); UsdGeom.SetStageUpAxis(stage, "Z")
build_warehouse(stage); build_rack(stage)
bins = load_bin_map()

cam = rep.create.camera(name="DsCam")
for p in stage.Traverse():
    if p.GetPath().pathString.startswith("/Replicator") and p.IsA(UsdGeom.Camera):
        UsdGeom.Camera(p).GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 1e6))
rp = rep.create.render_product(cam, (W, H), name="DsRP")
rgb = rep.AnnotatorRegistry.get_annotator("LdrColor"); rgb.attach([rp])
bbox = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight"); bbox.attach([rp])

idx = 0
for bid, b in bins.items():
    sx, sy, sz = b["scan_pose"]["position"]
    for v in range(VIEWS_PER_BIN):
        jx = (v % 3 - 1) * 0.15; jz = ((v // 3) - 1) * 0.15; back = 1.0 + 0.5 * (v % 2)
        pos = (sx + jx, sy - back, sz + jz)
        with cam:
            rep.modify.pose(position=pos, look_at=(sx, sy - 0.41 + 0.41, sz))
        for _ in range(20): app.update()
        rep.orchestrator.step(rt_subframes=32, wait_for_render=True)
        img = np.asarray(rgb.get_data())[:, :, :3].astype(np.uint8)
        Image.fromarray(img).save(f"{OUT}/images/{idx:05d}.png")
        bb = bbox.get_data()
        lines = []
        for row in bb["data"]:
            x1, y1, x2, y2 = row["x_min"], row["y_min"], row["x_max"], row["y_max"]
            cx = (x1 + x2) / 2 / W; cy = (y1 + y2) / 2 / H
            bw = (x2 - x1) / W; bh = (y2 - y1) / H
            if bw > 0 and bh > 0:
                lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")  # class 0 = label
        open(f"{OUT}/labels/{idx:05d}.txt", "w").write("\n".join(lines))
        idx += 1
print(f"DATASET_OK images={idx}")
# data.yaml for ultralytics
open(f"{OUT}/data.yaml", "w").write(
    f"path: {os.path.abspath(OUT)}\ntrain: images\nval: images\nnames:\n  0: label\n")
app.close()
```

- [ ] **Step 3: Run & verify**

Run: `scripts/run_isaac.sh scripts/gen_perception_dataset.py 2>&1 | tail -15`
Expected: `DATASET_OK images=144` (18 bins × 8) và thư mục `data/perception/images` + `labels` + `data.yaml`. Verify ngẫu nhiên 1 ảnh + nhãn khớp (mở ảnh, bbox bao nhãn). IF bbox annotator key names khác (`x_min`...) hoặc semantic API khác: đọc cấu trúc thật của `bbox.get_data()` (in `bb.dtype`/`bb["info"]`), sửa cho đúng, ghi INSTALL_NOTES.

- [ ] **Step 4: gitignore dataset + commit code**

```bash
printf '\ndata/perception/\nperception/weights/\n' >> .gitignore
git add sim/rack.py scripts/gen_perception_dataset.py .gitignore docs/superpowers/INSTALL_NOTES.md && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M3): auto-generate YOLO dataset via replicator bbox annotator

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 4: PaddleOCR text reader (env perception, TDD on parse)

**Files:**
- Create: `perception/ocr.py`
- Test: `tests/test_ocr_parse.py`

- [ ] **Step 1: Cài PaddleOCR vào env perception**

Run:
```bash
conda run -n perception pip install paddlepaddle paddleocr && conda run -n perception python -c "from paddleocr import PaddleOCR; print('PADDLE_OK')"
```
Expected: `PADDLE_OK` (lần đầu tải model). IF cài lỗi (paddle wheel/CPU-GPU): thử `paddlepaddle-gpu` hoặc bản CPU, ghi INSTALL_NOTES. Đây là dep nặng nhất — cô lập ở task này.

- [ ] **Step 2: Viết failing test cho parser (thuần, không cần model)**

```python
# tests/test_ocr_parse.py
from perception.ocr import parse_label_text

def test_parse_partno_qty():
    lines = ["GR LABEL", "Part No: PN-A01", "Qty: 11"]
    pn, qty = parse_label_text(lines)
    assert pn == "PN-A01" and qty == 11

def test_parse_tolerates_noise_and_case():
    lines = ["part no  pn-b03", "QTY :  19", "garbage"]
    pn, qty = parse_label_text(lines)
    assert pn == "PN-B03" and qty == 19

def test_parse_missing_returns_none():
    pn, qty = parse_label_text(["nothing here"])
    assert pn is None and qty is None
```

- [ ] **Step 3: Run → FAIL.**

- [ ] **Step 4: Implement `perception/ocr.py`**

```python
import re

def parse_label_text(lines):
    """From OCR text lines, extract (part_no, qty). Tolerant to case/spacing."""
    text = " \n ".join(lines)
    pn = None; qty = None
    m = re.search(r"part\s*no\.?\s*:?\s*([A-Za-z]{1,4}-?[A-Za-z0-9]+)", text, re.I)
    if m:
        pn = m.group(1).upper().replace(" ", "")
        if "-" not in pn and len(pn) > 2:  # normalize PNA01 -> PN-A01 style if needed
            pass
    m = re.search(r"qty\s*:?\s*(\d+)", text, re.I)
    if m:
        qty = int(m.group(1))
    return pn, qty

class LabelOCR:
    """Lazy PaddleOCR wrapper. read(image_rgb, bbox) -> (part_no, qty, raw_lines)."""
    def __init__(self, lang="en"):
        self._ocr = None; self._lang = lang
    def _ensure(self):
        if self._ocr is None:
            from paddleocr import PaddleOCR
            self._ocr = PaddleOCR(use_angle_cls=True, lang=self._lang, show_log=False)
    def read(self, image_rgb, bbox):
        self._ensure()
        x1, y1, x2, y2 = [int(v) for v in bbox]
        crop = image_rgb[max(0,y1):y2, max(0,x1):x2]
        result = self._ocr.ocr(crop, cls=True)
        lines = []
        for block in (result or []):
            for line in (block or []):
                lines.append(line[1][0])
        pn, qty = parse_label_text(lines)
        return pn, qty, lines
```
NOTE: PaddleOCR API/`show_log`/return shape thay đổi theo version — verify `result` structure ở Task 6 và điều chỉnh trích `lines`; giữ `parse_label_text` ổn định (đã test).

- [ ] **Step 5: Run → PASS** (parser test, không gọi model), commit:
```bash
git add perception/ocr.py tests/test_ocr_parse.py docs/superpowers/INSTALL_NOTES.md && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M3): PaddleOCR wrapper + tolerant label text parser"
```

## Task 5: Train YOLO (env perception)

**Files:**
- Create: `scripts/train_yolo.py`

- [ ] **Step 1: Implement `scripts/train_yolo.py`**

```python
"""Fine-tune YOLOv8n to detect the GR 'label' class from the synthetic dataset."""
from ultralytics import YOLO
import os

def main(epochs=40, imgsz=960):
    assert os.path.exists("data/perception/data.yaml"), "run gen_perception_dataset first"
    model = YOLO("yolov8n.pt")  # pretrained, auto-downloads
    model.train(data="data/perception/data.yaml", epochs=epochs, imgsz=imgsz,
                project="perception/runs", name="label_det", exist_ok=True)
    best = "perception/runs/label_det/weights/best.pt"
    os.makedirs("perception/weights", exist_ok=True)
    import shutil; shutil.copy(best, "perception/weights/best.pt")
    print(f"TRAIN_OK weights=perception/weights/best.pt")

if __name__ == "__main__":
    main()
```

- [ ] **Step 2: Run training**

Run: `conda run -n perception python scripts/train_yolo.py 2>&1 | tail -20`
Expected: `TRAIN_OK weights=perception/weights/best.pt`, mAP report > 0.5 trên val (tập synthetic dễ). Nếu GPU OOM: giảm `imgsz`/batch. Train trên RTX 5060 Ti vài phút.

- [ ] **Step 3: Commit code (weights gitignored)** `git add scripts/train_yolo.py && git commit -m "feat(M3): YOLOv8n training script for label detection"`.

## Task 6: Detector + pipeline + end-to-end verify

**Files:**
- Create: `perception/detector.py`
- Create: `perception/pipeline.py`
- Create: `scripts/verify_perception.py`
- Test: extend `tests/test_pipeline.py`

- [ ] **Step 1: Implement `perception/detector.py`**

```python
from perception.types import Detection

class LabelDetector:
    def __init__(self, weights="perception/weights/best.pt", conf=0.25):
        from ultralytics import YOLO
        self.model = YOLO(weights); self.conf = conf
    def detect(self, image_rgb):
        res = self.model.predict(image_rgb, conf=self.conf, verbose=False)[0]
        out = []
        for b in res.boxes:
            x1, y1, x2, y2 = [int(v) for v in b.xyxy[0].tolist()]
            out.append(Detection(cls="label", bbox=(x1, y1, x2, y2),
                                 conf=float(b.conf[0])))
        return out
```

- [ ] **Step 2: Implement `perception/pipeline.py`** (logic hợp nhất — testable với fakes)

```python
from perception.types import PerceptionResult
from perception.qr import decode_qr_crop

def fuse(image_rgb, detections, ocr_reader=None):
    """For each detected label region: try QR first (authoritative), fall back to OCR.
    Returns the best PerceptionResult or None."""
    best = None
    for d in detections:
        qr = decode_qr_crop(image_rgb, d.bbox)
        if qr:
            cand = PerceptionResult(qr[0], qr[1], d.conf, d.bbox, "qr")
        elif ocr_reader is not None:
            pn, qty, _ = ocr_reader.read(image_rgb, d.bbox)
            if pn is None and qty is None:
                continue
            cand = PerceptionResult(pn, qty, d.conf * 0.7, d.bbox, "ocr")
        else:
            continue
        if best is None or cand.confidence > best.confidence:
            best = cand
    return best
```

- [ ] **Step 3: Add pipeline fusion test (fakes, no model)**

```python
# append to tests/test_pipeline.py
import numpy as np
from PIL import Image
from perception.types import Detection
from perception.pipeline import fuse

def test_fuse_prefers_qr_from_real_label():
    arr = np.array(Image.open("sim/assets/labels/A1.png").convert("RGB"))
    h, w = arr.shape[:2]
    dets = [Detection("label", (0, 0, w, h), 0.9)]
    r = fuse(arr, dets)            # no ocr_reader -> QR path
    assert r is not None and r.part_no == "PN-A01" and r.qty == 11 and r.source == "qr"

class _FakeOCR:
    def read(self, img, bbox): return ("PN-Z9", 5, ["Part No: PN-Z9", "Qty: 5"])

def test_fuse_falls_back_to_ocr_when_no_qr():
    blank = np.full((50, 50, 3), 255, np.uint8)   # no QR
    dets = [Detection("label", (0, 0, 50, 50), 0.8)]
    r = fuse(blank, dets, ocr_reader=_FakeOCR())
    assert r.source == "ocr" and r.part_no == "PN-Z9" and r.qty == 5
```

- [ ] **Step 4: Run pipeline tests → PASS** `conda run -n perception python -m pytest tests/test_pipeline.py tests/test_qr.py -v`.

- [ ] **Step 5: Implement `scripts/verify_perception.py` (end-to-end on real render)**

```python
import numpy as np
from PIL import Image
from perception.detector import LabelDetector
from perception.ocr import LabelOCR
from perception.pipeline import fuse

img = np.array(Image.open("sim/assets/capture_A1.png").convert("RGB"))
det = LabelDetector()
dets = det.detect(img)
print(f"detections={len(dets)}")
res = fuse(img, dets, ocr_reader=LabelOCR())
assert res is not None, "no perception result"
print(f"PERCEPTION_OK source={res.source} part_no={res.part_no} qty={res.qty} conf={res.confidence:.2f}")
assert res.part_no == "PN-A01"
```

- [ ] **Step 6: Run end-to-end verify**

Run: `conda run -n perception python scripts/verify_perception.py 2>&1 | tail -15`
Expected: `detections>=1` và `PERCEPTION_OK source=qr part_no=PN-A01 qty=11 ...`. IF YOLO không bắt được nhãn (conf thấp): tăng dữ liệu/epoch ở Task 3/5 hoặc giảm conf; vì QR rõ nên ngay cả full-image decode cũng đúng — đảm bảo có ít nhất 1 detection bao nhãn.

- [ ] **Step 7: Commit**

```bash
git add perception/detector.py perception/pipeline.py scripts/verify_perception.py tests/test_pipeline.py && git -c user.email=mtdinh@nvidia.com -c user.name="mtdinh" commit -m "feat(M3): YOLO detector + fusion pipeline + end-to-end PERCEPTION_OK

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

## Task 7: M3 README

**Files:** Create `perception/README.md` — env `perception`, luồng YOLO→QR→OCR, cách sinh dataset (`scripts/run_isaac.sh scripts/gen_perception_dataset.py`), train (`conda run -n perception python scripts/train_yolo.py`), verify, và điểm tích hợp với M4 (hàm nhận ảnh → PerceptionResult). Commit.

---

## Self-Review

**Spec coverage (M3 trong spec, đã cập nhật YOLO+pyzbar+PaddleOCR):**
- YOLO detect QR/bin → Task 3 (dataset) + Task 5 (train) + Task 6 (detector) ✓
- pyzbar decode QR → Task 2 ✓
- PaddleOCR đọc text → Task 4 ✓
- Hợp nhất → PerceptionResult → Task 6 (fuse) ✓
- Chạy trên ảnh render thật + accuracy → Task 6 verify trên capture_A1.png ✓
- Dữ liệu train từ M1 (ground-truth bbox) → Task 3 (replicator bbox annotator) ✓

**Placeholder scan:** không có TBD; điểm bất định (semantic API, bbox key names, PaddleOCR return shape, paddle install) đều có hành động cụ thể + ghi INSTALL_NOTES. ✓

**Type consistency:** `Detection(cls,bbox,conf)`, `PerceptionResult(part_no,qty,confidence,bbox,source)`, `decode_qr_crop`/`decode_qr_in_image`, `parse_label_text`, `LabelOCR.read`, `LabelDetector.detect`, `fuse(image,detections,ocr_reader)` — nhất quán giữa các task. ✓

**Phụ thuộc giữa task:** Task 6 cần weights từ Task 5 (cần dataset Task 3). Task 2/QR độc lập. PaddleOCR (Task 4) là fallback — pipeline vẫn chạy với QR nếu OCR lỗi.

**Rủi ro:** (1) cài `paddlepaddle` đôi khi khó trên GPU mới → có thể dùng bản CPU cho OCR (chậm, chấp nhận được cho POC). (2) YOLO trên tập synthetic 144 ảnh có thể overfit nhưng đủ cho POC 18 nhãn cố định. (3) QR là nguồn chính + rõ nét nên độ chính xác part_no cao ngay cả khi YOLO/OCR yếu.
