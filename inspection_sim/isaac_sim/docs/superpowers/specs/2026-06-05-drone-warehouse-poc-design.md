# Automated Cycle Count — Drone Warehouse POC (Isaac Sim 6.0)

**Ngày:** 2026-06-05
**Nguồn yêu cầu:** `CLAUDE.md`, `260312_SOR_AUTOMATED CYCLE COUNT.11.docx`
**Trạng thái:** Design (chờ duyệt → writing-plans)

## 1. Mục tiêu

Xây dựng demo POC cho bài toán *Automated Cycle Count*: drone tích hợp camera AI tự động bay tới một vị trí rack/BIN do operator chỉ định, quét nhãn GR để trích Part No./Qty, đối chiếu với dữ liệu SAP, và phát cảnh báo khi sai khác. Toàn bộ chạy mô phỏng trên **Isaac Sim 6.0**.

Phạm vi POC theo SOR: rack **3 cột × 6 tầng = 18 BIN**, cảnh báo discrepancy **sau 10 giây**, UI hiển thị trạng thái real-time từng vị trí.

## 2. Quyết định thiết kế đã chốt

| Hạng mục | Lựa chọn |
|----------|----------|
| Phạm vi kế hoạch | Toàn bộ POC end-to-end (decompose thành 6 module) |
| Mô phỏng drone | Quadrotor **có physics** (4 rotor + controller) |
| Localization | **Ground-truth pose (oracle)** từ sim, đặt sau interface `LocalizationProvider` để sau gắn VIO |
| Camera AI | **YOLO** (detect QR/bin) + **pyzbar** (decode QR) + **PaddleOCR** (đọc text) trên ảnh render RTX thật |
| Backend/SAP/UI | **Mock nhẹ** (SQLite + FastAPI) + **UI web** grid 3×6 real-time |
| Tích hợp | **Phương án A**: Sim trong process Isaac Sim, publish state qua **WebSocket** tới FastAPI backend riêng; UI web nối backend |
| Cài đặt | **conda env mới `isaac6`** (Python 3.11), không đụng IsaacLab 4.5 hiện có |

## 3. Hiện trạng môi trường

- **Chưa có Isaac Sim 6.0.** Có IsaacLab gắn Isaac Sim 4.5 (`~/Desktop/IsaacLab/apps/isaacsim_4_5`) và conda env `unitree_sim_env` (IsaacLab 0.54.3 editable) — **giữ nguyên, không đụng tới.**
- GPU: **RTX 5060 Ti 16GB (Blackwell)**, driver 580.159.03 — đủ điều kiện Isaac Sim 6.0.
- Conda envs: `base`, `deepfilter-gan`, `project`, `unitree_sim_env`, `vllm`.

## 4. Kiến trúc tổng thể (Phương án A)

```
UI (web) ──POST /inspect {bin_id}──> Backend (FastAPI)
                                        │
                                        ├──WS command──> Sim process (Isaac Sim 6.0)
                                        │                   ├─ Drone nav (M2) bay tới scan pose
                                        │                   └─ Camera capture ──> Perception (M3)
                                        │<──WS scan {bin_id, part_no, qty}──────────┘
                                        ├──query──> SAP Mock (M5, SQLite)
                                        ├──match? (rule 10s)──> Alert
                                        ├──save──> History (SQLite)
                                        └──WS push──> UI (grid + alert + history)
```

Hai process: (1) **Sim process** chạy trong Python của Isaac Sim 6.0; (2) **Backend+UI** chạy trong env Python thường. Giao tiếp qua WebSocket JSON. Tách bạch để demo/test từng phần độc lập và có thể nâng lên ROS 2 sau.

## 5. Module breakdown

### M1 — Sim Environment (nền tảng)

**Mục đích:** Sinh scene kho mô phỏng đầy đủ bằng script Python (Agent viết, procedural), không kéo-thả GUI.

- **Cài đặt:** env `isaac6` (Python 3.11), `isaacsim[all]==6.0.0`. Script verify mở `SimulationApp` headless và in version.
- **Scene builder** (`sim/scene_builder.py`) dùng `SimulationApp` + `pxr.Usd`/`omni.usd`:
  - Kho: nền + warehouse asset (SimReady/Warehouse01), ánh sáng, scale 0.01.
  - Rack 3×6 = 18 BIN; mỗi BIN đặt pallet + box (SimReady, có physics preset).
  - **Nhãn GR:** mỗi pallet gắn 1 quad/decal texture **QR + text** (Part No., Qty) sinh tự động; QR encode đúng Part No. để M3 decode thật. Gán semantic label cho prim.
  - Drone quadrotor + camera RGB đặt tại "home".
- **BIN Location Map** (`sim/bin_map.yaml`): mỗi `bin_id` (`A1..C6`) → pose pallet + **scan pose** (vị trí + hướng camera drone) + ground-truth `part_no`/`qty`.

**Kiểm thử M1:** headless render 1 frame, ảnh camera drone thấy rack + nhãn; `bin_map.yaml` đủ 18 BIN; QR trên ảnh decode được.

### M2 — Drone Navigation

**Mục đích:** Quadrotor physics bay tới BIN theo waypoint và hover ổn định để quét.

- Quadrotor: thân cứng + 4 rotor (thrust), controller 2 tầng (position PID → velocity; attitude PID → rotor thrust).
- **`LocalizationProvider` interface:** triển khai `OracleLocalization` (đọc pose ground-truth từ sim). Interface tách rời để sau gắn VIO không phải viết lại nav.
- **Waypoint planner:** `bin_id` → tra `bin_map.yaml` → chuỗi waypoint home → approach → scan pose (hover N giây) → home, với offset an toàn tránh va kệ.
- **State machine:** `IDLE / FLYING / SCANNING / RETURNING`, publish qua WebSocket.

**Kiểm thử M2:** lệnh bay tới `B3`, hover ổn định tại scan pose, sai số vị trí < ngưỡng đặt trước.

### M3 — AI Camera Perception

**Mục đích:** Trích Part No./Qty từ ảnh camera drone (render RTX thật, đã verify ở M1) tại trạng thái `SCANNING`.

Pipeline thị giác (theo yêu cầu):
1. **YOLO detection** — phát hiện vùng **QR code** và **BIN/nhãn** trong khung hình (bounding boxes). Cho phép định vị nhãn trong ảnh rộng, không phụ thuộc nhãn nằm chính giữa.
2. **QR decode (pyzbar)** trên vùng QR đã crop → `part_no, qty` (nguồn chính, encode JSON).
3. **PaddleOCR** đọc text trên vùng nhãn ("Part No: …", "Qty: …") → đối chiếu/khôi phục khi QR mờ/lỗi.
4. Hợp nhất kết quả → `{bin_id, part_no, qty, confidence, bbox, image}` trả backend.

- Model: YOLO (ultralytics) cho detect; PaddleOCR cho text. Cần cài vào python của binary Isaac hoặc chạy như service riêng nhận ảnh.
- Dữ liệu train/eval: render nhiều góc/khoảng cách từ M1 (18 nhãn) làm tập synthetic; nhãn ground-truth có sẵn từ `bin_map.yaml`.

**Kiểm thử M3:** trên ảnh render thật từ sim (vd `sim/assets/capture_A1.png`), YOLO khoanh đúng vùng QR/nhãn, pyzbar decode đúng `PN-A01`, PaddleOCR đọc đúng "PN-A01"/"11"; đo accuracy trên tập synthetic.

### M4 — Inspection Backend (FastAPI)

**Mục đích:** Điều phối toàn luồng + logic cảnh báo.

- API `POST /inspect {bin_id}` → ra lệnh drone (WS) → nhận scan (M3) → query SAP (M5) → so khớp.
- **Rule 10s (theo SOR):** nếu sau 10s từ lúc bắt đầu mà scan ≠ SAP, hoặc không nhận được kết quả → tạo **discrepancy alert**.
- Lưu **history** (SQLite). Push trạng thái + alert tới UI qua WebSocket.

**Kiểm thử M4:** BIN khớp → completed + history; BIN lệch (seed sẵn) → alert trong ≤10s.

### M5 — SAP Mock

**Mục đích:** Đóng vai SAP cung cấp tồn kho hệ thống.

- **SQLite** `backend/sap_mock.db`, bảng `inventory(bin_id, part_no, qty)`, seed từ `bin_map.yaml` nhưng **cố ý lệch vài BIN** để demo cảnh báo.
- API `GET /sap/inventory/{bin_id}`, `PUT /sap/inventory/{bin_id}` (adjust khi xác minh lại).

**Kiểm thử M5:** query trả đúng seed; PUT cập nhật và đọc lại đúng.

### M6 — Operator UI (Web)

**Mục đích:** Giao diện vận hành theo SOR.

- **Grid 3×6** màu theo trạng thái: xám (chưa quét) / vàng (đang quét) / xanh (khớp) / đỏ (lệch).
- Nhập `bin_id` → `POST /inspect`; nghe WebSocket cập nhật real-time + banner alert + bảng history.
- **RBAC tối thiểu:** chọn role Admin/User/Viewer ở demo (Viewer chỉ xem, User chạy inspect, Admin adjust SAP).

**Kiểm thử M6:** chạy inspect từ UI, ô đổi màu theo trạng thái, alert hiển thị khi lệch, history cập nhật.

## 6. Cấu trúc thư mục

```
Drone_poc/
├── sim/            # M1: scene_builder.py, assets/, bin_map.yaml
├── drone/          # M2: nav controller, localization/
├── perception/     # M3: ocr_qr.py
├── backend/        # M4 + M5: app.py, sap_mock.db, history
├── ui/             # M6: web (HTML/JS hoặc React nhẹ)
└── docs/superpowers/specs/
```

## 7. Thứ tự build & tích hợp

M1 → M2 → M3 → (M5 + M4) → M6, tích hợp dần. Mỗi module có kiểm thử riêng trước khi nối.

## 8. Phù hợp KPI/SOR

- Latency cảnh báo ≤ 10s: **rule 10s ở M4**.
- Trạng thái real-time từng vị trí: **grid 3×6 ở M6 qua WebSocket**.
- RBAC Admin/User/Viewer: **M6 mức tối thiểu**.
- BIN map 3×6: **M1 `bin_map.yaml`**.
- (Ngoài phạm vi POC: SAP thật, ISO/OSHA/an toàn phần cứng, uptime 95% — chỉ mock/ghi nhận.)

## 9. Rủi ro & giả định

- **Isaac Sim 6.0 pip install** trên Blackwell: nếu pip không sẵn cho 6.0.0, fallback bản binary/workstation hoặc container. Verify ngay ở bước đầu M1.
- **Quadrotor physics** dễ mất ổn định: tune PID + giới hạn vận tốc; scan pose hover có vùng dung sai.
- **OCR/QR** phụ thuộc độ phân giải camera + góc quét: chọn scan pose vuông góc nhãn, đủ gần; QR ưu tiên hơn OCR text.
- Asset SimReady cần truy cập NVIDIA assets server (Nucleus/online). Verify quyền truy cập sớm.
```
