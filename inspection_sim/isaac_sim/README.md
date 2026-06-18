# Automated Cycle Count — Isaac Sim POC

An interactive **NVIDIA Isaac Sim 6.0** proof-of-concept for the warehouse cycle-count
mission: an AI-camera drone flies a narrow warehouse aisle to a specified rack/BIN, scans
the GR/QR label, the system reads Part No./Qty and compares against inventory, and raises a
discrepancy alert on mismatch. The operator drives a live, running simulation from the
browser and watches the drone camera, a 3D digital twin, and the LiDAR point cloud.

POC scope: a **3 columns × 6 levels = 18 BIN** rack, discrepancy alert within the **10 s**
KPI, per-BIN real-time status, and narrow-aisle flight with obstacle avoidance.

## What the demo shows

- A warehouse scene: an 18-BIN shelving rack of cartons carrying emissive **QR/GR labels**,
  a second rack forming a **narrow aisle**, and a floor obstacle.
- A physics drone with an onboard camera and an **RTX LiDAR**.
- **Live interaction**: operator selects a BIN → drone flies the aisle (with obstacle
  avoidance) → camera scans the QR → compare vs inventory → `completed` or **discrepancy
  alert** → history logged.
- **Live visualization** in the browser: drone camera video (QR-detection overlay), a 3D
  digital-twin map (racks, status-colored BINs, moving drone, LiDAR cloud), the BIN grid,
  alerts, and history.

## Modules

| Module | Role |
| --- | --- |
| **M1 Sim environment** | Isaac warehouse + 3×6 rack with QR/GR labels, second rack + narrow aisle + obstacle, drone + camera + LiDAR. |
| **M2 Drone navigation** | Physics quadrotor + APF obstacle avoidance; home→approach→scan→home state machine; localization behind a swappable interface (VIO-ready). |
| **M3 Perception** | QR detector → decode (authoritative) → OCR text fallback. Runs on the camera's RTX render. |
| **M4 Inspection backend** | FastAPI: inspect → compare → 10 s rule → history → WebSocket push; serves the UI and the 3D scene. |
| **M5 Inventory mock** | SQLite inventory seeded with intentional discrepancies to demo alerts. |
| **M6 Operator UI** | 3-zone web console: BIN input/grid/RBAC, live camera + 3D twin, alert bar. |

## BIN map: predefined, no SLAM

The BIN location map (`sim/bin_map.yaml`, 18 BINs) is a predefined/surveyed map of known
rack positions — the drone navigates to known BIN coordinates, so SLAM is not required for
this POC. Drone localization is stubbed behind a swappable interface; the RTX LiDAR still
produces a real point cloud for the live 3D map.


### 1. Conda + the two environments
```bash
# Install Miniconda if needed, then put `conda` on PATH for every new shell:
#   bash Miniconda3-latest-Linux-x86_64.sh -b -p ~/miniconda3
~/miniconda3/bin/conda init bash && exec bash

# conda 24+ requires accepting the default-channel Terms of Service once:
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r

# isaac6  — pure-Python asset generation + provides libzbar for QR decode
conda create -n isaac6 python=3.12 zbar pip -c conda-forge -y
conda run  -n isaac6 pip install qrcode PyYAML Pillow pytest

# perception — FastAPI backend + perception pipeline (YOLO-QR → pyzbar → PaddleOCR)
#   NOTE: environment.perception.yml is incomplete; install the pinned deps explicitly:
conda create -n perception python=3.11 zbar pip -c conda-forge -y
conda run  -n perception pip install numpy==1.26.4                                        # paddle needs numpy<2
conda run  -n perception pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
conda run  -n perception pip install paddlepaddle==2.6.2 paddleocr==2.7.3                 # known-good pair
conda run  -n perception pip install fastapi "uvicorn[standard]" websockets pydantic \
    opencv-python pyzbar qrcode qrdet imageio imageio-ffmpeg PyYAML Pillow
```

### 2. Wire Isaac into the project (assets, drone USD, QR libs)
```bash
cd /path/to/drone_poc_isaac_sim

# Local asset mirror at ~/isaacsim_assets (Simple_Warehouse env, props, RTX lidar):
scripts/download_assets.sh
#   Already have an Isaac 6.0 asset pack (e.g. a shared .../Assets/Isaac/6.0)? Instead of
#   re-downloading, symlink its subtrees in:
#     ln -sfn <pack>/Isaac  ~/isaacsim_assets/Isaac   &&   ln -sfn <pack>/NVIDIA ~/isaacsim_assets/NVIDIA

# The ANAFI drone USD (prebuilt in deploy/) must live under the asset mirror:
mkdir -p ~/isaacsim_assets/Custom/ANAFI_Ai
ln -sfn "$PWD/deploy/anafi_ai.usd"  ~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd

# QR decode inside Isaac's bundled python (pyzbar + libzbar from the isaac6 env):
~/isaacsim/python.sh -m pip install --user pyzbar
mkdir -p ~/.local/isaac_extra_libs
ln -sfn ~/miniconda3/envs/isaac6/lib/libzbar.so.0.3.0  ~/.local/isaac_extra_libs/libzbar.so.0.3.0
ln -sfn libzbar.so.0.3.0  ~/.local/isaac_extra_libs/libzbar.so.0
ln -sfn libzbar.so.0.3.0  ~/.local/isaac_extra_libs/libzbar.so
```

### 3. Generate scene assets (once)
```bash
conda run -n isaac6 python -m sim.bin_map     # → sim/bin_map.yaml (18 BINs)
conda run -n isaac6 python -m sim.gr_label    # → sim/assets/labels/*.png (18 QR labels)
```

## Run the live demo

Two processes — the **backend + Operator UI** (`http://localhost:8080`) and the **Isaac live engine**
(WebSocket `ws://localhost:8765`). Run each in its own terminal:

```bash
# Terminal A — backend + UI  (no conda activation needed; uses the env's python directly)
~/miniconda3/envs/perception/bin/python -m uvicorn backend.app:app --host 0.0.0.0 --port 8080
#   wait for: "Application startup complete"

# Terminal B — Isaac live engine (headless). First boot compiles shaders (~1-3 min).
scripts/run_isaac.sh scripts/live_sim.py --portable-root ~/.cache/isaac_sim_portable -- --headless
#   ready when:  ss -ltn | grep 8765      (LISTENING)
#   to SEE the Isaac window, drop "--headless" but KEEP the trailing "--":
#   scripts/run_isaac.sh scripts/live_sim.py --portable-root ~/.cache/isaac_sim_portable --
```

Open **http://localhost:8080/** → pick a role → type a BIN (e.g. `B3`) or click a cell to fly the
drone. **B2 / C4 / A5** are seeded discrepancies and alert red. Stop with `Ctrl-C` in each terminal
(or `pkill -f scripts/live_sim.py; pkill -f "uvicorn backend.app"`).

> **`--portable-root` is required on a read-only / shared Isaac install.** Isaac's default
> `--portable` mode writes its shader cache + `omni.datastore` *inside* the install dir; if you don't
> own it, the datastore can't get a write lock and **boot hangs (~15 GB RAM, no error)**. Pointing
> `--portable-root` at a writable dir fixes it and persists shaders for fast reboots. Add the same flag
> to **every** `scripts/run_isaac.sh …` command (including the verify / inspection ones below). On a
> private install you own, you may omit it.
>
> **Also:** `conda` must be on PATH (`conda init bash` once) or prefix commands with
> `~/miniconda3/bin/conda`. `scripts/run_live_demo.sh` is a one-shot launcher but does **not** pass
> `--portable-root`, so on a shared install prefer the two-terminal commands above.

Non-live / inspection-only demo and direct scene inspection:

```bash
scripts/run_demo.sh                                     # REST inspect on pre-rendered captures
scripts/run_isaac.sh -m sim.scene_builder -- --gui      # open warehouse + aisle + loaded racks + drone
scripts/run_isaac.sh scripts/fly_to_bin.py B3           # physics flight to a BIN and back
scripts/run_isaac.sh scripts/fly_aisle_demo.py C3       # narrow-aisle flight + obstacle-avoidance metrics
```

## Verify each module

```bash
conda run -n isaac6   python -m pytest tests/ -q            # sim + nav + obstacle/avoidance unit tests
conda run -n perception python -m pytest tests/test_qr.py tests/test_pipeline.py \
    tests/test_ocr_parse.py tests/test_sap_mock.py tests/test_inspection.py tests/test_udp_bridge.py -q
scripts/run_isaac.sh scripts/verify_scene.py               # SCENE_OK bins=18
scripts/run_isaac.sh scripts/verify_capture.py             # CAPTURE_QR_OK (genuine render)
conda run -n perception python scripts/verify_perception.py # PERCEPTION_OK part_no=PN-A01
scripts/run_isaac.sh scripts/verify_quadrotor.py           # HOVER_OK err<0.3
scripts/run_isaac.sh scripts/fly_aisle_demo.py C3          # AISLE_DEMO reached=True min_clearance>0 collisions=0
```

## Layout

```
sim/         M1 scene (config, bin_map, gr_label, warehouse, rack, drone_asset, obstacles, scene_builder)
drone/       M2 navigation (waypoints, controller, localization, quadrotor, flight, avoidance)
perception/  M3 (detector=qrdet, qr, ocr=PaddleOCR, pipeline, types)
backend/     M4+M5 (app=FastAPI, inspection, sap_mock, udp_bridge, visualize)
ui/          M6 operator console — 3-zone dashboard (index.html, app.js, style.css)
scripts/     run_isaac.sh, run_live_demo.sh, run_demo.sh, live_sim.py, download_assets.sh,
             convert_drone_cad.py, render_all_bins.py, fly_to_bin.py, fly_aisle_demo.py, verify_*
docs/superpowers/  specs, plans, INSTALL_NOTES (real-world gotchas)
```

## KPIs

- Discrepancy alert latency ≤ 10 s (measured per inspection).
- Real-time per-BIN status (3×6 grid + live camera + 3D twin over WebSocket).
- Narrow-aisle flight without collision (APF avoidance; reported clearance/collision metrics).
- RBAC Admin/User/Viewer via the UI role selector.
- Real inventory/ERP integration, hardware-safety certification, and uptime targets are out
  of POC scope (mocked / recorded).