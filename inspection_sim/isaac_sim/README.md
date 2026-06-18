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

---

# Quick start

If the machine already meets the [prerequisites](#prerequisites), the whole demo is **four commands**:

```bash
cd drone_poc_isaac_sim
scripts/setup_new_machine.sh                                  # one-time: conda envs, deps, assets, drone USD
conda run -n isaac6 python -m sim.bin_map                     # generate the 18-BIN map
conda run -n isaac6 python -m sim.gr_label                    # generate the 18 QR labels
scripts/run_live_demo.sh                                      # start Isaac + backend
```

Then open **http://localhost:8080/** — or, from another device on the same network,
**http://&lt;this-machine-ip&gt;:8080/** (see [Open the dashboard](#open-the-dashboard-this-machine-or-another-device)).

---

## Prerequisites

| Requirement | Notes |
| --- | --- |
| **Linux** (Ubuntu 22.04+ x86_64) | The Isaac Sim binary targets Linux. |
| **NVIDIA RTX GPU** + driver ≥ 535 | 8 GB VRAM is comfortable (built on an RTX 5060). The RTX path tracer is required for the QR render. |
| **~15 GB free system RAM** | Isaac peaks ~15 GB while booting. Close other heavy apps / other GPU jobs. |
| **NVIDIA Isaac Sim 6.0** (binary / workstation) | Download, unzip, run `./post_install.sh`. Install to `~/isaacsim` (or set `ISAAC_HOME`). The binary ships its **own Python 3.12** — you do *not* use conda for the Isaac parts. |
| **Miniconda / Anaconda** | `conda` on PATH (`conda init bash` once). |
| **Internet** (first run only) | downloads ~870 MB warehouse assets + pip wheels. |

Isaac Sim 6.0 download + install guide:
https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html

> The Isaac install is launched through `scripts/run_isaac.sh`, which wraps the bundled
> `~/isaacsim/python.sh` and sets the EULA + libzbar + `PYTHONPATH`. You never call `python.sh`
> directly.

## Setup (one time)

```bash
# conda 24+ blocks the default channels until you accept the Terms of Service once:
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/main
conda tos accept --override-channels --channel https://repo.anaconda.com/pkgs/r
```

### Option A — automated (recommended)

```bash
cd drone_poc_isaac_sim
scripts/setup_new_machine.sh
```

`setup_new_machine.sh` does everything from a clean machine:

1. Creates conda env **`isaac6`** (py3.12) + **`perception`** (py3.11), each with `zbar`.
2. Installs the **pinned** deps from `deploy/requirements-isaac6.txt` and
   `deploy/requirements-perception.txt` (numpy is pinned to `1.26.4` for Paddle; do not bump).
3. Symlinks **libzbar** into `~/.local/isaac_extra_libs` and installs `pyzbar`/`Pillow`/`opencv`
   into the Isaac binary's own Python (so QR decode works inside Isaac).
4. Copies the shipped **ANAFI drone USD** (`deploy/anafi_ai.usd`) into the asset mirror.
5. Downloads the **warehouse + props assets** (`scripts/download_assets.sh`, ~870 MB) into
   `~/isaacsim_assets`.

Override paths with env vars if your install lives elsewhere:

```bash
ISAAC_HOME=~/isaacsim ISAAC_ASSETS_LOCAL=~/isaacsim_assets scripts/setup_new_machine.sh
```

### Option B — manual (if the script fails midway, or you want control)

```bash
# 1. The two conda environments
conda create -y -n isaac6 python=3.12  && conda install -y -n isaac6 -c conda-forge zbar
conda run  -n isaac6 python -m pip install -r deploy/requirements-isaac6.txt

conda create -y -n perception python=3.11 && conda install -y -n perception -c conda-forge zbar
conda run  -n perception python -m pip install -r deploy/requirements-perception.txt

# 2. libzbar for the Isaac binary python (pyzbar QR decode)
mkdir -p ~/.local/isaac_extra_libs
ln -sf ~/miniconda3/envs/isaac6/lib/libzbar.so* ~/.local/isaac_extra_libs/
~/isaacsim/python.sh -m pip install qrcode==7.4.2 pyzbar==0.1.9 PyYAML Pillow opencv-python

# 3. Place the ANAFI drone USD into the asset mirror
mkdir -p ~/isaacsim_assets/Custom/ANAFI_Ai
cp deploy/anafi_ai.usd ~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd

# 4. Download warehouse/props assets
scripts/download_assets.sh
```

### Generate the scene assets (after setup)

```bash
conda run -n isaac6 python -m sim.bin_map     # → sim/bin_map.yaml (18 BINs)
conda run -n isaac6 python -m sim.gr_label    # → sim/assets/labels/*.png (18 QR labels)
```

## Run the demo

### One command (starts Isaac + backend together)

```bash
scripts/run_live_demo.sh             # Isaac GUI window + backend + UI
HEADLESS=1 scripts/run_live_demo.sh  # no Isaac window (server only) — a bit faster
```

- Isaac takes **~60–90 s** to boot (first run compiles shaders; later boots are fast).
- Log: `/tmp/live_sim.log`. Ready when `ss -ltn | grep 8765` shows `LISTEN`.
- `Ctrl-C` stops the backend and the Isaac child it started.

### Two terminals (more control)

```bash
# Terminal A — backend + UI  (uses the env python directly; no activation needed)
~/miniconda3/envs/perception/bin/python -m uvicorn backend.app:app --host 0.0.0.0 --port 8080
#   wait for: "Application startup complete"

# Terminal B — Isaac live engine (headless)
scripts/run_isaac.sh scripts/live_sim.py -- --headless
# --width 640 --height 360
#   ready when:  ss -ltn | grep 8765   (LISTENING)
#   to SEE the Isaac viewport, drop "--headless" but KEEP the trailing "--".
```

## Open the dashboard (this machine or another device)

| From | URL |
| --- | --- |
| The machine running it | **http://localhost:8080/** |
| Another device on the LAN (laptop/phone) | **http://&lt;server-ip&gt;:8080/** — get the IP with `hostname -I` |

The backend binds `0.0.0.0`, so it is reachable across the network. **Two ports must be
reachable** from the viewing device: **8080** (dashboard + REST) and **8765** (live camera +
telemetry WebSocket). Open both in the firewall if the page loads but the camera/3D map stay
blank. `ui/app.js` derives the WebSocket host from the page URL, so the remote browser
connects automatically.

> **The 3D map needs hardware WebGL.** Use a normal GPU-backed browser (Chrome/Edge/Firefox).
> A remote desktop with *software* GL (e.g. a VNC session using `llvmpipe`) shows
> "WebGL unavailable" for the 3D panel only — the camera feed and everything else still work.
> Viewing from another physical device's browser avoids this entirely.

**Using it:** pick a role → type a BIN (e.g. `B3`) or click a grid cell → the drone flies the
aisle, scans the QR, and the result appears. **B2 / C4 / A5** are seeded discrepancies and
alert red. "Inspect All" sweeps all 18 BINs; "Return Home" parks the drone.

## Troubleshooting

| Symptom | Cause & fix |
| --- | --- |
| **3D map: drone doesn't move / BINs don't change color** | The drone is **parked while IDLE** — it only flies and recolors a BIN when you **issue an inspect** (click a cell or type a BIN). Also: after a code update, **hard-refresh** the browser (Ctrl-Shift-R) to drop a cached `app.js`, and check the header **Isaac dot is green** (the `ws://…:8765` connection is live). |
| **3D map: "WebGL unavailable"** | Software-GL desktop (VNC `llvmpipe`). Open the dashboard from a real-GPU browser / another device. The rest of the UI is unaffected. |
| **Low / declining FPS** | The render is **GPU-bound**. Don't run another GPU job at the same time (a second Isaac/render app roughly halves FPS). Headless is faster than the GUI window. ~9 FPS at 960×540 is the practical ceiling on an RTX 5060. |
| **QR scan reads nothing / `dets=0`** | Make sure the labels were generated (`sim/assets/labels/*.png`). The live scan now falls back to compositing the real label texture, so a decode is guaranteed once the drone reaches the scan pose. |
| **The QR-result frame flashes by too fast to see** | The drone lingers at the scan pose `--scan-linger` seconds (default **5**) after decoding, holding the QR overlay on screen. Increase it for slower demos, e.g. `scripts/run_isaac.sh scripts/live_sim.py --portable-root ~/.cache/isaac_sim_portable -- --headless --scan-linger 8`. |
| **Isaac boot hangs at ~15 GB with no error** | Only on a **shared / read-only** Isaac install: Isaac's `--portable` cache can't get a write lock. Add `--portable-root ~/.cache/isaac_sim_portable` **before** the `--`: `scripts/run_isaac.sh scripts/live_sim.py --portable-root ~/.cache/isaac_sim_portable -- --headless`. Not needed on an install you own. |
| **Isaac killed (OOM) on boot** | Free RAM — Isaac needs ~15 GB. Close other heavy apps / other Isaac jobs. |
| **`conda create` fails with a Terms-of-Service error** | Run the two `conda tos accept …` commands in [Setup](#setup-one-time). |
| **`Address already in use` on 8080/8765** | A stale process. `scripts/run_live_demo.sh` frees the ports automatically; otherwise `pkill -f scripts/live_sim.py` and re-run. |

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

Non-live / inspection-only demo and direct scene inspection:

```bash
scripts/run_demo.sh                                     # REST inspect on pre-rendered captures
scripts/run_isaac.sh -m sim.scene_builder -- --gui      # open warehouse + aisle + loaded racks + drone
scripts/run_isaac.sh scripts/fly_to_bin.py B3           # physics flight to a BIN and back
scripts/run_isaac.sh scripts/fly_aisle_demo.py C3       # narrow-aisle flight + obstacle-avoidance metrics
```

## Layout

```
sim/         M1 scene (config, bin_map, gr_label, warehouse, rack, drone_asset, obstacles, scene_builder)
drone/       M2 navigation (waypoints, controller, localization, quadrotor, flight, avoidance)
perception/  M3 (detector=qrdet, qr, ocr=PaddleOCR, pipeline, types)
backend/     M4+M5 (app=FastAPI, inspection, sap_mock, udp_bridge, visualize)
ui/          M6 operator console — 3-zone dashboard (index.html, app.js, style.css)
scripts/     run_isaac.sh, run_live_demo.sh, run_demo.sh, setup_new_machine.sh, live_sim.py,
             download_assets.sh, convert_drone_cad.py, render_all_bins.py, fly_to_bin.py,
             fly_aisle_demo.py, verify_*
deploy/      anafi_ai.usd + requirements-isaac6.txt + requirements-perception.txt
docs/superpowers/  specs, plans, INSTALL_NOTES (real-world gotchas)
```

## KPIs

- Discrepancy alert latency ≤ 10 s (measured per inspection).
- Real-time per-BIN status (3×6 grid + live camera + 3D twin over WebSocket).
- Narrow-aisle flight without collision (APF avoidance; reported clearance/collision metrics).
- RBAC Admin/User/Viewer via the UI role selector.
- Real inventory/ERP integration, hardware-safety certification, and uptime targets are out
  of POC scope (mocked / recorded).
