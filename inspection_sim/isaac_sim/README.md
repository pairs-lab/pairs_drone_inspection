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

## Run

Isaac Sim runs on its bundled Python; the backend + perception run in a conda env. After
setup (Isaac Sim 6.0 binary install, the conda envs, and generating the scene assets), the
demo runs as two processes — the backend + Operator UI and the Isaac live engine:

```bash
# backend + UI  ->  http://localhost:8080
python -m uvicorn backend.app:app --host 0.0.0.0 --port 8080

# Isaac live engine (WebSocket ws://localhost:8765)
scripts/run_isaac.sh scripts/live_sim.py -- --headless
```

Open `http://localhost:8080/`, pick a role, and select a BIN to fly the drone. Seeded
discrepancy BINs raise red alerts. See `scripts/` for the non-live / inspection-only demos
and per-module verification.

## Layout

```
sim/         M1 scene (config, bin_map, gr_label, warehouse, rack, drone, obstacles, scene_builder)
drone/       M2 navigation (waypoints, controller, localization, quadrotor, flight, avoidance)
perception/  M3 (detector, qr, ocr, pipeline, types)
backend/     M4+M5 (FastAPI app, inspection, inventory mock, visualize)
ui/          M6 operator console (index.html, app.js, style.css)
scripts/     run_isaac.sh, live_sim.py, fly_to_bin.py, fly_aisle_demo.py, verify_*
```

## KPIs

- Discrepancy alert latency ≤ 10 s (measured per inspection).
- Real-time per-BIN status (3×6 grid + live camera + 3D twin over WebSocket).
- Narrow-aisle flight without collision (APF avoidance; reported clearance/collision metrics).
- RBAC Admin/User/Viewer via the UI role selector.
- Real inventory/ERP integration, hardware-safety certification, and uptime targets are out
  of POC scope (mocked / recorded).
