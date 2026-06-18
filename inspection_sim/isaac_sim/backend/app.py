"""M4 — Inspection Backend (FastAPI).

Runs in the `perception` conda env so it can import perception modules directly.

Start with:
    conda run -n perception python -m uvicorn backend.app:app --port 8000

Endpoints
---------
GET  /api/bins                     -> bin grid layout (3 cols x 6 levels)
POST /api/inspect/{bin_id}         -> run inspection, return InspectionResult JSON
GET  /api/history                  -> recent inspection records
GET  /api/sap/{bin_id}             -> get SAP inventory record
PUT  /api/sap/{bin_id}             -> adjust SAP record (re-verify)
WS   /ws                           -> push inspection results / alerts to UI
GET  /api/live_stream              -> UDP→MJPEG browser bridge (multipart/x-mixed-replace)
GET  /api/live_status              -> {"streaming": bool} UDP probe
GET  /                             -> serve ui/ static files

Architecture notes
------------------
- Perception models (QRCodeDetector, LabelOCR) are lazy-loaded once at startup.
- SAP mock is seeded on startup.
- History is stored in SQLite (backend/history.db, same dir as sap_mock.db).
- WebSocket broadcast: every POST /api/inspect/{bin_id} pushes the JSON result
  to all connected WS clients so the UI can update in real-time.
- Latency KPI: the POST handler measures wall-clock time from start to result
  and logs a WARNING if >= 10 s (SOR requirement: alert within 10 s).
"""

from __future__ import annotations

import asyncio
import json
import logging
import os
import sqlite3
import subprocess
import sys
import time
from contextlib import asynccontextmanager
from dataclasses import asdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Set

import numpy as np
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from PIL import Image
from pydantic import BaseModel

# ---------------------------------------------------------------------------
# Paths (resolved relative to repo root, i.e. cwd when running uvicorn)
# ---------------------------------------------------------------------------
REPO_ROOT = Path(os.getcwd())
SAP_DB = str(REPO_ROOT / "backend" / "sap_mock.db")
HISTORY_DB = str(REPO_ROOT / "backend" / "history.db")
CAPTURES_DIR = REPO_ROOT / "sim" / "assets" / "captures"
ANNOTATED_DIR = REPO_ROOT / "sim" / "assets" / "annotated"
MISSION_VIDEO_PATH = REPO_ROOT / "sim" / "assets" / "mission.mp4"
MISSION_FRAMES_DIR = REPO_ROOT / "sim" / "assets" / "mission_frames"
MISSION_TRAJ_PATH  = REPO_ROOT / "backend" / "mission_trajectory.json"
UI_DIR = REPO_ROOT / "ui"

# ---------------------------------------------------------------------------
# UDP→MJPEG bridge config (overridable via environment variables)
# ---------------------------------------------------------------------------
UDP_HOST = os.environ.get("LIVE_UDP_HOST", "127.0.0.1")
UDP_PORT = int(os.environ.get("LIVE_UDP_PORT", "5600"))

# ---------------------------------------------------------------------------
# Logging
# ---------------------------------------------------------------------------
logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
log = logging.getLogger("backend.app")

# ---------------------------------------------------------------------------
# Global state: perception models (lazy-loaded at startup)
# ---------------------------------------------------------------------------
_qr_detector = None
_ocr_reader = None
_bin_map: Dict[str, Any] = {}

# WebSocket clients
_ws_clients: Set[WebSocket] = set()


# ---------------------------------------------------------------------------
# Lifespan (startup / shutdown)
# ---------------------------------------------------------------------------
@asynccontextmanager
async def lifespan(app: FastAPI):
    """Startup: seed SAP, load bin_map, init perception models."""
    global _qr_detector, _ocr_reader, _bin_map

    log.info("Starting up — seeding SAP mock ...")
    from backend.sap_mock import seed_from_bin_map, DISCREPANCY_BINS
    n = seed_from_bin_map(db_path=SAP_DB)
    log.info(f"SAP seeded: {n} rows (0 means already seeded). Discrepancy bins: {list(DISCREPANCY_BINS)}")

    log.info("Loading bin_map ...")
    from sim.bin_map import load_bin_map
    _bin_map = load_bin_map()
    log.info(f"bin_map loaded: {len(_bin_map)} bins")

    log.info("Initializing perception models ...")
    try:
        from perception.detector import QRCodeDetector
        _qr_detector = QRCodeDetector(model_size="n", conf=0.3)
        log.info("QRCodeDetector loaded")
    except Exception as e:
        log.warning(f"QRCodeDetector unavailable: {e}")
        _qr_detector = None

    try:
        from perception.ocr import LabelOCR
        _ocr_reader = LabelOCR(lang="en")
        # Trigger model download on startup
        _ocr_reader._ensure()
        log.info("LabelOCR loaded")
    except Exception as e:
        log.warning(f"LabelOCR unavailable: {e}")
        _ocr_reader = None

    _ensure_history_db()
    log.info("Backend ready.")

    yield  # --- app running ---

    log.info("Shutting down backend.")


# ---------------------------------------------------------------------------
# App
# ---------------------------------------------------------------------------
app = FastAPI(title="Drone Warehouse Inspection Backend", version="0.1.0", lifespan=lifespan)


# ---------------------------------------------------------------------------
# History SQLite helpers
# ---------------------------------------------------------------------------
def _ensure_history_db():
    conn = sqlite3.connect(HISTORY_DB)
    conn.execute(
        """
        CREATE TABLE IF NOT EXISTS history (
            id           INTEGER PRIMARY KEY AUTOINCREMENT,
            bin_id       TEXT NOT NULL,
            scanned_part TEXT,
            scanned_qty  INTEGER,
            system_part  TEXT,
            system_qty   INTEGER,
            match        INTEGER NOT NULL,
            status       TEXT NOT NULL,
            latency_s    REAL NOT NULL,
            timestamp    TEXT NOT NULL
        )
        """
    )
    conn.commit()
    conn.close()


def _save_history(result) -> None:
    conn = sqlite3.connect(HISTORY_DB)
    conn.execute(
        """
        INSERT INTO history
            (bin_id, scanned_part, scanned_qty, system_part, system_qty,
             match, status, latency_s, timestamp)
        VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
        """,
        (
            result.bin_id,
            result.scanned_part,
            result.scanned_qty,
            result.system_part,
            result.system_qty,
            int(result.match),
            result.status,
            result.latency_s,
            result.timestamp,
        ),
    )
    conn.commit()
    conn.close()


def _fetch_history(limit: int = 50) -> List[Dict]:
    conn = sqlite3.connect(HISTORY_DB)
    conn.row_factory = sqlite3.Row
    rows = conn.execute(
        "SELECT * FROM history ORDER BY id DESC LIMIT ?", (limit,)
    ).fetchall()
    conn.close()
    return [dict(r) for r in rows]


# ---------------------------------------------------------------------------
# WebSocket broadcast
# ---------------------------------------------------------------------------
async def _broadcast(payload: dict) -> None:
    msg = json.dumps(payload)
    disconnected = set()
    for ws in _ws_clients:
        try:
            await ws.send_text(msg)
        except Exception:
            disconnected.add(ws)
    _ws_clients.difference_update(disconnected)


# ---------------------------------------------------------------------------
# Endpoints
# ---------------------------------------------------------------------------

@app.get("/api/bins")
def get_bins():
    """Return bin grid layout: 3 cols (A, B, C) x 6 levels (1-6)."""
    from sim.config import COLUMNS, LEVELS
    grid = []
    for col in COLUMNS:
        for level in LEVELS:
            bid = f"{col}{level}"
            grid.append({
                "bin_id": bid,
                "column": col,
                "level": level,
                "part_no": _bin_map.get(bid, {}).get("part_no"),
                "qty": _bin_map.get(bid, {}).get("qty"),
            })
    return {"bins": grid, "columns": COLUMNS, "levels": list(LEVELS)}


@app.post("/api/inspect/{bin_id}")
async def inspect_bin(bin_id: str):
    """Run a full inspection for the given bin.

    1. Load the pre-rendered capture image.
    2. Run perception pipeline (QR + OCR).
    3. Fetch SAP record.
    4. Compare → InspectionResult.
    5. Assert latency < 10 s (SOR KPI).
    6. Save to history, broadcast over WS.
    7. Return JSON.
    """
    bin_id = bin_id.upper()
    if bin_id not in _bin_map:
        raise HTTPException(status_code=404, detail=f"Unknown bin_id: {bin_id}")

    t0 = time.monotonic()

    # -- 1. Load capture image --
    capture_path = CAPTURES_DIR / f"{bin_id}.png"
    if not capture_path.exists():
        # Fallback: try the pre-existing label image (for testing without Isaac)
        capture_path = REPO_ROOT / "sim" / "assets" / "labels" / f"{bin_id}.png"
    if not capture_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"No capture image for {bin_id}. Run render_all_bins.py first.",
        )

    image_rgb = np.array(Image.open(capture_path).convert("RGB"))

    # -- 2. Perception --
    detections = []
    if _qr_detector is not None:
        try:
            detections = _qr_detector.detect(image_rgb)
        except Exception as e:
            log.warning(f"QRCodeDetector failed on {bin_id}: {e}")

    # Run pipeline — also capture OCR lines for the annotated overlay
    ocr_lines: list = []
    from perception.pipeline import fuse
    from perception.qr import decode_qr_crop
    perception_result = fuse(image_rgb, detections, ocr_reader=_ocr_reader)

    # Capture OCR lines from the best detection (same logic as pipeline, no re-run)
    if _ocr_reader is not None and detections:
        best_det = detections[0]
        try:
            _, _, ocr_lines = _ocr_reader.read(image_rgb, best_det.bbox)
            if ocr_lines is None:
                ocr_lines = []
        except Exception as e:
            log.warning(f"OCR line capture failed for {bin_id}: {e}")

    # -- 3. SAP lookup --
    from backend.sap_mock import get_inventory
    sap_record = get_inventory(bin_id, db_path=SAP_DB)

    # -- 4. Compare (preliminary, without annotated_url yet) --
    latency_s = time.monotonic() - t0
    from backend.inspection import inspect_from_perception
    result = inspect_from_perception(
        bin_id=bin_id,
        perception_result=perception_result,
        sap_record=sap_record,
        latency_s=latency_s,
    )

    # -- 4b. Annotate capture --
    annotated_url: Optional[str] = None
    try:
        from backend.visualize import annotate_capture
        annotate_capture(
            bin_id=bin_id,
            perception_result=perception_result,
            detections=detections,
            ocr_lines=ocr_lines,
            status=result.status,
        )
        annotated_url = f"/api/annotated/{bin_id}"
        # Patch annotated_url into the result (dataclass is not frozen)
        result.annotated_url = annotated_url
        log.info(f"Annotated frame saved for {bin_id}: {annotated_url}")
    except Exception as e:
        log.warning(f"annotate_capture failed for {bin_id}: {e}")

    # -- 5. SOR KPI: latency < 10 s --
    if result.latency_s >= 10.0:
        log.warning(
            f"SOR KPI BREACH: {bin_id} inspection latency {result.latency_s:.2f}s >= 10s"
        )
    else:
        log.info(f"{bin_id}: {result.status} in {result.latency_s:.3f}s")

    # -- 6. History + broadcast --
    _save_history(result)
    payload = asdict(result)
    # Ensure bbox is JSON-serializable (tuple -> list)
    if payload.get("bbox") and not isinstance(payload["bbox"], list):
        payload["bbox"] = list(payload["bbox"])
    await _broadcast({"event": "inspection", "data": payload})

    if result.status == "discrepancy":
        await _broadcast({
            "event": "alert",
            "bin_id": bin_id,
            "message": (
                f"DISCREPANCY at {bin_id}: "
                f"scanned=({result.scanned_part}, qty={result.scanned_qty}) "
                f"vs SAP=({result.system_part}, qty={result.system_qty})"
            ),
            "data": payload,
        })

    # -- 7. Return --
    return payload


@app.get("/api/history")
def get_history(limit: int = 50):
    """Return recent inspection history (newest first)."""
    return {"history": _fetch_history(limit=limit)}


@app.get("/api/sap/{bin_id}")
def get_sap(bin_id: str):
    """Get current SAP inventory record for a bin."""
    bin_id = bin_id.upper()
    from backend.sap_mock import get_inventory
    record = get_inventory(bin_id, db_path=SAP_DB)
    if record is None:
        raise HTTPException(status_code=404, detail=f"SAP record not found for {bin_id}")
    return record


class SapUpdate(BaseModel):
    part_no: str
    qty: int


@app.put("/api/sap/{bin_id}")
def update_sap(bin_id: str, body: SapUpdate):
    """Update SAP inventory record (Admin: after physical re-verification)."""
    bin_id = bin_id.upper()
    from backend.sap_mock import set_inventory
    set_inventory(bin_id, body.part_no, body.qty, db_path=SAP_DB)
    log.info(f"SAP updated: {bin_id} -> part_no={body.part_no}, qty={body.qty}")
    return {"bin_id": bin_id, "part_no": body.part_no, "qty": body.qty, "updated": True}


@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    """WebSocket endpoint: push inspection results and alerts to connected UIs."""
    await ws.accept()
    _ws_clients.add(ws)
    log.info(f"WS client connected (total={len(_ws_clients)})")
    try:
        while True:
            # Keep connection alive; client can send pings
            await ws.receive_text()
    except WebSocketDisconnect:
        pass
    finally:
        _ws_clients.discard(ws)
        log.info(f"WS client disconnected (total={len(_ws_clients)})")


@app.get("/api/capture/{bin_id}")
def get_capture(bin_id: str):
    """Serve the raw drone camera capture image for a bin.

    Cache-busting: client should append ?t=<timestamp> to get fresh bytes after
    each inspection cycle (file is overwritten in-place per inspect).
    """
    bin_id = bin_id.upper()
    capture_path = CAPTURES_DIR / f"{bin_id}.png"
    if not capture_path.exists():
        # Fallback to label image
        capture_path = REPO_ROOT / "sim" / "assets" / "labels" / f"{bin_id}.png"
    if not capture_path.exists():
        raise HTTPException(status_code=404, detail=f"No capture image for {bin_id}")
    return FileResponse(
        str(capture_path),
        media_type="image/png",
        headers={"Cache-Control": "no-store"},
    )


@app.get("/api/annotated/{bin_id}")
def get_annotated(bin_id: str):
    """Serve the annotated drone camera frame (detection overlay) for a bin.

    Returns 404 if the bin has not been inspected yet (no annotated file).
    Cache-busting: client should append ?t=<timestamp> to force reload after
    each new inspection run overwrites the file.
    """
    bin_id = bin_id.upper()
    annotated_path = ANNOTATED_DIR / f"{bin_id}.png"
    if not annotated_path.exists():
        raise HTTPException(
            status_code=404,
            detail=f"No annotated frame for {bin_id}. Run an inspection first.",
        )
    return FileResponse(
        str(annotated_path),
        media_type="image/png",
        headers={"Cache-Control": "no-store"},
    )


# ---------------------------------------------------------------------------
# Scene geometry endpoint
# ---------------------------------------------------------------------------

@app.get("/api/scene")
def get_scene():
    """Return 3D scene layout for the browser digital-twin.

    Reads geometry constants directly (no Isaac Sim import needed).
    Returns:
      - warehouse bounds
      - primary and secondary racks as boxes (center + half_extents)
      - aisle description
      - floor obstacle
      - all 18 bins (id, column, level, world position, box size, label face)
    """
    import yaml

    # -- load bin_map from YAML directly (avoids heavy Isaac deps) --
    bin_map_path = REPO_ROOT / "sim" / "bin_map.yaml"
    try:
        with open(bin_map_path) as f:
            bm = yaml.safe_load(f)
    except FileNotFoundError:
        raise HTTPException(status_code=500, detail=f"bin_map.yaml not found at {bin_map_path}")

    # -- geometry constants (replicated from sim/config.py; no Isaac import) --
    COLUMNS         = ["A", "B", "C"]
    LEVELS          = [1, 2, 3, 4, 5, 6]
    COLUMN_SPACING  = 1.2
    LEVEL_HEIGHT    = 0.8
    BOX_W           = 0.70
    BOX_D           = 0.50
    BOX_H           = 0.50
    RACK_X_MIN      = -0.5
    RACK_X_MAX      =  2.9
    RACK_Y_MIN      = -0.45
    RACK_Y_MAX      =  0.45
    RACK_TOTAL_H    = len(LEVELS) * LEVEL_HEIGHT   # 4.8 m
    SECOND_RACK_Y   = -1.80
    AISLE_WIDTH     = 1.30
    AISLE_CENTER_Y  = -0.90
    OBS_X           =  1.20
    OBS_Y           = -0.90
    OBS_HALF_W      =  0.35
    OBS_HALF_D      =  0.35
    OBS_HALF_H      =  0.30

    bins = []
    for col_idx, col in enumerate(COLUMNS):
        for level in LEVELS:
            bid = f"{col}{level}"
            b = bm.get(bid, {})
            sp = b.get("scan_pose", {}).get("position", [0, 0, 0])
            pp = b.get("pallet_pose", {}).get("position", [0, 0, 0])
            # box world center
            bx = col_idx * COLUMN_SPACING
            by = 0.0
            bz = (level - 1) * LEVEL_HEIGHT + BOX_H / 2
            bins.append({
                "bin_id":        bid,
                "column":        col,
                "level":         level,
                "world_pos":     [round(bx, 4), round(by, 4), round(bz, 4)],
                "box_size":      [BOX_W, BOX_D, BOX_H],
                "label_face_dir": [0, -1, 0],   # labels face -Y
                "scan_pos":      [round(float(sp[0]), 4), round(float(sp[1]), 4), round(float(sp[2]), 4)],
                "part_no":       b.get("part_no"),
                "qty":           b.get("qty"),
            })

    scene = {
        "warehouse": {
            "x_min": -5.0, "x_max": 15.0,
            "y_min": -5.0, "y_max":  5.0,
            "z_min":  0.0, "z_max": 10.0,
        },
        "primary_rack": {
            "center": [
                round((RACK_X_MIN + RACK_X_MAX) / 2, 4),
                round((RACK_Y_MIN + RACK_Y_MAX) / 2, 4),
                round(RACK_TOTAL_H / 2, 4),
            ],
            "half_extents": [
                round((RACK_X_MAX - RACK_X_MIN) / 2, 4),
                round((RACK_Y_MAX - RACK_Y_MIN) / 2, 4),
                round(RACK_TOTAL_H / 2, 4),
            ],
            "x_min": RACK_X_MIN, "x_max": RACK_X_MAX,
            "y_min": RACK_Y_MIN, "y_max": RACK_Y_MAX,
            "z_min": 0.0,        "z_max": RACK_TOTAL_H,
        },
        "second_rack": {
            "center": [
                round((RACK_X_MIN + RACK_X_MAX) / 2, 4),
                round(SECOND_RACK_Y, 4),
                round(RACK_TOTAL_H / 2, 4),
            ],
            "half_extents": [
                round((RACK_X_MAX - RACK_X_MIN) / 2, 4),
                0.45,
                round(RACK_TOTAL_H / 2, 4),
            ],
            "x_min": RACK_X_MIN,       "x_max": RACK_X_MAX,
            "y_min": SECOND_RACK_Y - 0.45, "y_max": SECOND_RACK_Y + 0.45,
            "z_min": 0.0,              "z_max": RACK_TOTAL_H,
        },
        "aisle": {
            "center_y":  AISLE_CENTER_Y,
            "width":     AISLE_WIDTH,
            "x_min":     RACK_X_MIN,
            "x_max":     RACK_X_MAX,
        },
        "obstacle": {
            "center":       [OBS_X, OBS_Y, OBS_HALF_H],
            "half_extents": [OBS_HALF_W, OBS_HALF_D, OBS_HALF_H],
            "x_min":        OBS_X - OBS_HALF_W, "x_max": OBS_X + OBS_HALF_W,
            "y_min":        OBS_Y - OBS_HALF_D, "y_max": OBS_Y + OBS_HALF_D,
            "z_min":        0.0,                "z_max": OBS_HALF_H * 2,
        },
        "bins":  bins,
        "bin_count": len(bins),
    }
    return scene


# ---------------------------------------------------------------------------
# Trajectory endpoint
# ---------------------------------------------------------------------------

@app.get("/api/trajectory")
def get_trajectory():
    """Serve backend/mission_trajectory.json recorded by scripts/record_mission.py."""
    if not MISSION_TRAJ_PATH.exists():
        raise HTTPException(
            status_code=404,
            detail="mission_trajectory.json not found. Run scripts/record_mission.py first.",
        )
    with open(MISSION_TRAJ_PATH) as f:
        return JSONResponse(content=json.load(f))


# ---------------------------------------------------------------------------
# Mission video endpoint
# ---------------------------------------------------------------------------

@app.get("/api/mission_video")
def get_mission_video():
    """Serve sim/assets/mission.mp4 (video/mp4, FileResponse).

    Returns 404 if the video has not been generated yet.
    """
    if not MISSION_VIDEO_PATH.exists():
        raise HTTPException(
            status_code=404,
            detail="mission.mp4 not found. Run scripts/record_mission.py first.",
        )
    return FileResponse(
        str(MISSION_VIDEO_PATH),
        media_type="video/mp4",
        headers={"Cache-Control": "no-store", "Accept-Ranges": "bytes"},
    )


@app.get("/api/mission_frames/{n}")
def get_mission_frame(n: int):
    """Serve frame sequence fallback: sim/assets/mission_frames/<NNNN>.png."""
    frame_path = MISSION_FRAMES_DIR / f"{n:04d}.png"
    if not frame_path.exists():
        raise HTTPException(status_code=404, detail=f"Frame {n} not found.")
    return FileResponse(str(frame_path), media_type="image/png",
                        headers={"Cache-Control": "no-store"})


# ---------------------------------------------------------------------------
# UDP→MJPEG browser bridge
# ---------------------------------------------------------------------------

@app.get("/api/live_stream")
async def live_stream():
    """Stream the Isaac drone-camera UDP feed as multipart MJPEG.

    Reads ``udp://UDP_HOST:UDP_PORT`` (mpegts/H.264 from live_sim.py) and
    transcodes to ``multipart/x-mixed-replace`` JPEG so any ``<img>`` tag can
    display it natively::

        <img src="/api/live_stream">

    If no UDP source is available within ~10 s the endpoint returns 503 with a
    JSON hint.  The ffmpeg subprocess is killed immediately when the client
    disconnects (generator ``finally`` block).
    """
    from backend.udp_bridge import build_mjpeg_cmd

    cmd = build_mjpeg_cmd(host=UDP_HOST, port=UDP_PORT, quality=5, fps=15)
    log.info(f"live_stream: starting ffmpeg bridge: {' '.join(cmd)}")

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )
    except FileNotFoundError:
        raise HTTPException(status_code=503, detail="ffmpeg not found on PATH")

    # Give ffmpeg up to 15 s to produce the first byte.
    # The mpegts/H.264 bridge needs ~2s of analyzeduration + time to reach the
    # first IDR keyframe before ffmpeg emits the first JPEG (typically 10-14s).
    # If ffmpeg exits or produces nothing within that window, return 503.
    deadline = time.monotonic() + 15.0
    first_byte_seen = False
    while time.monotonic() < deadline:
        if proc.poll() is not None:
            stderr_out = proc.stderr.read().decode(errors="replace")
            log.warning(f"live_stream: ffmpeg exited early: {stderr_out[:300]}")
            raise HTTPException(
                status_code=503,
                detail={"error": "live_sim not running", "hint": "start scripts/live_sim.py first"},
            )
        # Non-blocking peek: see if any stdout bytes are ready
        import select
        rlist, _, _ = select.select([proc.stdout], [], [], 0.25)
        if rlist:
            first_byte_seen = True
            break

    if not first_byte_seen:
        proc.kill()
        raise HTTPException(
            status_code=503,
            detail={"error": "live_sim not running", "hint": "start scripts/live_sim.py first"},
        )

    async def _generate():
        try:
            while True:
                chunk = proc.stdout.read(65536)
                if not chunk:
                    break
                yield chunk
        finally:
            try:
                proc.kill()
            except Exception:
                pass

    return StreamingResponse(
        _generate(),
        media_type="multipart/x-mixed-replace; boundary=ffmpeg",
        headers={"Cache-Control": "no-cache"},
    )


@app.get("/api/live_status")
async def live_status():
    """Quick probe: return ``{"streaming": true}`` if UDP stream is active.

    Runs a short ffmpeg grab (max 3 s) in a thread pool so this endpoint stays
    non-blocking.  Returns within ~3 s whether live_sim is running or not.
    """
    from backend.udp_bridge import grab_single_jpeg

    loop = asyncio.get_event_loop()
    # timeout_s=10 because the live mpegts/H.264 probe needs ~2s of analyze
    # time to find an IDR keyframe before it can decode a single JPEG frame.
    jpeg_bytes = await loop.run_in_executor(
        None,
        lambda: grab_single_jpeg(host=UDP_HOST, port=UDP_PORT, timeout_s=10.0),
    )
    streaming = jpeg_bytes is not None
    log.info(f"live_status: streaming={streaming} (probe returned {len(jpeg_bytes) if jpeg_bytes else 0} bytes)")
    return {"streaming": streaming}


# ---------------------------------------------------------------------------
# Static UI files
# ---------------------------------------------------------------------------
if UI_DIR.exists():
    app.mount("/", StaticFiles(directory=str(UI_DIR), html=True), name="ui")
