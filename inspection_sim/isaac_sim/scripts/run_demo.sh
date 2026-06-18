#!/usr/bin/env bash
# One-command launcher for the Automated Cycle Count POC operator demo.
#
# Flow: Operator UI -> backend -> (scan = perception on the drone camera's real
# RTX render of each BIN) -> compare with SAP mock -> completed / discrepancy alert
# -> history. Open the printed URL and click bins (or "Inspect All").
#
# Prereqs (one-time):
#   - Isaac Sim 6.0 binary at ~/isaacsim, assets at ~/isaacsim_assets
#     (scripts/download_assets.sh), conda envs `isaac6` + `perception`.
#   - Scene assets: conda run -n isaac6 python -m sim.bin_map && ... -m sim.gr_label
set -e
cd "$(dirname "$0")/.."
PERCEPTION_PY="$HOME/miniconda3/envs/perception/bin/python"
PORT="${PORT:-8080}"

# 1. Ensure the 18 real BIN camera renders exist (the drone's scan images).
if [ "$(ls sim/assets/captures/*.png 2>/dev/null | wc -l)" -lt 18 ]; then
  echo "[run_demo] Rendering 18 BIN captures in Isaac Sim (one-time, a few minutes)..."
  scripts/run_isaac.sh scripts/render_all_bins.py
fi

# 2. Start the inspection backend + operator UI.
echo "[run_demo] Starting backend on http://localhost:$PORT  (Ctrl+C to stop)"
echo "[run_demo] Open the Operator Console:  http://localhost:$PORT/"
exec "$PERCEPTION_PY" -m uvicorn backend.app:app --host 0.0.0.0 --port "$PORT"
