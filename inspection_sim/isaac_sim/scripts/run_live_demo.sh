#!/usr/bin/env bash
# run_live_demo.sh — Start the full live operator dashboard demo.
#
# Starts two processes:
#   1. Isaac Sim live_sim.py  (WebSocket server on ws://localhost:8765)
#   2. Inspection backend     (FastAPI + Operator UI on http://localhost:8080)
#
# Isaac Sim takes ~60-90 s to boot and compile shaders on first run.
# The UI will show "waiting for Isaac live_sim…" until WS 8765 is ready.
#
# Usage:
#   scripts/run_live_demo.sh             # starts both; Ctrl-C stops the backend
#   PORT=9090 scripts/run_live_demo.sh   # custom port

set -e
cd "$(dirname "$0")/.."

PERCEPTION_PY="$HOME/miniconda3/envs/perception/bin/python"
PORT="${PORT:-8080}"
WS_PORT="${WS_PORT:-8765}"

# --- Free ports / kill stale instances so we never hit "address already in use" ---
free_port() {
  local p="$1"
  local pids
  pids=$(lsof -ti ":$p" 2>/dev/null || true)
  if [ -n "$pids" ]; then
    echo "[live_demo] Port $p busy (PIDs: $pids) — killing stale process(es)..."
    kill $pids 2>/dev/null || true; sleep 2; kill -9 $pids 2>/dev/null || true
  fi
}
echo "[live_demo] Cleaning up any previous demo processes..."
pkill -f "scripts/live_sim.py" 2>/dev/null || true
sleep 1
free_port "$WS_PORT"     # live_sim WebSocket
free_port "$PORT"        # backend / UI

echo "============================================================"
echo "  Automated Cycle Count — Live Demo"
echo "============================================================"
echo ""
echo "[live_demo] Step 1: Starting Isaac Sim live_sim (background)"
echo "[live_demo]   → WebSocket server: ws://localhost:8765"
echo "[live_demo]   → Isaac takes ~60-90 s to boot + compile shaders"
echo "[live_demo]   → Log: /tmp/live_sim.log"
echo ""

# Start Isaac live_sim in background (via run_isaac.sh).
# Default: GUI window (so you SEE the Isaac Sim viewport at the parked-drone view).
# Set HEADLESS=1 to run without a window (server only).
LIVE_ARGS=""
if [ "${HEADLESS:-0}" = "1" ]; then
  LIVE_ARGS="-- --headless"
  echo "[live_demo]   → HEADLESS mode (no Isaac window)"
else
  echo "[live_demo]   → GUI mode: Isaac Sim window opens at the parked-drone view"
fi
scripts/run_isaac.sh scripts/live_sim.py $LIVE_ARGS > /tmp/live_sim.log 2>&1 &
ISAAC_PID=$!
echo "[live_demo] Isaac PID: $ISAAC_PID"

# Trap so Isaac is killed if backend exits
cleanup() {
  echo ""
  echo "[live_demo] Stopping Isaac (PID $ISAAC_PID) + live_sim children..."
  kill "$ISAAC_PID" 2>/dev/null || true
  # run_isaac.sh execs python.sh -> kit; kill the whole live_sim tree by name
  pkill -f "scripts/live_sim.py" 2>/dev/null || true
  free_port "$WS_PORT"
  echo "[live_demo] Done."
}
trap cleanup EXIT INT TERM

echo ""
echo "[live_demo] Step 2: Starting Inspection Backend (perception env)"
echo "[live_demo]   → Operator Console: http://localhost:$PORT/"
echo "[live_demo]   → API:              http://localhost:$PORT/api/scene"
echo ""
echo "[live_demo] *** Open http://localhost:$PORT/ in your browser ***"
echo "[live_demo] The camera panel will show 'waiting for Isaac live_sim...'"
echo "[live_demo] Once Isaac boots (~60-90s) the live feed + drone tracking will appear."
echo ""

exec "$PERCEPTION_PY" -m uvicorn backend.app:app \
  --host 0.0.0.0 \
  --port "$PORT" \
  --log-level info
