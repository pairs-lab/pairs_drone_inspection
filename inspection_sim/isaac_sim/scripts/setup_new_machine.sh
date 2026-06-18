#!/usr/bin/env bash
# setup_new_machine.sh — one-time setup of the Drone POC on a fresh machine.
# Assumes: Isaac Sim 6.0 binary already unzipped at $ISAAC_HOME (default ~/isaacsim,
# with post_install.sh already run), Miniconda installed, internet available.
#
# Creates conda envs (isaac6 + perception), installs pinned deps, sets up the
# libzbar symlinks, places the ANAFI drone USD, and downloads the warehouse assets.
set -e
cd "$(dirname "$0")/.."
REPO="$(pwd)"
ISAAC_HOME="${ISAAC_HOME:-$HOME/isaacsim}"
ASSETS="${ISAAC_ASSETS_LOCAL:-$HOME/isaacsim_assets}"

echo "[setup] repo=$REPO  isaac=$ISAAC_HOME  assets=$ASSETS"
[ -x "$ISAAC_HOME/python.sh" ] || { echo "ERROR: Isaac Sim not found at $ISAAC_HOME (install the binary first)"; exit 1; }

# 1. conda env isaac6 (Python 3.12) — pure-Python sim/label tooling + zbar
echo "[setup] creating conda env isaac6 (python 3.12)..."
conda create -y -n isaac6 python=3.12
conda install -y -n isaac6 -c conda-forge zbar
conda run -n isaac6 python -m pip install -r deploy/requirements-isaac6.txt

# 2. conda env perception (Python 3.11) — YOLO/qrdet + PaddleOCR + backend + zbar
echo "[setup] creating conda env perception (python 3.11)..."
conda create -y -n perception python=3.11
conda install -y -n perception -c conda-forge zbar
conda run -n perception python -m pip install -r deploy/requirements-perception.txt

# 3. libzbar symlinks for the Isaac binary python (pyzbar needs libzbar on LD_LIBRARY_PATH)
echo "[setup] creating libzbar symlinks at ~/.local/isaac_extra_libs ..."
mkdir -p "$HOME/.local/isaac_extra_libs"
for f in "$HOME"/miniconda3/envs/isaac6/lib/libzbar.so*; do
  [ -e "$f" ] && ln -sf "$f" "$HOME/.local/isaac_extra_libs/"
done
# Also install the pure-Python deps into the Isaac binary's own python
"$ISAAC_HOME/python.sh" -m pip install qrcode==7.4.2 pyzbar==0.1.9 PyYAML Pillow opencv-python || true

# 4. Place the converted ANAFI drone USD (shipped in the bundle under deploy/)
echo "[setup] placing ANAFI drone USD ..."
mkdir -p "$ASSETS/Custom/ANAFI_Ai"
if [ -f "$REPO/deploy/anafi_ai.usd" ]; then
  cp "$REPO/deploy/anafi_ai.usd" "$ASSETS/Custom/ANAFI_Ai/anafi_ai.usd"
  echo "[setup]   copied deploy/anafi_ai.usd -> $ASSETS/Custom/ANAFI_Ai/"
else
  echo "[setup]   WARNING: deploy/anafi_ai.usd not in bundle. Drone will use the"
  echo "[setup]            primitive-quadrotor fallback, or rebuild via convert_drone_cad.py."
fi

# 5. Download warehouse + props assets (public NVIDIA S3)
echo "[setup] downloading warehouse/props assets (~870 MB) ..."
scripts/download_assets.sh

echo ""
echo "[setup] DONE. Next:"
echo "  conda run -n isaac6 python -m sim.bin_map"
echo "  conda run -n isaac6 python -m sim.gr_label"
echo "  scripts/run_live_demo.sh   # then open http://localhost:8080/"
