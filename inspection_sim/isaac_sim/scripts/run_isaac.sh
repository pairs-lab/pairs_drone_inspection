#!/usr/bin/env bash
# Launch a Python script with the Isaac Sim 6.0 BINARY interpreter (python.sh).
#
# Why a wrapper: the binary bundles its own Python 3.12. We add:
#   - OMNI_KIT_ACCEPT_EULA=YES  (skip first-run EULA prompt)
#   - libzbar on LD_LIBRARY_PATH (for pyzbar QR decode; lib provided by conda
#     env isaac6, symlinked into ~/.local/isaac_extra_libs to avoid pulling the
#     whole conda lib dir which shadows system libs).
#
# Usage (run from repo root so `import sim.*` resolves):
#   scripts/run_isaac.sh scripts/verify_scene.py
#   scripts/run_isaac.sh -m sim.scene_builder
ISAAC_HOME="${ISAAC_HOME:-$HOME/isaacsim}"
EXTRA_LIBS="${ISAAC_EXTRA_LIBS:-$HOME/.local/isaac_extra_libs}"

export OMNI_KIT_ACCEPT_EULA=YES
export LD_LIBRARY_PATH="$EXTRA_LIBS:$LD_LIBRARY_PATH"
export PYTHONPATH="$(pwd):$PYTHONPATH"   # so `import sim.*` works from repo root

exec "$ISAAC_HOME/python.sh" "$@"
