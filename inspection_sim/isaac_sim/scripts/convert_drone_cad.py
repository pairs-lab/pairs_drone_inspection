"""Convert ANAFI Ai STEP file to USD via HOOPS (omni.kit.converter.hoops_core subprocess).

Usage (from repo root):
    scripts/run_isaac.sh scripts/convert_drone_cad.py

The ANAFI Ai STEP is ~469 MB; conversion takes 5-30 minutes.
Run in the background and tail the log.

Strategy:
  The correct STEP->USD path in Isaac Sim 6.0 is:
    1. Use omni.kit.converter.hoops_core via a Kit subprocess (--exec), which is
       how omni.services.convert.cad does it internally.
    2. Pass a JSON config with converter options, input path and output path.
  This mirrors the internal subprocess_convert() in
  omni.services.convert.cad.services.subprocess_convert.

Output:
    ~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd
"""
import json
import os
import subprocess
import sys
import tempfile

# ── Paths ─────────────────────────────────────────────────────────────────────
_STEP_ORIG = os.path.expanduser(
    "~/Desktop/Drone_poc/ANAFI_Ai.step_/ANAFI Ai_SIMPLIFIED_20211213.STEP"
)
USD_OUT = os.path.expanduser(
    "~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd"
)
KIT_EXE = os.path.expanduser("~/isaacsim/kit/kit")

# HOOPS launch script inside the hoops_core extension
_HOOPS_MAIN_PATH = None
for d in os.listdir(os.path.expanduser("~/isaacsim/extscache/")):
    if d.startswith("omni.services.convert.cad-"):
        _HOOPS_MAIN_PATH = os.path.expanduser(
            f"~/isaacsim/extscache/{d}/omni/services/convert/cad/services/process/hoops_main.py"
        )
        break

if _HOOPS_MAIN_PATH is None or not os.path.exists(_HOOPS_MAIN_PATH):
    # Fallback: use the hoops extension's own launch script
    for d in os.listdir(os.path.expanduser("~/isaacsim/extscache/")):
        if d.startswith("omni.kit.converter.hoops-"):
            candidate = os.path.expanduser(
                f"~/isaacsim/extscache/{d}/omni/kit/converter/hoops/process/launch_hoops_app.py"
            )
            if os.path.exists(candidate):
                _HOOPS_MAIN_PATH = candidate
                break

os.makedirs(os.path.dirname(USD_OUT), exist_ok=True)

print(f"[convert_drone_cad] Kit   : {KIT_EXE}")
print(f"[convert_drone_cad] Script: {_HOOPS_MAIN_PATH}")
print(f"[convert_drone_cad] Input : {_STEP_ORIG}")
print(f"[convert_drone_cad] Output: {USD_OUT}")
print(f"[convert_drone_cad] Input size: {os.path.getsize(_STEP_ORIG)/1e6:.1f} MB")

if not os.path.exists(KIT_EXE):
    print(f"[convert_drone_cad] ERROR: kit binary not found: {KIT_EXE}")
    sys.exit(1)

if _HOOPS_MAIN_PATH is None or not os.path.exists(_HOOPS_MAIN_PATH):
    print(f"[convert_drone_cad] ERROR: hoops_main.py script not found")
    sys.exit(1)

# ── Config JSON (HOOPS options) ───────────────────────────────────────────────
# See HoopsOptions in omni.kit.converter.hoops_core for all keys.
# use_meter_as_world_unit=True means the STEP mm values are scaled to metres.
config = {
    "use_meter_as_world_unit": True,
    "merge_all_meshes": False,
    "bOptimize": False,
    "convertHidden": False,
}

cfg_fd, cfg_path = tempfile.mkstemp(suffix=".json", prefix="hoops_cfg_")
with os.fdopen(cfg_fd, "w") as f:
    json.dump(config, f)

print(f"[convert_drone_cad] Config: {cfg_path}")

# ── Build the ext-folder args ─────────────────────────────────────────────────
extscache = os.path.expanduser("~/isaacsim/extscache")
exts_dir  = os.path.expanduser("~/isaacsim/exts")
exts_internal = os.path.expanduser("~/isaacsim/extsInternal")

# ── Launch subprocess ─────────────────────────────────────────────────────────
# Quoting: paths with spaces MUST be quoted inside the --exec argument string
# because kit's argparse splits on spaces.  We build the inner exec string
# with explicit quotes around each argument.
def qp(p):
    """Quote a path for use inside the --exec script argument string."""
    return f'"{p}"'

exec_str = (
    f'{_HOOPS_MAIN_PATH} '
    f'--input-path {qp(_STEP_ORIG)} '
    f'--output-path {qp(USD_OUT)} '
    f'--config-path {qp(cfg_path)}'
)

cmd = [
    KIT_EXE,
    "--ext-folder", extscache,
    "--ext-folder", exts_dir,
    "--ext-folder", exts_internal,
    "--enable", "omni.kit.converter.hoops_core",
    "--allow-root",
    "--/app/fastShutdown=1",
    "--exec", exec_str,
    "--info",
    "--/persistent/app/usd/muteUsdDiagnostics=false",
]

print(f"[convert_drone_cad] Launching: {' '.join(cmd[:8])} ... --exec '{exec_str}'")
print("[convert_drone_cad] Conversion started (this may take many minutes) ...")

result = subprocess.run(cmd, capture_output=False, text=True)
rc = result.returncode
print(f"[convert_drone_cad] Kit subprocess exited with code {rc}")

# ── Check output ──────────────────────────────────────────────────────────────
if os.path.exists(USD_OUT):
    sz = os.path.getsize(USD_OUT)
    print(f"[convert_drone_cad] Output file: {sz/1e6:.2f} MB  path={USD_OUT}")
    if sz > 200_000:
        print(f"[convert_drone_cad] Conversion SUCCESS  output_size={sz/1e6:.1f} MB")
    else:
        print(f"[convert_drone_cad] WARNING: output file is suspiciously small ({sz} bytes)")
else:
    print(f"[convert_drone_cad] Conversion FAILED: output file not created")

# ── Quick bbox sanity check ───────────────────────────────────────────────────
if os.path.exists(USD_OUT) and os.path.getsize(USD_OUT) > 200_000:
    try:
        # Use isaac binary python for this check (already running inside it if called via run_isaac.sh)
        from pxr import Usd, UsdGeom
        stage = Usd.Stage.Open(USD_OUT)
        cache = UsdGeom.BBoxCache(
            Usd.TimeCode.Default(),
            [UsdGeom.Tokens.default_, UsdGeom.Tokens.render],
            useExtentsHint=True,
        )
        root = stage.GetPseudoRoot()
        rng = cache.ComputeWorldBound(root).ComputeAlignedRange()
        mn, mx = rng.GetMin(), rng.GetMax()
        sz_dim = (mx[0]-mn[0], mx[1]-mn[1], mx[2]-mn[2])
        print(f"[convert_drone_cad] Native bbox (m or mm): {sz_dim[0]:.4f} x {sz_dim[1]:.4f} x {sz_dim[2]:.4f}")
        print(f"[convert_drone_cad] CONVERSION_OK bbox={sz_dim}")
    except Exception as e:
        print(f"[convert_drone_cad] bbox check error (non-fatal): {e}")

# Clean up temp config
try:
    os.unlink(cfg_path)
except Exception:
    pass

print("[convert_drone_cad] Done.")
