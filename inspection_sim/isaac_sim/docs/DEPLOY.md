# Deploy on another machine — inventory & steps

## What the demo needs (inventory)

| # | Component | Size | In the transfer bundle? | How to obtain on the new machine |
|---|-----------|------|------------------------|----------------------------------|
| 1 | **Project repo** (sim/ drone/ perception/ backend/ ui/ scripts/ docs/, incl. `sim/bin_map.yaml`) | ~1 MB | ✅ **copied** | extracted from the bundle (or `git clone`) |
| 2 | **Converted ANAFI drone USD** `anafi_ai.usd` | 35 MB | ✅ **copied** | bundle → place at `~/isaacsim_assets/Custom/ANAFI_Ai/anafi_ai.usd` (NOT re-downloadable; only rebuildable from the STEP CAD) |
| 3 | **Env spec / pinned deps** (`deploy/requirements-*.txt`, `environment.*.yml`) | tiny | ✅ **copied** | used by `scripts/setup_new_machine.sh` |
| 4 | **Isaac Sim 6.0 BINARY** at `~/isaacsim` | 13 GB zip → ~27 GB | ❌ download | `downloads.isaacsim.nvidia.com` → unzip to `~/isaacsim` → `./post_install.sh`. (Do NOT use pip `isaacsim==6.0.0` — it is missing extensions; see `docs/superpowers/INSTALL_NOTES.md`.) |
| 5 | **Warehouse/props assets** at `~/isaacsim_assets` | ~870 MB | ❌ download | `scripts/download_assets.sh` (public NVIDIA S3, needs internet) |
| 6 | **conda envs** `isaac6` (py3.12) + `perception` (py3.11) | ~15 GB | ❌ rebuild | `scripts/setup_new_machine.sh` (pip installs pinned deps; needs internet) |
| 7 | **libzbar** symlinks `~/.local/isaac_extra_libs` | tiny | ❌ recreate | `scripts/setup_new_machine.sh` (symlinks libzbar from the `isaac6` conda env) |
| 8 | **qrdet / ultralytics / PaddleOCR model weights** | ~50–100 MB | ❌ auto | downloaded automatically on first perception run (needs internet once) |

**Transfer bundle (what you copy to the new machine)** ≈ **36 MB**: the repo + the ANAFI USD + deploy specs. Everything else is downloaded/rebuilt on the target.

## Hardware / OS prerequisites (target machine)

- Linux x86_64, **NVIDIA RTX GPU** (RTX 30/40/50-series; demo built on RTX 5060 Ti / Blackwell), recent driver (≥ 550).
- ~50 GB free disk (Isaac binary 27 GB + assets ~1 GB + conda envs ~15 GB).
- Miniconda/Anaconda installed. `ffmpeg`, `git`, `curl`, `lsof` available (`sudo apt install ffmpeg`).
- A display (`$DISPLAY`) if you want the Isaac GUI window; headless works for the web demo.

## Step-by-step

```bash
# 0. Put the bundle on the new machine and extract it (gives ~/Desktop/Drone_poc)
tar xzf Drone_poc_deploy.tar.gz -C ~/Desktop

# 1. Install Isaac Sim 6.0 BINARY (one-time, ~13 GB download)
#    Download isaac-sim-standalone-6.0.0-linux-x86_64.zip from
#    https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/download.html
mkdir -p ~/isaacsim && unzip isaac-sim-standalone-6.0.0-linux-x86_64.zip -d ~/isaacsim
cd ~/isaacsim && ./post_install.sh && cd -

# 2. Create conda envs + deps + zbar symlinks + place ANAFI USD + download assets
cd ~/Desktop/Drone_poc
scripts/setup_new_machine.sh           # ISAAC_HOME=~/isaacsim by default

# 3. Generate scene assets (BIN map + GR label textures)
conda run -n isaac6 python -m sim.bin_map
conda run -n isaac6 python -m sim.gr_label

# 4. Run the live demo
scripts/run_live_demo.sh
#    wait ~60-90 s for Isaac to boot, then open http://localhost:8080/
```

## Notes

- `scripts/run_isaac.sh` assumes Isaac at `~/isaacsim`. Override with `ISAAC_HOME=/path scripts/run_isaac.sh ...`.
- If the ANAFI USD is missing, `spawn_drone()` falls back to a primitive quadrotor — the demo still runs.
- To rebuild the ANAFI USD from CAD instead of copying it: put the ANAFI STEP under the repo and run `scripts/run_isaac.sh scripts/convert_drone_cad.py`.
- All the hard-won environment gotchas (broken pip bundle, RTX texture/near-clip, batched RigidPrim API, RTX LiDAR annotator name, PaddleOCR/paddle versions) are in `docs/superpowers/INSTALL_NOTES.md`.
