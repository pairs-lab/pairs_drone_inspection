# M1 — Sim Environment (Isaac Sim 6.0)

Procedurally builds the warehouse POC scene: a 3×6 rack (18 BINs) with pallets and
emissive GR-label quads (QR + text), plus a drone (body + camera).

## Prerequisites

- Isaac Sim 6.0 **binary** installed at `~/isaacsim` (see `docs/superpowers/INSTALL_NOTES.md`).
  The pip `isaacsim==6.0.0` bundle is broken (missing extensions) — use the binary.
- Pure-Python deps for label/QR tooling are in conda env `isaac6` AND in the binary
  python (`~/isaacsim/python.sh -m pip install qrcode pyzbar PyYAML Pillow pytest`).
- Run Isaac scripts via `scripts/run_isaac.sh` (sets EULA + libzbar + PYTHONPATH).

## Generate scene assets

```bash
conda run -n isaac6 python -m sim.bin_map     # -> sim/bin_map.yaml (18 BINs A1..C6)
conda run -n isaac6 python -m sim.gr_label    # -> sim/assets/labels/<bin>.png (QR+text)
```

## Build the scene

```bash
scripts/run_isaac.sh -m sim.scene_builder            # headless -> sim/warehouse_poc.usd
scripts/run_isaac.sh -m sim.scene_builder -- --gui   # open Isaac Sim window
```

## Tests & verification

```bash
conda run -n isaac6 python -m pytest tests/ -v       # pure-Python: bin_map, gr_label
scripts/run_isaac.sh scripts/verify_isaac.py         # ISAAC_OK (headless launch)
scripts/run_isaac.sh scripts/verify_scene.py         # SCENE_OK bins=18
scripts/run_isaac.sh scripts/verify_capture.py       # CAPTURE_QR_OK part_no=PN-A01 (real render)
```

`sim/assets/capture_A1.png` is the genuine RTX render the drone camera sees — the QR
decodes from real pixels (no compositing). This image is the input the M3 perception
module (YOLO + PaddleOCR) will consume.

## Files

| File | Responsibility |
|------|----------------|
| `config.py` | Layout constants (3 cols, 6 levels, spacing, scan standoff, home pose) |
| `bin_map.py` | Generate/validate/load `bin_map.yaml` (18 BINs → poses + ground-truth) |
| `gr_label.py` | Generate GR-label PNGs (QR encodes part_no+qty) + QR decode helpers |
| `warehouse.py` | Ground + lighting |
| `rack.py` | 3×6 rack: pallet cube + emissive textured label quad per BIN |
| `drone_asset.py` | Drone body + camera prim at a given pose |
| `scene_builder.py` | Entrypoint: assemble full scene, save USD |
