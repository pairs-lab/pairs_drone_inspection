# PAIRS Autonomous Drone Inspection Simulation (MuJoCo)

## Overview
This project is a Proof of Concept (POC) simulation for an autonomous drone-based inventory inspection system using the **MuJoCo** physics engine. It simulates a drone navigating a warehouse environment, scanning QR codes (representing GR labels) on storage bins, and verifying inventory against expected data.


## Features
* **Realistic Physics Simulation:** Built on MuJoCo, simulating drone dynamics and environmental interactions.
* **Dynamic Environment Generation:** Automatically constructs a 3x6 rack warehouse grid with shelves, boxes, and structural frames.
* **Procedural QR Code Generation:** Dynamically generates texture maps (QR codes) for each storage bin based on inventory data, including simulated errors (mismatches).
* **Automated Navigation & Scanning:** Simulates the drone's flight path (go-to-BIN navigation flow) to specific storage locations.
* **Interactive Operator Console:** A Tkinter-based UI dashboard allowing operators to select target bins and view real-time FPV camera feeds.
* **Real-time Discrepancy Alerting:** The system automatically cross-checks scanned data and visually highlights (blinks red) bins with inventory mismatches.

## Project Structure
The project follows a modular architecture for maintainability:

```text
inspection_mujoco_demo/
├── models/                     # 3D Assets (.obj) and generated textures
├── src/                        # Source Code Directory
│   ├── __init__.py             # Python package marker
│   ├── config.py               # Shared state, global variables, and UI styling
│   ├── environment.py          # Generates QR textures and the MuJoCo XML scene
│   ├── simulation.py           # MuJoCo physics engine thread and control logic
│   └── ui.py                   # Tkinter Operator Console dashboard
├── download_assets.sh          # Script to fetch necessary 3D meshes (if applicable)
└── run_demo.py                 # Main entry point to launch the application