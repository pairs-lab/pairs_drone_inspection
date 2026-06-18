# inspection_gazebo

Gazebo Classic 11 (ROS Noetic) simulation of the warehouse cycle-count inspection
environment. Self-contained so it can be handed over and built without the rest of
the inspection stack.

## What's in the world (`worlds/warehouse.world`)

- **Two pallet-racking units** forming a **1.6 m aisle** along `+x` (6 levels, 3 bays,
  ~7 m tall) — blue uprights / orange beams / grey pallets, matching the partner site.
- **AprilTag anchors** (`apriltag_marker`) on the aisle-facing beams — 3 per side — for
  global-localization corrections and to defeat perceptual aliasing.
- **A fiducial charging dock** at the aisle entrance (`x = -6`) for precision-landing tests.

Geometry is parameterized in `models/warehouse_rack/generate_rack.py` (re-run to match a
site survey: bays, levels, pitch, aisle depth).

## The downward ToF

The inspection drone is spawned with `--enable-rangefinder`, which adds the existing
Garmin down-rangefinder from `x500.sdf.jinja`. It publishes `sensor_msgs/Range` on
`/<uav>/hw_api/distance_sensor` — exactly the topic the PAIRS EstimationManager's
garmin/AGL correction consumes. No new sensor code is needed.

## Run it

**Full bring-up (recommended) — tmux session** (gazebo + world + drone + autonomy core + takeoff),
mirroring the PAIRS `one_drone_3dlidar` session:

```bash
# build + source the workspace first (Noetic / catkin)
roscd inspection_gazebo/tmux/warehouse && ./start.sh     # ./kill.sh to stop
```

The `goto` window has a preloaded command to fly down the aisle (`[3.0, 0.0, 1.5, 0.0]`).
The core flies on sim-GPS for now (`config/custom_config.yaml`); switching to Point-LIO /
LIO-SAM is Step 2.

**Gazebo only (no autonomy core)** — quick world/sensor check:

```bash
roslaunch inspection_gazebo full_sim.launch                        # gazebo + world + drone
# or the manual two-step:
roslaunch inspection_gazebo simulation.launch                      # gazebo + world
rosrun   inspection_gazebo spawn_inspection_uav.sh 1 x500           # spawn drone (+ToF/Livox/cam)
```

Sensor flags are configurable via the `sensors:=` launch arg (passed through to the PAIRS
drone spawner).

## Layout (mirrors kr_autonomous_flight/autonomy_sim/gazebo_sim)

```
inspection_gazebo/
├── launch/   full_sim.launch (top), simulation.launch (world only)
├── worlds/   warehouse.world
├── models/   warehouse_rack (generated), apriltag_marker (reused texture)
├── scripts/  spawn_inspection_uav.sh
└── config/
```

## Follow-ups (not in this first pass)

- **Unique tag IDs:** every `apriltag_marker` currently shares one tag36h11 texture
  (id 0). Generate per-anchor textures (tag36h11) so anchors are distinguishable.
- **Semantic BIN map:** the rack here is geometry only; the (col,level)→standoff-pose
  layer is a later inspection_core deliverable.
- **Wire AprilTag → EstimationManager** correction once tags are unique.
