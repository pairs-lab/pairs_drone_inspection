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

## Precision landing on the charging dock

The session wires the PAIRS precise-landing stack onto the charging dock at the aisle
entrance (world `-6, 0`):

- **The pad** — `tag_dock` is the recursive `Apriltag_recursive1` fiducial from
  `pairs_precise_landing_gazebo` (a `tagCustom48h12` pad: a big id 0 @ 0.30 m for the far
  approach with a small id 10 @ 0.06 m nested in its centre for the final centimetres),
  flat on the dock facing **+Z up** so the descending drone reads it.
- **The eye** — the drone is spawned with `--enable-bluefox-camera`, a downward camera
  publishing `/uav1/bluefox_optflow/image_raw` (+`camera_info`) — the fiducial stream.
- **The chain** — the `precland` tmux window runs one launch that transitively starts the
  AprilTag detector → landing-pad LKF (`landing_pad_estimation`) → descent controller
  (`precise_landing`):
  ```bash
  roslaunch pairs_precise_landing precise_landing.launch \
      apriltag_config:=./config/apriltag.yaml \
      camera_node:=bluefox_optflow image_topic:=image_raw \
      estimator_config:=./config/landing_estimator.yaml \
      controller_config:=./config/landing_controller.yaml
  ```
- **The trigger ("button")** — the `dock` tmux window pre-stages three commands in shell
  history (press ↑ to recall, newest first):
  1. `rosservice call /uav1/control_manager/goto "goal: [-6.0, 0.0, 2.0, 0.0]"` — fly over the dock
  2. `rosservice call /uav1/precise_landing/land`  — start the staged precise descent (`std_srvs/Trigger`)
  3. `rosservice call /uav1/precise_landing/abort` — climb back off the pad and return to IDLE

  The land service is only accepted while **flying normally with the pad in view** (state
  machine IDLE → ALIGN → DESCEND → ALIGN2 → LANDING), so take off and reach the dock first.

> Needs the precise-landing packages (shipped in the PAIRS base image) and a **GPU/display**
> for the downward camera to render — without rendered frames the detector sees no tag. The
> world-load and dock-pad spawn are verified headless; the full descent is a GPU-sim test.

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
