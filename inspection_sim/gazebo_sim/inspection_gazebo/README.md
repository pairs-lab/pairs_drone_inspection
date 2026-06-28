# inspection_gazebo

Gazebo Classic 11 (ROS Noetic) simulation of the warehouse cycle-count inspection
environment. Self-contained so it can be handed over and built without the rest of
the inspection stack.

## What's in the world (`worlds/warehouse.world`)

- **Six pallet-racking units** forming **3 aisles** (6 levels, 3 bays, ~7 m tall) — blue uprights / orange beams / grey pallets. Each level contains pallets filling all 18 bins per rack.
- **Unique Anchor AprilTags** (`apriltag_marker`) placed at the start of each rack (IDs 101 to 106) for robust rack identification and relative navigation.
- **Bin AprilTags** (ID 94) centered inside each bin space to facilitate high-precision visual servoing.
- **A fiducial charging dock** at the aisle entrance (`x = -6`) for precision-landing tests.

Geometry is parameterized in `models/warehouse_rack/generate_rack.py` (re-run to match a
site survey: bays, levels, pitch, aisle depth).

## Navigation Features (`scripts/relative_navigator.py`)

This package includes a fully functional, autonomous GUI ground control station that demonstrates a highly robust **Hybrid Navigation System**:

1. **Phase 1: Global Approach (Tìm Kệ)**
   - The drone uses global coordinates to fly down a safe center-corridor (`X = -5.5`) to the entrance of the selected rack.
   - It searches for and locks onto the specific Anchor Tag (e.g. ID 103 for Rack 3).

2. **Phase 2: Manhattan Zigzag Navigation (Bay Zigzag)**
   - Using the Anchor Tag as a relative datum, the drone computes the exact positions of all 18 bins.
   - It employs a true **Lawnmower (Zigzag) Path Planning algorithm** to move safely along the Manhattan Grid (X and Z axes) without colliding into racks.
   - Supports both flying to a specific bin dynamically, or performing a full autonomous 18-bin scan sequence.

3. **Phase 3: Visual Servoing (Căn giữa tự động)**
   - Upon arriving at the calculated bin coordinates, the drone locks onto the nearest Bin Tag (ID 94).
   - It applies a Proportional (P) Controller using the camera's X and Y pixel errors to dynamically shift the drone's position until the bin tag is perfectly centered in the frame.

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

Once the simulation starts and the drone is hovering, launch the autonomous GCS:

```bash
rosrun inspection_gazebo relative_navigator.py
```

**Gazebo only (no autonomy core)** — quick world/sensor check:

```bash
roslaunch inspection_gazebo full_sim.launch                        # gazebo + world + drone
# or the manual two-step:
roslaunch inspection_gazebo simulation.launch                      # gazebo + world
rosrun inspection_gazebo spawn_inspection_uav.sh 1 x500            # spawn drone (+ToF/Livox/cam)
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

## Layout

```
inspection_gazebo/
├── launch/   full_sim.launch (top), simulation.launch (world only)
├── worlds/   warehouse.world
├── models/   warehouse_rack (generated), apriltag_marker (textures)
├── scripts/  spawn_inspection_uav.sh, relative_navigator.py
├── config/   tags.yaml
└── tmux/     warehouse session definitions
```
