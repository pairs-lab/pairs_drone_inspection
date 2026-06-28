# inspection_gazebo

Gazebo Classic 11 (ROS Noetic) simulation of the warehouse cycle-count inspection
environment. Self-contained so it can be handed over and built without the rest of
the inspection stack.

## What's in the world (`worlds/warehouse.world`)

- **Six pallet-racking units** forming **3 aisles** (6 levels, 3 bays, ~7 m tall) — blue uprights / orange beams / grey pallets. Each level contains pallets filling all 18 bins per rack.
- **Unique Anchor AprilTags** (`apriltag_marker`) placed at the start of each rack (IDs 101 to 106) for robust rack identification and relative navigation.
- **Bin AprilTags** (ID 94) centered inside each bin space to facilitate high-precision visual servoing.
- **A fiducial charging dock** at the aisle entrance (`x = -6`) carrying the recursive
  (`tagCustom48h12`) `Apriltag_recursive1` landing pad for precision-landing tests.

Geometry is parameterized in `models/warehouse_rack/generate_rack.py` (re-run to match a
site survey: bays, levels, pitch, aisle depth).

## Operator control panel (inspection_core rqt plugin)

The operator GUI lives in the **`inspection_core`** package as an rqt plugin — label
**"PAIRS Inspection Control"** (`inspection_core.inspection_panel.InspectionPanel`), under
the **PAIRS** rqt plugin group. Launch it standalone, or pull it into any rqt session:

```bash
roslaunch inspection_core inspection.launch          # standalone rqt window
# or:  rosrun inspection_core inspection_gui
# or inside any rqt:  Plugins -> PAIRS -> PAIRS Inspection Control
```

From the panel you can: **auto-find a rack** via its Anchor Tag (IDs 101 to 106), **zig-zag to
a bin** or run a full **18-bin zig-zag rack sweep** (visual-servo centering on the bin tag),
**precise-land on the charging dock** (Go to dock / LAND / ABORT), and watch a **live camera
feed**.

The panel relies on the rack/bin AprilTag detector started by `apriltag.launch` (runs an
`apriltag_ros` continuous node on the front RealSense `front_rgbd/infra1` stream, publishing
`/<uav>/tag_detections`):

```bash
roslaunch inspection_gazebo apriltag.launch uav_name:=uav1
```

## Sensors

The inspection drone is spawned with four sensor flags:

- `--enable-rangefinder` — downward Garmin ToF from `x500.sdf.jinja`, publishing
  `sensor_msgs/Range` on `/<uav>/hw_api/distance_sensor` (the AGL topic the PAIRS
  EstimationManager's garmin correction consumes). No new sensor code is needed.
- `--enable-livox` — Livox Mid-360 (the primary LiDAR-inertial sensor; localization is
  sim-GPS today, with Point-LIO LiDAR-inertial as the planned next step).
- `--enable-realsense-front` — front RGB-D; `apriltag.launch` runs the rack/bin detector on
  its `front_rgbd/infra1` stream into `/<uav>/tag_detections`.
- `--enable-bluefox-camera` — downward mono camera that renders the precise-landing fiducial
  on the charging dock.

## Run it

**Full bring-up (recommended) — tmux session** (gazebo + world + drone + autonomy core +
precise landing + takeoff + operator panel), mirroring the PAIRS `one_drone_3dlidar` session:

```bash
# build + source the workspace first (Noetic / catkin)
roscd inspection_gazebo/tmux/warehouse && ./start.sh     # ./kill.sh to stop
```

The session opens, in order, these windows:
`roscore`, `gazebo`, `status`, `hw_api`, `core`, `avoid`, `precland`, `takeoff`, `goto`,
`inspect`, `viz`, `kill`. The notable ones:

- **`avoid`** — obstacle avoidance off the Livox Mid-360 (`inspection_core avoidance.launch`):
  `octomap_server` builds a live 3D map, `octomap_planner` serves collision-free `goto`, and
  `pairs_bumper` feeds the ControlManager's reactive bumper. Without it the drone flies
  straight-line `goto` and hits the racks.
- **`precland`** — the `pairs_precise_landing` chain (AprilTag detector → landing-pad LKF →
  descent controller) reading the downward bluefox cam.
- **`goto`** — pre-loaded history (press Up) for the collision-free `octomap_planner/goto`
  (use this) and the raw straight-line `control_manager/goto`.
- **`inspect`** — the rack/bin AprilTag detector (`apriltag.launch`) plus the `inspection_core`
  rqt operator panel. The panel now also carries the **flight controls** (arm / takeoff /
  land / hover / e-land / goto) that used to live in the separate `pairs_rqt_control` window.
- **`viz`** — merged window: RViz (the `inspection_core` warehouse config — octomap map +
  Mid-360 cloud + planned path) + robot model + rviz interface + i3 layout. The former
  separate rviz / gui / layout windows are consolidated here; the standalone flight-control
  GUI is gone (merged into the rqt panel), as is the old `dock` window (its go-to-dock /
  land / abort are buttons in the panel).

Once the drone is hovering, drive the inspection from the `inspect` window's rqt panel.

**Gazebo only (no autonomy core)** — quick world/sensor check:

```bash
roslaunch inspection_gazebo full_sim.launch                        # gazebo + world + drone
# or the manual two-step:
roslaunch inspection_gazebo simulation.launch                      # gazebo + world
rosrun inspection_gazebo spawn_inspection_uav.sh 1 x500            # spawn drone (+ToF/Livox/cams)
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
- **The trigger** — the `inspection_core` rqt panel's **Go to dock / LAND / ABORT** buttons
  drive the same three services (the old standalone `dock` tmux window is gone):
  1. `/uav1/control_manager/goto "goal: [-6.0, 0.0, 2.0, 0.0]"` — fly over the dock
  2. `/uav1/precise_landing/land`  — start the staged precise descent (`std_srvs/Trigger`)
  3. `/uav1/precise_landing/abort` — climb back off the pad and return to IDLE

  The land service is only accepted while **flying normally with the pad in view** (state
  machine IDLE → ALIGN → DESCEND → ALIGN2 → LANDING), so take off and reach the dock first.

> Needs the precise-landing packages (shipped in the PAIRS base image) and a **GPU/display**
> for the downward camera to render — without rendered frames the detector sees no tag. The
> world-load and dock-pad spawn are verified headless; the full descent is a GPU-sim test.

## Layout

```
inspection_gazebo/
├── launch/   full_sim.launch (top), simulation.launch (world only), apriltag.launch (rack/bin detector)
├── worlds/   warehouse.world
├── models/   warehouse_rack (generated), apriltag_marker (textures)
├── scripts/  spawn_inspection_uav.sh
├── config/   tags.yaml, settings.yaml
└── tmux/     warehouse session definitions

The operator GUI now lives in the `inspection_core` package (rqt plugin), not here.
```
