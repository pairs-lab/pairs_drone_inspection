# inspection_core

The **inspection-specific autonomy layer** for the warehouse cycle-count drone. It sits
*on top of* the PAIRS UAV system and only adds what is unique to the inspection mission —
it does **not** re-implement control, estimation, or planning (those come from
`pairs_uav_managers`, `pairs_uav_trackers`, `pairs_uav_state_estimators`, etc.).

Organisation mirrors `kr_autonomous_flight/autonomy_core`; the technique is PAIRS
(ROS Noetic / catkin, manager-plugin interfaces, bloom→deb→apt→docker packaging).

## Scope

| Concern | Where it lives | Status |
| --- | --- | --- |
| **Semantic BIN map** — (column,level) → standoff drone pose + nearest tag + SAP slot | `config/bin_map/` | example map (this step) |
| **Go-to-BIN mission logic** — operator picks a BIN → look up standoff pose → call `ControlManager` `goto` → hold for capture | `src/` (mission node) | next step |
| **Perception glue** — label/QR capture + decode hooks | `src/` | later |

The PAIRS stack supplies the rest: `EstimationManager` (LiDAR-inertial + fiducial
corrections), `ControlManager` (MPC tracker + SE(3)/MPC controllers + bumper + failsafe),
the OctoMap planner, and the HW API to PX4.

## Obstacle avoidance — off the Livox Mid-360

`config/avoidance/` + `launch/avoidance.launch` add **collision-free flight** in the racked
aisles, off the Mid-360 already on the airframe (`--enable-livox`). Two layers:

- **Deliberative** — `pairs_octomap_server` builds a live 3D occupancy map; `pairs_octomap_planner`
  plans paths through it. Call `/<uav>/octomap_planner/goto` (`pairs_msgs/Vec4 [x,y,z,heading]`)
  to fly *around* the racks. The rqt panel's navigation and the warehouse `goto` window use it.
- **Reactive** — `pairs_bumper` turns the Mid-360 cloud into obstacle sectors; the
  ControlManager's `obstacle_bumper` (enabled in the public defaults, clearance tuned down for
  narrow aisles in the warehouse `custom_config.yaml`) repels from them as the last line of defence.

```bash
roslaunch inspection_core avoidance.launch        # after the core + Mid-360 are up
```

The warehouse tmux launches this automatically (the `avoid` window). All avoidance packages
(`pairs_octomap_*`, `pairs_bumper`, `pairs_subt_planning_lib`) ship in the base PAIRS image.

> **⚠ Needs a GPU sim flight to tune.** The narrow-aisle clearances in `config/avoidance/*.yaml`
> (planner `safe_obstacle_distance`, bumper min distance) are first cuts — verify the drone can
> still traverse a ~2.6 m aisle (the planner doesn't refuse to move, the bumper doesn't oscillate).

## Localization (GPS-denied) — opt-in Point-LIO

`config/localization/` + `launch/localization.launch` add **LiDAR-inertial localization** for the
warehouse (no GPS), using the PAIRS **Point-LIO** estimator on the Livox Mid-360 (odometry-only —
no persisted map). The Garmin **down-ToF** still provides AGL for landing.

It is **opt-in** — the warehouse sim still defaults to sim-GPS, because Point-LIO needs a **GPU sim
flight to validate** (frames/offsets/tuning) that can't be done headlessly. To switch:

```bash
# 1) run the core with the localization config (instead of the GPS one):
roslaunch pairs_uav_core core.launch ... \
    custom_config:=$(rospack find inspection_core)/config/localization/custom_config.yaml ...
# 2) bring up Point-LIO + the topic/frame reconciliation:
roslaunch inspection_core localization.launch
```

`localization.launch` relays the stock `--enable-livox` topics
(`mid360_cloud_nodelet/{points,imu}` → `livox/{points,imu}`) and publishes the `fcu→livox` static
TF that Point-LIO expects. The Point-LIO packages ship in the Docker image; for a bare apt install
also `apt install ros-noetic-point-lio ros-noetic-pairs-point-lio-core
ros-noetic-pairs-point-lio-estimator-plugin`.

> **Follow-ups (scaffolded, not wired):** D455 **OpenVINS** as a VIO *backup* estimator — the plugin
> is already in the base image, but the sim camera (a D435) has **no IMU**, so VIO needs `hw_api/imu`
> + sim calibration. A true **AprilTag drift-anchor** estimator correction needs the estimator's
> covariance-pose type (currently a TODO stub); today tags only drive the *landing* controller.

## Operator GUI — the rqt panel

`src/inspection_core/inspection_panel.py` is the **single rqt control panel** (plugin
*PAIRS Inspection Control*, under the **PAIRS** plugin group) — it replaces *both* the old
standalone Tkinter `relative_navigator` *and* the separate `pairs_rqt_control` flight window.
It gives the operator:

- **Flight control** (merged from `pairs_rqt_control`) — arm / disarm / offboard / one-click
  takeoff / land / land-home / hover / e-land, a live armed/offboard/tracker status line, and a
  free `goto` with *Go To (avoid)* (collision-free via the octomap planner) and *Go To (direct)*.
- **Relative navigation** — pick a rack (1–6) + bin (1–18): *Auto find rack* (lock the
  rack's anchor AprilTag, ids 101–106), *Move to bin* (zig-zag + visual-servo onto the bin
  tag), *Scan entire rack* (full 18-bin sweep). Coarse moves route through the collision-free
  planner; fine visual-servo nudges go straight-line.
- **Precise landing** on the charging dock — *Go to dock* / *LAND* / *ABORT*.
- **Live camera** — tag-detection overlay, front colour/IR, or the down landing cam.

A matching RViz view ships in `config/rviz/warehouse.rviz` (`roslaunch inspection_core
rviz.launch`): the standard PAIRS drone view plus the Mid-360 cloud, the live octomap map, the
planned path, and the bumper sectors.

```bash
roslaunch inspection_core inspection.launch        # standalone window (reads $UAV_NAME)
rosrun   inspection_core inspection_gui             # equivalent
# or inside any rqt session:  Plugins → PAIRS → PAIRS Inspection Control
```

It is also brought up by the warehouse sim (the `inspect` tmux window and
`inspection_gazebo full_sim.launch`). The panel only **consumes** `/<uav>/tag_detections`;
the AprilTag detector runs separately (`inspection_gazebo apriltag.launch`).

## BIN map

`config/bin_map/warehouse_bins.yaml` is generated by `generate_bin_map.py` and is keyed to
the simulation racks in `inspection_sim/gazebo_sim` (6 racks × 3 columns × 6 levels = 108
BINs, ids `R<rack>B<nn>` with 18 bins per rack). Each record stores the **drone standoff
pose** required to read a BIN's label — not just the BIN position — because that is the
actionable quantity for navigation. The simulation map is *synthetic*; the **real** map is
built during a survey/mapping pass (see below).

## Building the real BIN map — `scripts/bin_map_recorder.py`

During the mapping pass, teleop the drone to each BIN's viewing pose and snapshot it:

```bash
rosrun inspection_core bin_map_recorder.py \
    _world_frame:=world_origin _base_frame:=uav1/fcu \
    _output_file:=$(rospack find inspection_core)/config/bin_map/warehouse_bins.yaml
# at each BIN (drone holding the standoff pose):
rostopic pub -1 /bin_map_recorder/record std_msgs/String "data: 'A2'"
# when done:
rostopic pub -1 /bin_map_recorder/save   std_msgs/Empty "{}"
```

It records the drone's current **world pose** as that BIN's `standoff_pose`, projects the
`bin_pose` forward by the standoff distance, tags the nearest AprilTag anchor, and writes
`warehouse_bins.yaml` + a `tag_id → world_pose` anchor table. Optional `_qr_enabled:=true`
auto-records BINs from decoded beam QR codes. Method + frame anchoring are documented in
`docs/bin_map_building.md` (local).
