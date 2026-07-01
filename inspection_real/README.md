# inspection_real

Real-robot deployment for the warehouse cycle-count inspection drone — the on-hardware
counterpart of `inspection_sim`. It runs the **same `inspection_core` mission layer** on the
PAIRS UAV system, but with real sensors, a real Pixhawk 6X, and GPS-denied SLAM. It mirrors
the structure of `pairs_uav_deployment` (the PAIRS field-deployment package).

> **Status: deployment scaffold.** The plumbing is complete and consistent with the sim, but
> every value tagged **⚠ TODO / measure** below must be filled in on the physical airframe,
> and the whole stack must be validated on hardware (tethered, props-off first). Nothing here
> has flown yet.

## What deploys — real-world readiness matrix

| Subsystem | Status | What ships here | What you must still do on hardware |
|---|---|---|---|
| **PX4 / Pixhawk 6X (hw_api)** | 🟢 ready | `RUN_TYPE=realworld` → `pairs_uav_px4_api` starts MAVROS↔`/dev/pixhawk:2000000`; downward ToF arrives free via `mavros/distance_sensor/garmin` | Flash PX4 airframe + `SER_TEL2_BAUD=2000000`, ESC/motor + full sensor calibration, RC + failsafe |
| **SLAM / localization** | 🟡 partial | Point-LIO on the real Mid-360 as the *only* estimator (`custom_config.yaml`, `localization.launch`) | **Measure `fcu→livox`** extrinsic; set Mid-360/Jetson IPs; validate drift + a level, stationary init |
| **Obstacle avoidance / navigation** | 🟡 partial | octomap planner + bumper off the Mid-360 (`avoidance.launch` → `inspection_core`), aisle-tuned clearances | Confirm the real cloud topic; re-tune `safe_obstacle_distance`/bumper on a real flight |
| **Inspection strategy** | 🟢 ready | rack/bin zig-zag + visual-servo + the rqt panel (`inspection_core`, unchanged) | Re-key `RACKS_CONFIG` to the real rack layout; re-tune servo gains for the real camera |
| **3D BIN map** | 🟡 partial | survey tool (`inspection_core/bin_map_recorder.py`) + real anchor AprilTag config | Physically survey the warehouse (build `warehouse_bins.yaml`); mount + measure anchor tags |
| **Sensor drivers** | 🟡 partial | Livox Mid-360 (driver + `MID360_config.json`), RealSense D455 (`realsense_front.launch`), sensor TFs | Set IPs/serials; **install `realsense2-camera` on the Jetson (arm64)**; wire the down cam |
| **Jetson / platform** | 🟡 partial | udev / netplan / chrony / systemd + preflight (`system/`, `scripts/`) | **Build an arm64 image** (see Risks); set device ids/IPs; apply system files |
| **Precise landing** | 🟡 partial | `pairs_precise_landing` chain wired in the tmux (`precland` window) | Fit + calibrate the downward camera; print + place the recursive landing pad |
| **QR / goods-label decode** | 🔴 missing | — (capture point is reached; decode is a stub) | Add a QR/barcode + OCR node feeding the capture step |
| **Perception label pipeline** | 🔴 missing | — | Wire decode → inventory reconciliation |

🟢 ready · 🟡 needs hardware measurement/validation · 🔴 not implemented yet

## Package layout

```
inspection_real/
├── config/
│   ├── custom_config.yaml       # Point-LIO as the only estimator + obstacle_bumper + takeoff
│   ├── network_config.yaml      # robot_names: [uav1]
│   ├── worlds/world_warehouse.yaml   # GPS-denied local origin + indoor safety area  (survey)
│   ├── drivers/MID360_config.json    # Livox network + extrinsic  (set IPs)
│   └── apriltag/apriltag_real.yaml   # forward-cam rack-anchor detector  (real ids/sizes)
├── launch/
│   ├── sensors.launch           # RealSense D455 + sensor TFs (Livox owned by localization)
│   ├── localization.launch      # real Livox driver + Point-LIO  (measured fcu->livox TF)
│   ├── avoidance.launch         # -> inspection_core/avoidance.launch, real cloud + frame
│   ├── apriltag.launch          # rack/bin detector on the real forward camera
│   ├── realsense_front.launch   # D455 bring-up (guarded; needs realsense2_camera)
│   └── bringup.launch           # hw_api + status + core (one-shot / systemd)
├── tmux/warehouse/{start,kill,record}.sh   # the real-robot session (RUN_TYPE=realworld)
├── scripts/{preflight_check,install_system}.sh
└── system/{udev,netplan,chrony,systemd}/   # Jetson OS bring-up files
```

## Run it (on the drone)

```bash
# one-time: install OS files (review IPs/ids first), then reboot / replug sensors
sudo rosrun inspection_real install_system.sh

# each flight:
roscd inspection_real/tmux/warehouse && ./start.sh     # ./kill.sh to stop
rosrun inspection_real preflight_check.sh              # BEFORE arming — must PASS
```

The session opens: `roscore`, `simtime` (use_sim_time false), `sensors`, `lio`, `hw_api`,
`status`, `core`, `autostart`, `avoid`, `inspect`, `precland`, `rviz`, `rosbag`, `kill`.
Once hovering, drive the mission from the **inspect** window's rqt panel (same panel as sim).

## Hardware bring-up checklist (ordered)

1. **Airframe** — assemble; measure `uav_mass`, motor thrust curve, arm length; author the real
   `platform_config` (start from `pairs_uav_gazebo_simulation` `x500.yaml`, replace measured values).
2. **Pixhawk 6X** — flash PX4; set the airframe, `SER_TEL2_BAUD=2000000`, MAVLink on TEL2; do
   accel/gyro/mag/level + ESC calibration; bind RC + a **kill switch**; set battery + failsafe.
   Flash `pairs_uav_deployment/miscellaneous/pixhawk_sdcard_config/pixhawk_6x_6c/etc/extras.txt`.
3. **Downward ToF** — wire to the Pixhawk, configure as a PX4 distance sensor (downward), confirm
   `DISTANCE_SENSOR` streams (it reaches ROS as `mavros/distance_sensor/garmin`).
4. **Jetson** — flash JetPack (Ubuntu 20.04/arm64); install the PAIRS + inspection stack (arm64,
   see Risks); `sudo rosrun inspection_real install_system.sh`; set `RUN_TYPE=realworld`,
   `UAV_NAME`, `UAV_TYPE` in the profile.
5. **Sensors** — Mid-360 static IP on `eth0` (match `MID360_config.json` + netplan); install
   `realsense2-camera`; set the D455 serial; fit + calibrate the downward camera.
6. **Extrinsics** — measure `fcu→livox` (in `localization.launch`) and the sensor mount TFs
   (in `sensors.launch`).
7. **Survey** — mount rack anchor tags, key `apriltag_real.yaml` + `RACKS_CONFIG`, fly the survey
   pass with `bin_map_recorder.py` → `warehouse_bins.yaml`.
8. **Bench** — props OFF: `./start.sh`, `preflight_check.sh` must PASS, verify RC/kill switch,
   watch Point-LIO odom + octomap in RViz while walking the drone.
9. **Tethered hover**, then short **manual aisle flight**, then enable autonomous nav.

## Hardware BOM

- **Pixhawk 6X** flight controller (PX4) — control, IMU, RC, battery telemetry, ToF forwarding
- **Jetson Orin NX** (arm64 companion) — Point-LIO, octomap, apriltag, mission, MAVROS
- **Livox Mid-360** (Ethernet LiDAR+IMU) — GPS-denied SLAM + obstacle avoidance
- **Intel RealSense D455** (front, USB3) — rack/bin AprilTag reading + label capture
- **Downward ToF** (Garmin LIDAR-Lite v3 or equiv., I²C→Pixhawk) — AGL / min-height / bumper down
- **Downward camera** (bluefox/USB/CSI) — precise-landing fiducial on the charging dock
- RC transmitter + receiver (manual takeover + **kill switch**); LiPo + monitoring; wired eth for the Mid-360

## Open risks / blockers

- **arm64 image (biggest gap).** The published Docker/apt are **amd64**; the Jetson Orin NX is
  **arm64**. Build `pairs_system` + the inspection debs for `linux/arm64` (native on the Jetson or
  `docker buildx`) before deploying. Nothing here runs on the Jetson until that exists.
- **QR/label decode is not implemented.** The drone reaches each bin's capture pose and centres,
  but there is no goods-label reader yet — the inventory-reconciliation half of the mission is open.
- **Unvalidated flight tuning.** Point-LIO extrinsic/drift, the narrow-aisle planner/bumper
  clearances, the airframe thrust model, and the octomap world frame all need a real flight to lock.
- **Safety.** Indoor = no GPS failsafe. Verify the RC kill switch, the geofence
  (`world_warehouse.yaml` safety area), and the estimator-innovation eland behaviour on the bench.
- **EMI / mounting.** Keep the Mid-360 clear of motor/ESC EMI and vibration; a bad mount degrades LIO.
