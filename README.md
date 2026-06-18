# PAIRS Drone Inspection

Autonomous-drone **warehouse inventory cycle-count** system, built on the
[PAIRS UAV system](https://github.com/pairs-lab/pairs_uav_system). An operator selects a
rack/BIN; the drone autonomously flies a narrow GPS-denied aisle to a precise standoff
pose, reads the goods label, and the result is reconciled against the inventory system.

The repository is organised like `kr_autonomous_flight` (a clean `core / sim / real`
split) but the technique is **PAIRS**: ROS Noetic / catkin, the PAIRS manager-plugin
control & estimation stack, and the PAIRS bloom→`.deb`→signed-apt→Docker packaging
pipeline.

## Layout

```
pairs_drone_inspection/
├── inspection_core/   mission autonomy ON TOP of PAIRS — semantic BIN map + go-to-BIN logic
│                      (control/estimation/planning come from the PAIRS UAV system, not here)
├── inspection_sim/
│   ├── gazebo_sim/    Gazebo Classic 11 warehouse world + inspection drone (inspection_gazebo)
│   └── isaac_sim/     NVIDIA Isaac Sim 6.0 interactive POC (perception + operator UI)
└── inspection_real/   real-robot deployment configs (Pixhawk 6X + Jetson Orin NX)
```

## Quick start (Gazebo simulation)

Requires a built + sourced workspace with the PAIRS UAV system (ROS Noetic).

```bash
# full bring-up: warehouse world + inspection drone + autonomy core + takeoff
roscd inspection_gazebo/tmux/warehouse && ./start.sh     # ./kill.sh to stop

# or gazebo + drone only:
roslaunch inspection_gazebo full_sim.launch
```

The drone spawns with a downward ToF (Garmin rangefinder), a Livox Mid-360, and a forward
camera; the `goto` window flies it down the aisle centerline. See
[inspection_sim/gazebo_sim/inspection_gazebo/README.md](inspection_sim/gazebo_sim/inspection_gazebo/README.md).

## Install (apt / Docker)

Prebuilt packages are published to the project apt repository and a Docker image; see the
packaging directories. The apt repo serves signed ROS Noetic `.deb`s; the Docker image
bundles the runnable stack.

## License

BSD-3-Clause — see [LICENSE](LICENSE). Built on the PAIRS UAV system (BSD-3-Clause),
itself derived from the CTU MRS UAV system.
