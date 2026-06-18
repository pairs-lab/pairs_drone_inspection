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

## Install

### Option A — apt (ROS Noetic, Ubuntu 20.04)

Requires a ROS Noetic install (`ros-noetic-desktop-full` recommended, for Gazebo/RViz). The
inspection packages depend on the PAIRS UAV system, so add **both** signed apt repos — they
share one PAIRS Lab signing key:

```bash
# signing key (one time)
curl -fsSL https://thanhnguyencanh.github.io/apt/KEY.gpg \
  | sudo gpg --dearmor -o /usr/share/keyrings/pairs.gpg

# main PAIRS repo (dependencies: control, estimation, gazebo sim, ...)
echo "deb [signed-by=/usr/share/keyrings/pairs.gpg] https://thanhnguyencanh.github.io/apt noetic main" \
  | sudo tee /etc/apt/sources.list.d/pairs.list
# inspection project repo
echo "deb [signed-by=/usr/share/keyrings/pairs.gpg] https://thanhnguyencanh.github.io/pairs_drone_inspection_apt noetic main" \
  | sudo tee /etc/apt/sources.list.d/pairs-inspection.list

sudo apt update
sudo apt install ros-noetic-inspection-core ros-noetic-inspection-gazebo
```

Then launch the simulation:

```bash
source /opt/ros/noetic/setup.bash
roslaunch inspection_gazebo full_sim.launch                 # gazebo + warehouse + drone
# or the full one-command bring-up (adds autonomy core + takeoff):
roscd inspection_gazebo/tmux/warehouse && ./start.sh        # ./kill.sh to stop
```

### Option B — Docker (no host ROS install)

The image bundles the full PAIRS UAV system **plus** the inspection packages:

```bash
docker pull thanhnc19/pairs_drone_inspection:noetic
```

Run it with GUI (RViz/Gazebo) and the GPU:

```bash
xhost +local:root
docker run -it --rm --name pairs_inspection \
  --net host --privileged --gpus all \
  --env DISPLAY="$DISPLAY" --env QT_X11_NO_MITSHM=1 \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  thanhnc19/pairs_drone_inspection:noetic bash
```

Then, inside the container:

```bash
roscd inspection_gazebo/tmux/warehouse && ./start.sh        # ./kill.sh to stop
```

Drop `--gpus all` if you have no NVIDIA container runtime (Gazebo falls back to CPU
rendering). Mount a host overlay workspace with `--volume <host>/pairs_ws:/opt/pairs_ws:rw`
to develop your own packages on top.

## License

BSD-3-Clause — see [LICENSE](LICENSE). Built on the PAIRS UAV system (BSD-3-Clause),
itself derived from the CTU MRS UAV system.
