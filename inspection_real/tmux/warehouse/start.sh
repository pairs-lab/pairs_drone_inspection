#!/bin/bash
# REAL warehouse-inspection bring-up (Pixhawk 6X + Jetson Orin NX, GPS-denied), plain-tmux
# style matching the sim session. One window per section; ./kill.sh stops everything.
#
# Prerequisites on the Jetson (see ../../README.md + ../../scripts/install_system.sh):
#   - /dev/pixhawk udev symlink to the FCU; Livox Mid-360 reachable on eth0; RealSense on USB3
#   - PAIRS + inspection debs installed (arm64); RUN_TYPE=realworld in the profile
#   - real airframe platform_config, measured fcu->livox TF, surveyed BIN map + anchor tags
#
# Difference from sim: RUN_TYPE=realworld (starts MAVROS<->Pixhawk, not gazebo), real sensors,
# Point-LIO localization instead of sim-GPS, use_sim_time false.

SESSION_NAME=warehouse_real

SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# environment for every pane.
#   RUN_TYPE=realworld  -> pairs_uav_px4_api api.launch starts MAVROS (/dev/pixhawk:2000000)
#   UAV_MASS            -> REQUIRED on the realworld branch of core.launch (uav_manager reads
#                          $(env UAV_MASS)); there is NO default. ⚠ set the MEASURED all-up mass
#                          of the real airframe in kg (the 1.5 below is an x500 placeholder).
PRE_WINDOW='export UAV_NAME=uav1; export RUN_TYPE=realworld; export UAV_TYPE=x500; export WORLD_NAME=warehouse; export UAV_MASS=1.5'
SETUP="cd $SCRIPTPATH; $PRE_WINDOW"

if [ -n "$TMUX" ]; then echo "Already inside tmux, detach first."; exit 1; fi
if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "Session $SESSION_NAME already exists; attach with 'tmux a -t $SESSION_NAME' or ./kill.sh."
  exit 1
fi

# ---------------- window: roscore ----------------
read W_roscore P <<< "$(tmux new-session -d -s "$SESSION_NAME" -n "roscore" -x 250 -y 50 -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'roscore' Enter
tmux select-layout -t "$W_roscore" tiled

# ---------------- window: simtime (real clock, NOT gazebo) ----------------
read W_simtime P <<< "$(tmux new-window -t "$SESSION_NAME" -n "simtime" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForRos; rosparam set use_sim_time false' Enter
tmux select-layout -t "$W_simtime" tiled

# ---------------- window: sensors (forward RealSense + sensor TFs; Garmin ToF via MAVLink) ----------------
read W_sensors P <<< "$(tmux new-window -t "$SESSION_NAME" -n "sensors" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForRos; roslaunch inspection_real sensors.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_sensors" tiled

# ---------------- window: localization (Livox Mid-360 driver + Point-LIO) ----------------
read W_lio P <<< "$(tmux new-window -t "$SESSION_NAME" -n "lio" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForRos; roslaunch inspection_real localization.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_lio" tiled

# ---------------- window: hw_api (MAVROS <-> Pixhawk 6X over /dev/pixhawk) ----------------
read W_hw_api P <<< "$(tmux new-window -t "$SESSION_NAME" -n "hw_api" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForRos; roslaunch pairs_uav_px4_api api.launch' Enter
tmux select-layout -t "$W_hw_api" tiled

# ---------------- window: status ----------------
read W_status P <<< "$(tmux new-window -t "$SESSION_NAME" -n "status" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_status status.launch' Enter
tmux select-layout -t "$W_status" tiled

# ---------------- window: core (autonomy core on the real airframe + Point-LIO estimator) ----------------
# ⚠ platform_config defaults to the standard x500 model — replace with your MEASURED airframe.
read W_core P <<< "$(tmux new-window -t "$SESSION_NAME" -n "core" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; RD=`rospack find inspection_real`; roslaunch pairs_uav_core core.launch platform_config:=`rospack find pairs_uav_gazebo_simulation`/config/pairs_uav_system/$UAV_TYPE.yaml custom_config:=$RD/config/custom_config.yaml world_config:=$RD/config/worlds/world_warehouse.yaml network_config:=$RD/config/network_config.yaml' Enter
tmux select-layout -t "$W_core" tiled

# ---------------- window: autostart ----------------
read W_auto P <<< "$(tmux new-window -t "$SESSION_NAME" -n "autostart" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_autostart automatic_start.launch' Enter
tmux select-layout -t "$W_auto" tiled

# ---------------- window: avoid (octomap map + planner + reactive bumper off the Mid-360) ----------------
read W_avoid P <<< "$(tmux new-window -t "$SESSION_NAME" -n "avoid" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_real avoidance.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_avoid" tiled

# ---------------- window: inspect (rack/bin AprilTag detector + the operator rqt panel) ----------------
read W_inspect P <<< "$(tmux new-window -t "$SESSION_NAME" -n "inspect" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_real apriltag.launch uav_name:=$UAV_NAME' Enter
P=$(tmux split-window -t "$W_inspect" -P -F '#{pane_id}')
tmux select-layout -t "$W_inspect" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_core inspection.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_inspect" tiled

# ---------------- window: precland (precise landing on the charging dock; needs the down cam) ----------------
read W_precland P <<< "$(tmux new-window -t "$SESSION_NAME" -n "precland" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s roslaunch pairs_precise_landing precise_landing.launch camera_node:=bluefox_optflow image_topic:=image_raw' Enter
tmux select-layout -t "$W_precland" tiled

# ---------------- window: rviz (warehouse view: octomap map + Mid-360 + planned path) ----------------
read W_viz P <<< "$(tmux new-window -t "$SESSION_NAME" -n "rviz" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_core rviz.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_viz" tiled

# ---------------- window: rosbag (flight logging) ----------------
read W_bag P <<< "$(tmux new-window -t "$SESSION_NAME" -n "rosbag" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s ./record.sh' Enter
tmux select-layout -t "$W_bag" tiled

# ---------------- window: kill ----------------
read W_kill P <<< "$(tmux new-window -t "$SESSION_NAME" -n "kill" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SCRIPTPATH/kill.sh"

tmux set-option -t "$SESSION_NAME" mouse on
tmux select-window -t "$W_status"
tmux -2 attach-session -t "$SESSION_NAME"
