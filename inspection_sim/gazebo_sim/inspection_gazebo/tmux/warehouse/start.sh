#!/bin/bash
# Plain-tmux session: full warehouse-inspection sim bring-up (gazebo + warehouse
# world + drone with downward ToF/Livox + autonomy core + takeoff). One window per
# section; edit the send-keys lines to change what runs. ./kill.sh stops everything.

SESSION_NAME=warehouse

# absolute path to this script's directory; every pane starts here
SCRIPT=$(readlink -f "$0")
SCRIPTPATH=$(dirname "$SCRIPT")

# commands executed first in every pane
PRE_WINDOW='export UAV_NAME=uav1; export RUN_TYPE=simulation; export UAV_TYPE=x500'
SETUP="cd $SCRIPTPATH; $PRE_WINDOW"

if [ -n "$TMUX" ]; then
  echo "Already inside tmux, detach first."
  exit 1
fi

if tmux has-session -t "$SESSION_NAME" 2>/dev/null; then
  echo "Session $SESSION_NAME already exists; attach with 'tmux a -t $SESSION_NAME' or stop it with ./kill.sh."
  exit 1
fi

# ---------------- window: roscore ----------------
read W_roscore P <<< "$(tmux new-session -d -s "$SESSION_NAME" -n "roscore" -x 250 -y 50 -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'roscore' Enter
tmux select-layout -t "$W_roscore" tiled

# ---------------- window: gazebo (warehouse world + spawn inspection drone) ----------------
read W_gazebo P <<< "$(tmux new-window -t "$SESSION_NAME" -n "gazebo" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForRos; roslaunch inspection_gazebo simulation.launch gui:=true' Enter
P=$(tmux split-window -t "$W_gazebo" -P -F '#{pane_id}')
tmux select-layout -t "$W_gazebo" tiled
# downward ToF (--enable-rangefinder) + Livox Mid-360 (--enable-livox) + forward RealSense
# + downward bluefox camera (--enable-bluefox-camera) = the fiducial stream precise landing reads
tmux send-keys -t "$P" "$SETUP; "'waitForGazebo; sleep 5; waitForSpawn; rosservice call /pairs_drone_spawner/spawn "1 --$UAV_TYPE --pos -6.0 0.0 0.1 0.0 --enable-rangefinder --enable-livox --enable-realsense-front --enable-bluefox-camera"' Enter
P=$(tmux split-window -t "$W_gazebo" -P -F '#{pane_id}')
tmux select-layout -t "$W_gazebo" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; gz camera -c gzclient_camera -f $UAV_NAME; history -s gz camera -c gzclient_camera -f $UAV_NAME' Enter
tmux select-layout -t "$W_gazebo" tiled

# ---------------- window: status ----------------
read W_status P <<< "$(tmux new-window -t "$SESSION_NAME" -n "status" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_status status.launch' Enter
tmux select-layout -t "$W_status" tiled

# ---------------- window: hw_api ----------------
read W_hw_api P <<< "$(tmux new-window -t "$SESSION_NAME" -n "hw_api" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForTime; roslaunch pairs_uav_px4_api api.launch' Enter
tmux select-layout -t "$W_hw_api" tiled

# ---------------- window: core ----------------
read W_core P <<< "$(tmux new-window -t "$SESSION_NAME" -n "core" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_core core.launch platform_config:=`rospack find pairs_uav_gazebo_simulation`/config/pairs_uav_system/$UAV_TYPE.yaml custom_config:=./config/custom_config.yaml world_config:=./config/world_config.yaml network_config:=./config/network_config.yaml' Enter
tmux select-layout -t "$W_core" tiled

# ---------------- window: avoid (octomap map + collision-free planner + reactive bumper) ----------------
# Obstacle avoidance off the Livox Mid-360: octomap_server builds a live 3D map,
# octomap_planner serves collision-free goto (octomap_planner/goto), pairs_bumper
# feeds obstacle sectors the ControlManager repels from. Without this the drone
# flies straight-line goto and crashes into the racks.
read W_avoid P <<< "$(tmux new-window -t "$SESSION_NAME" -n "avoid" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_core avoidance.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_avoid" tiled

# ---------------- window: precland (apriltag detector + landing-pad LKF + descent controller) ----------------
# ONE launch transitively starts the detector -> landing_pad_estimation -> precise_landing chain.
# camera_node/image_topic point at the downward bluefox cam added to the spawn above.
read W_precland P <<< "$(tmux new-window -t "$SESSION_NAME" -n "precland" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch pairs_precise_landing precise_landing.launch apriltag_config:=./config/apriltag.yaml camera_node:=bluefox_optflow image_topic:=image_raw estimator_config:=./config/landing_estimator.yaml controller_config:=./config/landing_controller.yaml' Enter
tmux select-layout -t "$W_precland" tiled

# ---------------- window: takeoff ----------------
read W_takeoff P <<< "$(tmux new-window -t "$SESSION_NAME" -n "takeoff" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForHw; roslaunch pairs_uav_autostart automatic_start.launch' Enter
P=$(tmux split-window -t "$W_takeoff" -P -F '#{pane_id}')
tmux select-layout -t "$W_takeoff" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; until rosservice call /$UAV_NAME/hw_api/arming 1 | grep -q "success: True"; do sleep 1; done; sleep 2; rosservice call /$UAV_NAME/hw_api/offboard' Enter
tmux select-layout -t "$W_takeoff" tiled

# ---------------- window: goto (fly down the aisle; centerline y=0, +x) ----------------
# Two pre-loaded history entries (press Up): the collision-free planner goto (use
# this — routes around the racks), and the raw straight-line control_manager goto.
read W_goto P <<< "$(tmux new-window -t "$SESSION_NAME" -n "goto" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'history -s rosservice call /$UAV_NAME/control_manager/goto \"goal: \[3.0, 0.0, 1.5, 0.0\]\"; history -s rosservice call /$UAV_NAME/octomap_planner/goto \"goal: \[3.0, 0.0, 1.5, 0.0\]\"' Enter
tmux select-layout -t "$W_goto" tiled

# ---------------- window: inspect (rack/bin AprilTag detector + the operator rqt panel) ----------------
# pane 1 = front-camera detector -> /$UAV_NAME/tag_detections (+ tag_detections_image)
# pane 2 = the rqt operator panel (relative nav + precise-landing buttons + live camera feed).
# The panel's "Go to dock / LAND / ABORT" buttons replace the old standalone 'dock' CLI window.
read W_inspect P <<< "$(tmux new-window -t "$SESSION_NAME" -n "inspect" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_gazebo apriltag.launch uav_name:=$UAV_NAME' Enter
P=$(tmux split-window -t "$W_inspect" -P -F '#{pane_id}')
tmux select-layout -t "$W_inspect" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_core inspection.launch UAV_NAME:=$UAV_NAME' Enter
tmux select-layout -t "$W_inspect" tiled

# ---------------- window: viz (merged: warehouse rviz + robot model + rviz interface + i3 layout) ----------------
# Consolidates the former separate rviz / gui / layout windows. The flight-control
# GUI is no longer a separate pane here — it is merged into the inspection panel
# (inspect window). RViz loads inspection_core's warehouse config (octomap map +
# Mid-360 cloud + planned path) instead of the stock pairs_uav_core one.
read W_viz P <<< "$(tmux new-window -t "$SESSION_NAME" -n "viz" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch inspection_core rviz.launch UAV_NAME:=$UAV_NAME' Enter
P=$(tmux split-window -t "$W_viz" -P -F '#{pane_id}')
tmux select-layout -t "$W_viz" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch pairs_rviz_plugins load_robot.launch' Enter
P=$(tmux split-window -t "$W_viz" -P -F '#{pane_id}')
tmux select-layout -t "$W_viz" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; roslaunch pairs_rviz_plugins rviz_interface.launch' Enter
P=$(tmux split-window -t "$W_viz" -P -F '#{pane_id}')
tmux select-layout -t "$W_viz" tiled
tmux send-keys -t "$P" "$SETUP; "'waitForControl; sleep 3; ~/.i3/layout_manager.sh ./layout.json' Enter
tmux select-layout -t "$W_viz" tiled

# ---------------- window: kill (press enter inside to stop the session) ----------------
read W_kill P <<< "$(tmux new-window -t "$SESSION_NAME" -n "kill" -P -F '#{window_id} #{pane_id}')"
tmux send-keys -t "$P" "$SCRIPTPATH/kill.sh"

# mouse support (select panes / scroll with the mouse)
tmux set-option -t "$SESSION_NAME" mouse on

tmux select-window -t "$W_status"
tmux -2 attach-session -t "$SESSION_NAME"
