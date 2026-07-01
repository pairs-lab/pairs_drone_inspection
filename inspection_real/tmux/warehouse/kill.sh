#!/bin/bash
# Stop the real warehouse-inspection tmux session and its ROS nodes.
tmux kill-session -t warehouse_real 2>/dev/null
pkill -f 'roslaunch inspection_real' 2>/dev/null
pkill -f 'roslaunch pairs_uav_core core.launch' 2>/dev/null
echo "warehouse_real session stopped."
