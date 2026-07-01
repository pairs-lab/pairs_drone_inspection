#!/bin/bash
# Flight logging for the real inspection drone. Records the safety-critical + mission topics
# to ~/bag_files (skips the raw camera streams by default to keep the bag small; add them if
# you need to re-run perception offline).
UAV_NAME=${UAV_NAME:-uav1}
mkdir -p ~/bag_files
cd ~/bag_files

rosbag record -o warehouse_real \
  /$UAV_NAME/hw_api/status \
  /$UAV_NAME/hw_api/distance_sensor \
  /$UAV_NAME/hw_api/imu \
  /$UAV_NAME/mavros/state \
  /$UAV_NAME/estimation_manager/diagnostics \
  /$UAV_NAME/estimation_manager/odom_main \
  /$UAV_NAME/control_manager/diagnostics \
  /$UAV_NAME/control_manager/control_reference \
  /$UAV_NAME/point_lio/odom \
  /$UAV_NAME/livox/points \
  /$UAV_NAME/bumper/obstacle_sectors \
  /$UAV_NAME/octomap_planner/diagnostics \
  /$UAV_NAME/tag_detections \
  /tf /tf_static \
  __name:=inspection_rosbag
