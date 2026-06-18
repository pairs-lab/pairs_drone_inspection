#!/bin/bash
# Spawn one inspection drone into the running Gazebo sim via the PAIRS drone spawner.
#
#   spawn_inspection_uav.sh [UAV_ID] [UAV_TYPE] [SENSOR_FLAGS...]
#
# Defaults to the warehouse-inspection sensor suite: downward ToF (Garmin
# rangefinder), Livox Mid-360, and a forward RealSense. Waits for the spawner
# service before issuing the request.
set -euo pipefail

UAV_ID="${1:-1}"
UAV_TYPE="${2:-x500}"
if [ "$#" -gt 2 ]; then
  shift 2
  SENSORS="$*"
else
  SENSORS="--enable-rangefinder --enable-livox --enable-realsense-front"
fi

echo "[spawn_inspection_uav] waiting for /pairs_drone_spawner/spawn ..."
rosservice call --wait /pairs_drone_spawner/spawn "${UAV_ID} --${UAV_TYPE} ${SENSORS}"
echo "[spawn_inspection_uav] spawned uav${UAV_ID} (${UAV_TYPE}) with: ${SENSORS}"
