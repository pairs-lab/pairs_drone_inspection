#!/bin/bash
# Stamp the warehouse RViz config with the active UAV_NAME (mirrors
# pairs_uav_core/scripts/refactor_rviz_config.sh) so the hard-coded uav1 topics
# follow $UAV_NAME, then rviz.launch loads /tmp/warehouse.rviz.

PACKAGE_PATH=$(rospack find inspection_core)

cp "$PACKAGE_PATH/config/rviz/warehouse.rviz" /tmp/warehouse.rviz

sed -i "s/uav[0-9]/$UAV_NAME/g" /tmp/warehouse.rviz
