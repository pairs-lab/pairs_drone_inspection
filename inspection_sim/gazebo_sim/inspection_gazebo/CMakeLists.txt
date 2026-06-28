cmake_minimum_required(VERSION 3.5)
project(inspection_gazebo)

find_package(catkin REQUIRED)

catkin_package()

# Install simulation assets (also makes them available in the .deb).
# USE_SOURCE_PERMISSIONS keeps the +x bit on tmux/warehouse/{start,kill}.sh; without it
# install(DIRECTORY) drops to 0644 and `./start.sh` fails. tmux/ ships the launcher AND
# its co-located config/ (start.sh loads ./config/*.yaml relative to its own dir).
install(DIRECTORY launch worlds models config tmux
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

install(PROGRAMS scripts/spawn_inspection_uav.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
