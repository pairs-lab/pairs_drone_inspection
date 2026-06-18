cmake_minimum_required(VERSION 3.5)
project(inspection_gazebo)

find_package(catkin REQUIRED)

catkin_package()

# Install simulation assets (also makes them available in the .deb).
install(DIRECTORY launch worlds models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(PROGRAMS scripts/spawn_inspection_uav.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
