cmake_minimum_required(VERSION 3.5)
project(inspection_gazebo)

# Add necessary libraries for the Python node
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  apriltag_ros
)

catkin_package()

# ==========================================
# Add Python control node
catkin_install_python(PROGRAMS
  scripts/relative_navigator.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
# ==========================================

# Install simulation assets (also makes them available in the .deb).
install(DIRECTORY launch worlds models config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Install bash script
install(PROGRAMS scripts/spawn_inspection_uav.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)