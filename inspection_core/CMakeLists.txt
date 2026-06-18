cmake_minimum_required(VERSION 3.5)
project(inspection_core)

find_package(catkin REQUIRED)

catkin_package()

# Install the semantic BIN map + config (also shipped in the .deb).
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
