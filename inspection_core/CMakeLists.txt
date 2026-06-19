cmake_minimum_required(VERSION 3.5)
project(inspection_core)

find_package(catkin REQUIRED)

catkin_package()

# Install the semantic BIN map + config (also shipped in the .deb).
install(DIRECTORY config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Survey tool: records BIN poses during the mapping pass -> warehouse_bins.yaml
catkin_install_python(PROGRAMS scripts/bin_map_recorder.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
