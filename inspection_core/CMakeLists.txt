cmake_minimum_required(VERSION 3.5)
project(inspection_core)

find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_py)

# installs the src/inspection_core python module (the rqt panel)
catkin_python_setup()

catkin_package()

# Semantic BIN map + config, the rqt launch, and the rqt plugin manifest.
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# Survey tool (records BIN poses -> warehouse_bins.yaml) + the standalone GUI runner.
catkin_install_python(PROGRAMS
  scripts/bin_map_recorder.py
  scripts/inspection_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# RViz config refactor helper (stamps $UAV_NAME into warehouse.rviz; used by rviz.launch)
install(PROGRAMS scripts/refactor_rviz_config.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
