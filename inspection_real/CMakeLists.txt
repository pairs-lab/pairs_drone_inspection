cmake_minimum_required(VERSION 3.5)
project(inspection_real)

find_package(catkin REQUIRED)

catkin_package()

# configs + launch (read-only data)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

# tmux sessions, preflight/install scripts, and the OS system files (udev/netplan/chrony/
# systemd) — keep the +x bit on the .sh entrypoints.
install(DIRECTORY tmux scripts system
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)
