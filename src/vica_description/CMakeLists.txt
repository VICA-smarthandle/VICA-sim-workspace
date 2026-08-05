cmake_minimum_required(VERSION 3.8)
project(vica_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    urdf
    meshes
    rviz
    config
    maps
  DESTINATION share/${PROJECT_NAME}
)

# Isaac Sim assets: the stage USDs, the scripts that rebuild every ROS 2
# interface graph and restore the joint drives, and the RTX lidar profile.
# USE_SOURCE_PERMISSIONS keeps export_isaac_urdf.sh executable; the default
# strips the bit.
#
# Note that export_isaac_urdf.sh writes urdf/vica.urdf next to itself, so run
# the copy in src/ -- an installed copy would regenerate into the install tree,
# where the result is neither committed nor the file anyone imports.
install(
  DIRECTORY isaac_vica_assets
  DESTINATION share/${PROJECT_NAME}
  USE_SOURCE_PERMISSIONS
)

install(
  PROGRAMS scripts/nav2_startup_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_startup_gate
)

ament_package()
