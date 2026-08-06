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

install(
  PROGRAMS scripts/compare_nav2_with_robot.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME compare_nav2_with_robot
)

install(
  PROGRAMS scripts/measure_on_robot.sh
  DESTINATION lib/${PROJECT_NAME}
  RENAME measure_on_robot
)

install(
  PROGRAMS scripts/measure_rotation.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME measure_rotation
)

install(
  PROGRAMS scripts/rotation_space.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME rotation_space
)

install(
  PROGRAMS scripts/measure_handle.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME measure_handle
)

ament_package()
