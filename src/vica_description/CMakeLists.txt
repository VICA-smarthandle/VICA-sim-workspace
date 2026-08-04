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

install(
  PROGRAMS scripts/nav2_startup_gate.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME nav2_startup_gate
)

ament_package()
