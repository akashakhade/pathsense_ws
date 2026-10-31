cmake_minimum_required(VERSION 3.8)
project(pathsense_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config rviz worlds web
  DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/arm_teleop_keyboard.py
  scripts/rviz_teleop_sim.py
  scripts/graph_navigator.py
  scripts/urdf_color_split.py
  scripts/phone_server.py
  DESTINATION lib/${PROJECT_NAME})

ament_package()