################################################################################
# Minimum CMake, project name, and compile options
################################################################################
cmake_minimum_required(VERSION 3.5)
project(team1_tb3_sim)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(MSVC)
  add_compile_definitions(_USE_MATH_DEFINES)
endif()

################################################################################
# Find ament & system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(gazebo_ros_pkgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
# For Python runtime nodes (installed as scripts)
# rclpy is an exec_depend in package.xml (no find_package needed here)

################################################################################
# Build (C++ targets are optional; Python can coexist)
################################################################################
# Gazebo link/include paths
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(
  include
  ${GAZEBO_INCLUDE_DIRS}
)

# Common dependency list for C++ targets
set(dependencies
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2
)

# ---- Example C++ node (optional) ----
# You can remove this if you don't need a C++ node yet.
set(EXEC_NAME "team1_tb3_drive")
if (EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/team1_tb3_drive.cpp")
  add_executable(${EXEC_NAME} src/team1_tb3_drive.cpp)
  ament_target_dependencies(${EXEC_NAME} ${dependencies})
endif()


# Install Python executables (so `ros2 run` works)
install(PROGRAMS
  src/object_detection_node.py
  src/angle_from_pixel_node.py
  src/control_node.py
  src/distance_from_yolo_node.py
  DESTINATION lib/${PROJECT_NAME}
)

################################################################################
# Install data (launch, models, rviz, urdf, worlds) and headers
################################################################################
install(DIRECTORY launch models rviz urdf worlds
  DESTINATION share/${PROJECT_NAME}/
)

install(DIRECTORY include/
  DESTINATION include/
)

################################################################################
# Export for ament
################################################################################
ament_export_include_directories(include)
ament_export_dependencies(gazebo_ros_pkgs)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(rclcpp)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(tf2)
ament_package()
