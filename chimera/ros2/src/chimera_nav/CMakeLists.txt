cmake_minimum_required(VERSION 3.16)
project(chimera_nav)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Find CHIMERA core dependencies
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(GTSAM REQUIRED)
find_package(yaml-cpp REQUIRED)

# Include CHIMERA core
include_directories(
  include
  ${CMAKE_SOURCE_DIR}/../../../src  # Path to chimera core
  ${GTSAM_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
)

# Link to CHIMERA core library
link_directories(${CMAKE_SOURCE_DIR}/../../../build)

# CHIMERA ROS2 Node
add_executable(chimera_nav_node
  src/chimera_nav_node.cpp
  src/sensor_bridge.cpp
)

ament_target_dependencies(chimera_nav_node
  rclcpp
  std_msgs
  nav_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

target_link_libraries(chimera_nav_node
  chimera_core
  gtsam
  gtsam_unstable
  Eigen3::Eigen
  yaml-cpp
  boost_json
)

# Install targets
install(TARGETS
  chimera_nav_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
