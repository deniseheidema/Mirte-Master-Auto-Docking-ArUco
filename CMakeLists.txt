cmake_minimum_required(VERSION 3.5)
project(auto_marker_docking)

# Default C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

# === Executables ===

add_executable(auto_marker_docking_node src/auto_marker_docking_node.cpp)
ament_target_dependencies(auto_marker_docking_node
  rclcpp
  sensor_msgs
  geometry_msgs
  cv_bridge
  OpenCV
  Eigen3
)

add_executable(aruco_simple_detector src/aruco_simple_detector.cpp)
ament_target_dependencies(aruco_simple_detector
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
)

# Install executables
install(TARGETS
  auto_marker_docking_node
  aruco_simple_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install params
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
