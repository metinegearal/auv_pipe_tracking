cmake_minimum_required(VERSION 3.8)
project(pipe_track_estimation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GTSAM REQUIRED)

# Include directories
include_directories(
  include
  ${GTSAM_INCLUDE_DIR}
)

# Add the main executable
add_executable(estimation_node
  src/estimation_node.cpp
  src/gtsam_estimator.cpp
)

# Link ROS 2 and GTSAM libraries
ament_target_dependencies(estimation_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
)
target_link_libraries(estimation_node gtsam)

install(TARGETS estimation_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME})

ament_package()