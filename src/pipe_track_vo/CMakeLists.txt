cmake_minimum_required(VERSION 3.8)
project(pipe_track_vo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc calib3d video features2d highgui)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_library(${PROJECT_NAME}_lib
  src/feature_tracker.cpp
  src/visual_odometry.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  Eigen3::Eigen
)

add_executable(vo_node
  src/feature_tracker.cpp
  src/visual_odometry.cpp
  src/vo_node.cpp
)
target_link_libraries(vo_node
  ${PROJECT_NAME}_lib
  ${OpenCV_LIBS}
  Eigen3::Eigen
)
ament_target_dependencies(vo_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  cv_bridge
  image_transport
  tf2
  tf2_ros
  tf2_geometry_msgs
)

install(TARGETS ${PROJECT_NAME}_lib vo_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_package()