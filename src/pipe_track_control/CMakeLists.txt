cmake_minimum_required(VERSION 3.8)
project(pipe_track_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(nlohmann_json REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

add_executable(dlqr_control_node src/dlqr_control_node.cpp)
ament_target_dependencies(dlqr_control_node
  rclcpp rcl_interfaces geometry_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(dlqr_control_node Eigen3::Eigen nlohmann_json::nlohmann_json)
target_compile_features(dlqr_control_node PRIVATE cxx_std_17)

install(TARGETS dlqr_control_node
  DESTINATION lib/${PROJECT_NAME})

add_executable(waypoint_control_node src/waypoint_control_node.cpp)
ament_target_dependencies(waypoint_control_node 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs
)
target_link_libraries(waypoint_control_node nlohmann_json::nlohmann_json)
target_include_directories(waypoint_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(waypoint_control_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS waypoint_control_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
