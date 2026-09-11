cmake_minimum_required(VERSION 3.8)
project(pipe_track_planning)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Eigen3 REQUIRED)

# --- ORIGINAL NODE ---
add_executable(point_extract 
  src/point_extract.cpp
  src/explore_2d.cpp
  src/navigation_utils.cpp
)
ament_target_dependencies(point_extract 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs
  OpenCV
  cv_bridge
  Eigen3
)
target_link_libraries(point_extract Eigen3::Eigen)
target_include_directories(point_extract PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(point_extract PUBLIC c_std_99 cxx_std_17)

# --- NEW MAP-BASED NODE ---
add_executable(point_extract_navi 
  src/point_extract_navi.cpp
  src/explore_2d.cpp
)
ament_target_dependencies(point_extract_navi 
  rclcpp 
  std_msgs 
  sensor_msgs 
  nav_msgs
  OpenCV
  cv_bridge
  Eigen3
)
target_link_libraries(point_extract_navi Eigen3::Eigen)
target_include_directories(point_extract_navi PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(point_extract_navi PUBLIC c_std_99 cxx_std_17)

# --- INSTALL BOTH EXECUTABLES ---
install(TARGETS 
  point_extract
  point_extract_navi
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()