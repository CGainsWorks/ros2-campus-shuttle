cmake_minimum_required(VERSION 3.16)
project(shuttle_control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(shuttle_msgs REQUIRED)

# The ROS-free controller core. Built here as a subproject so the node, the
# Nav2 plugins and the offline benchmark all link the identical objects.
# The flag has to be set before the subdirectory is added, or the core's test
# target is never created and `colcon test` silently skips the control tests.
if(BUILD_TESTING)
  set(SHUTTLE_CORE_BUILD_TESTS ON)
endif()
add_subdirectory(core)

set(NODE_DEPS
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  shuttle_msgs
)

add_executable(path_tracker_node src/path_tracker_node.cpp)
target_link_libraries(path_tracker_node shuttle_control_core)
ament_target_dependencies(path_tracker_node ${NODE_DEPS})

add_executable(metrics_recorder_node src/metrics_recorder_node.cpp)
target_link_libraries(metrics_recorder_node shuttle_control_core)
ament_target_dependencies(metrics_recorder_node ${NODE_DEPS})

add_library(shuttle_nav2_controllers SHARED src/nav2_shuttle_controllers.cpp)
target_include_directories(shuttle_nav2_controllers PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(shuttle_nav2_controllers shuttle_control_core)
ament_target_dependencies(shuttle_nav2_controllers
  ${NODE_DEPS}
  nav2_core
  nav2_costmap_2d
  nav2_util
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core plugins.xml)

install(TARGETS path_tracker_node metrics_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS shuttle_control_core shuttle_nav2_controllers
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS controller_bench
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include/
)
install(DIRECTORY core/include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Per-file copyright headers are not used in this repository; the licence is
  # declared once in LICENSE. cpplint overlaps with the uncrustify style already
  # enforced, so it is skipped rather than fought.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(shuttle_control_core shuttle_nav2_controllers)
ament_export_dependencies(${NODE_DEPS} nav2_core nav2_costmap_2d pluginlib)
ament_package()
