cmake_minimum_required(VERSION 3.8)
project(atlas_bringup)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_dfki_planner REQUIRED)
find_package(ad_ros2 REQUIRED)

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

#### HelloWorld Action ####

add_library(hello_world_action SHARED
  src/hello_world_action.cpp
)
target_include_directories(hello_world_action PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(hello_world_action PUBLIC
  behaviortree_cpp::behaviortree_cpp
  nav2_util::nav2_util_core
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  rclcpp_lifecycle::rclcpp_lifecycle
)

# necessary for macro BT_REGISTER_NODES to export function
target_compile_definitions(hello_world_action PRIVATE BT_PLUGIN_EXPORT)

install(
  TARGETS hello_world_action
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

#### Elevation Layer ####

add_library(nav2_elevation_costmap_plugin_core SHARED
  src/elevation_layer.cpp
)
ament_target_dependencies(nav2_elevation_costmap_plugin_core
  rclcpp
  nav2_core
  nav2_costmap_2d
  pluginlib
  nav_msgs
)
include_directories(include)
install(TARGETS nav2_elevation_costmap_plugin_core
  DESTINATION lib
)



install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY
  config
  launch
  models
  worlds
  maps
  behavior_trees
  DESTINATION share/${PROJECT_NAME}/
)

pluginlib_export_plugin_description_file(nav2_costmap_2d elevation_layer.xml)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_targets(hello_world_action HAS_LIBRARY_TARGET)
# ament_export_dependencies()
ament_package()
