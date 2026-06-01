cmake_minimum_required(VERSION 3.8)
project(typego_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(place_graph REQUIRED)
find_package(typego_interface REQUIRED)

# Add include directory
include_directories(include)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  # uncrustify is skipped (as in place_graph): the project does not
  # enforce a uniform auto-format across packages.
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_waypoint_anchor
    test/test_waypoint_anchor.cpp
    src/waypoint_anchor.cpp
    src/user_waypoint_store.cpp)
  ament_target_dependencies(test_waypoint_anchor place_graph)
endif()

# ----------- place_graph_waypoints_node -----------
add_executable(place_graph_waypoints_node
  src/place_graph_waypoints.cpp
  src/waypoint_anchor.cpp
  src/user_waypoint_store.cpp)
set_target_properties(place_graph_waypoints_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
ament_target_dependencies(place_graph_waypoints_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  std_msgs
  typego_interface
  place_graph
  ament_index_cpp
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
  target_link_libraries(place_graph_waypoints_node PUBLIC stdc++fs)
endif()

# ----------- nav2_speed_service_node -----------
find_package(rcl_interfaces REQUIRED)
add_executable(nav2_speed_service_node src/nav2_speed_service.cpp)
set_target_properties(nav2_speed_service_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
target_link_libraries(nav2_speed_service_node PUBLIC
  rclcpp::rclcpp
  ${rcl_interfaces_TARGETS}
  ${typego_interface_TARGETS}
)

# ----------- get_position_node -----------
add_executable(get_position_node src/get_position.cpp)
set_target_properties(get_position_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
ament_target_dependencies(get_position_node PUBLIC
  rclcpp
  geometry_msgs
  tf2_ros
)
target_link_libraries(get_position_node PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::tf2_ros
)
# Link filesystem library if needed (for GCC < 9)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
  target_link_libraries(get_position_node PUBLIC stdc++fs)
endif()

# ----------- install targets -----------
install(TARGETS
  place_graph_waypoints_node
  nav2_speed_service_node
  get_position_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME}/)

ament_package()
