cmake_minimum_required(VERSION 3.8)
project(typego_sdk)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(CURL REQUIRED)

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
  ament_lint_auto_find_test_dependencies()
endif()

# ----------- waypoints_node -----------
find_package(typego_interface REQUIRED)
add_executable(waypoints_service_node src/waypoints_service.cpp)
set_target_properties(waypoints_service_node PROPERTIES
  CXX_STANDARD 17
  CXX_STANDARD_REQUIRED ON
)
target_link_libraries(waypoints_service_node PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${typego_interface_TARGETS}
  ${visualization_msgs_TARGETS}
  rclcpp::rclcpp
  tf2::tf2
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
  ${OpenCV_LIBS}
  ${CURL_LIBRARIES}
)
# Link filesystem library if needed (for GCC < 9)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
  target_link_libraries(waypoints_service_node PUBLIC stdc++fs)
endif()

# ----------- install targets -----------
install(TARGETS
  waypoints_service_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY resource DESTINATION share/${PROJECT_NAME}/)

ament_package()
