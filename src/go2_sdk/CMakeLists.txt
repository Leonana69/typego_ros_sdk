cmake_minimum_required(VERSION 3.8)
project(go2_sdk)

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
find_package(typego_sdk REQUIRED)

# Add include directory
include_directories(include)

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

# ----------- lidar_service_node -----------
add_executable(lidar_service_node src/lidar_service.cpp)
ament_target_dependencies(lidar_service_node PUBLIC
  rclcpp
  sensor_msgs
  tf2_ros
  typego_sdk
)
target_link_libraries(lidar_service_node PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# ----------- tf_service_node -----------
add_executable(tf_service_node src/tf_service.cpp)
ament_target_dependencies(tf_service_node PUBLIC
  rclcpp
  sensor_msgs
  tf2_ros
  typego_sdk
)
target_link_libraries(tf_service_node PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  tf2_ros::static_transform_broadcaster_node
  tf2_ros::tf2_ros
)

# ----------- video_service_node -----------
pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0)

add_executable(video_service_node src/video_service.cpp)
ament_target_dependencies(video_service_node PUBLIC
  rclcpp
  sensor_msgs
  typego_sdk
)
target_include_directories(video_service_node PRIVATE
  ${GST_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(video_service_node PUBLIC
  ${sensor_msgs_TARGETS}
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs_library
  ${GST_LIBRARIES}
  ${OpenCV_LIBS}
)

# ----------- cmd_vel_controller_node -----------
add_executable(cmd_vel_controller_node src/cmd_vel_controller.cpp)
ament_target_dependencies(cmd_vel_controller_node PUBLIC
  rclcpp
  geometry_msgs
  typego_sdk
)
target_link_libraries(cmd_vel_controller_node PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  ${CURL_LIBRARIES}
)

# ----------- install targets -----------
install(TARGETS
  lidar_service_node
  video_service_node
  tf_service_node
  cmd_vel_controller_node
  DESTINATION lib/${PROJECT_NAME})

# install(DIRECTORY include/ DESTINATION include/)

ament_package()
