cmake_minimum_required(VERSION 3.8)
# Package name is `autonomy_2d`, not the directory name -- see package.xml.
project(autonomy_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(typego_interface REQUIRED)

# ----------- nav2_speed_service_node -----------
# Serves typego_interface/SetSpeed against Nav2's controller parameters. The
# 3D stack has its own SetSpeed server in local_planner, so this one moved
# here with the rest of the Nav2 integration.
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

install(TARGETS
  nav2_speed_service_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME}/)

ament_package()
