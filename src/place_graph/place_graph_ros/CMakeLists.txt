cmake_minimum_required(VERSION 3.8)
project(place_graph_ros)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(place_graph REQUIRED)

add_executable(place_graph_node src/place_graph_node.cpp)
ament_target_dependencies(place_graph_node
  rclcpp
  nav_msgs
  geometry_msgs
  std_msgs
  visualization_msgs
  place_graph
)

install(TARGETS place_graph_node
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_uncrustify_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
