cmake_minimum_required(VERSION 3.5)
project(tare_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(Eigen3 REQUIRED)

# OR-Tools is vendored under or-tools/ — amd64 binaries are committed; arm64
# binaries are downloaded by `make or-tools` (invoked from the Dockerfile and
# `make setup`). Intentionally NOT declared in package.xml so rosdep doesn't
# try to resolve it. Prepending ORTOOLS_ROOT to CMAKE_PREFIX_PATH lets
# ortoolsConfig.cmake's find_dependency() calls (absl, protobuf, re2, Cbc,
# Clp, Cgl, Osi, scip, ZLIB) resolve against the vendored configs instead of
# system packages — system absl/protobuf are ABI-incompatible with v9.8.
set(ORTOOLS_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/or-tools")
list(PREPEND CMAKE_PREFIX_PATH "${ORTOOLS_ROOT}")

# or-tools/{include,lib,bin} are gitignored and fetched by `make or-tools`
# per-arch. A missing tree means the user hasn't run setup yet.
if(NOT EXISTS "${ORTOOLS_ROOT}/lib/cmake/ortools/ortoolsConfig.cmake")
  message(FATAL_ERROR
    "OR-Tools not found at ${ORTOOLS_ROOT}. Run `make or-tools` from the "
    "workspace root to fetch the arch-correct v9.8 tarball. This runs "
    "automatically inside Docker when AUTONOMY_TYPE=full.")
endif()

# Workaround: the OR-Tools v9.8 C++ runtime tarball ships *-targets.cmake
# files that import three executables (protoc, scip, fzn-cp-sat) which the
# tarball doesn't actually include. CMake's import-validator (introduced in
# protobuf 25 / OR-Tools v9.8) hard-fails if these files are missing. We
# never invoke any of them — tare_planner has no .proto sources, doesn't
# call SCIP standalone, and doesn't run flatzinc — so zero-byte placeholders
# satisfy the existence check without functional impact.
file(MAKE_DIRECTORY "${ORTOOLS_ROOT}/bin")
foreach(_ortools_stub protoc-25.0.0 scip fzn-cp-sat)
  if(NOT EXISTS "${ORTOOLS_ROOT}/bin/${_ortools_stub}")
    file(WRITE "${ORTOOLS_ROOT}/bin/${_ortools_stub}" "")
  endif()
endforeach()

find_package(ortools CONFIG REQUIRED
  PATHS "${ORTOOLS_ROOT}/lib/cmake/ortools"
  NO_DEFAULT_PATH)

include_directories(
  ${PCL_INCLUDE_DIRS}
  include
  ${EIGEN3_INCLUDE_DIR}
  ${ORTOOLS_ROOT}/include
)
# ortools::ortools propagates its include dirs to libraries that link it
# (e.g. tsp_solver). But several targets here #include tsp_solver.h
# transitively (grid_world.h → tsp_solver.h, tare_visualizer.h → grid_world.h)
# without listing tsp_solver in their target_link_libraries, so we add the
# OR-Tools include dir globally to match the codebase's global-include style.

add_library(lidar_model src/lidar_model/lidar_model.cpp)
ament_target_dependencies(lidar_model rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 visualization_msgs pcl_ros pcl_conversions)

add_library(graph src/graph/graph.cpp)
ament_target_dependencies(graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)

add_library(misc_utils src/utils/misc_utils.cpp)
ament_target_dependencies(misc_utils rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)

add_library(pointcloud_utils src/utils/pointcloud_utils.cpp)
ament_target_dependencies(pointcloud_utils rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)

add_library(tsp_solver src/tsp_solver/tsp_solver.cpp)
ament_target_dependencies(tsp_solver rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(tsp_solver ortools::ortools)

add_library(viewpoint src/viewpoint/viewpoint.cpp)
ament_target_dependencies(viewpoint rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(viewpoint lidar_model)

add_library(rolling_grid src/rolling_grid/rolling_grid.cpp)
ament_target_dependencies(rolling_grid rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(rolling_grid misc_utils pointcloud_utils)

add_library(viewpoint_manager src/viewpoint_manager/viewpoint_manager.cpp)
ament_target_dependencies(viewpoint_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(viewpoint_manager rolling_grid viewpoint grid_world)

add_library(local_coverage_planner src/local_coverage_planner/local_coverage_planner.cpp)
ament_target_dependencies(local_coverage_planner rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(local_coverage_planner viewpoint_manager)

add_library(grid_world src/grid_world/grid_world.cpp)
ament_target_dependencies(grid_world rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(grid_world viewpoint_manager tsp_solver keypose_graph exploration_path)

add_library(pointcloud_manager src/pointcloud_manager/pointcloud_manager.cpp)
ament_target_dependencies(pointcloud_manager rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)

add_library(keypose_graph src/keypose_graph/keypose_graph.cpp)
ament_target_dependencies(keypose_graph rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(keypose_graph ${PCL_LIBRARIES} viewpoint_manager)

add_library(rolling_occupancy_grid src/rolling_occupancy_grid/rolling_occupancy_grid.cpp)
ament_target_dependencies(rolling_occupancy_grid rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)

add_library(planning_env src/planning_env/planning_env.cpp)
ament_target_dependencies(planning_env rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(planning_env ${PCL_LIBRARIES} rolling_occupancy_grid)

add_library(exploration_path src/exploration_path/exploration_path.cpp)
ament_target_dependencies(exploration_path rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(exploration_path ${PCL_LIBRARIES})

add_library(tare_visualizer src/tare_visualizer/tare_visualizer.cpp)
ament_target_dependencies(tare_visualizer rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions visualization_msgs)
target_link_libraries(tare_visualizer ${PCL_LIBRARIES})

add_library(sensor_coverage_planner_ground src/sensor_coverage_planner/sensor_coverage_planner_ground.cpp)
ament_target_dependencies(sensor_coverage_planner_ground rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(sensor_coverage_planner_ground graph planning_env keypose_graph viewpoint_manager pointcloud_manager grid_world local_coverage_planner tare_visualizer)

add_executable(navigationBoundary src/navigation_boundary_publisher/navigationBoundary.cpp)
ament_target_dependencies(navigationBoundary rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2 pcl_ros pcl_conversions)
target_link_libraries(navigationBoundary ${PCL_LIBRARIES})

add_executable(tare_planner_node src/tare_planner_node/tare_planner_node.cpp)
ament_target_dependencies(tare_planner_node rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs tf2)
target_link_libraries(tare_planner_node sensor_coverage_planner_ground)

# Executables live at install/<pkg>/lib/<pkg>/; libortools.so* lives at
# install/<pkg>/lib/. INSTALL_RPATH_USE_LINK_PATH must be FALSE — without it
# CMake bakes the absolute source-dir path of libortools.so into RUNPATH,
# making the install tree dependent on the build tree.
set_target_properties(tare_planner_node navigationBoundary PROPERTIES
  INSTALL_RPATH "$ORIGIN/.."
  INSTALL_RPATH_USE_LINK_PATH FALSE)

#############
## Install ##
#############

install(TARGETS
  navigationBoundary
  tare_planner_node
  DESTINATION lib/${PROJECT_NAME})

# Ship the vendored libortools.so* alongside the executables.
# install(DIRECTORY) preserves the SONAME symlink chain
# (libortools.so -> libortools.so.9 -> libortools.so.9.8.3296).
install(DIRECTORY ${ORTOOLS_ROOT}/lib/
  DESTINATION lib
  USE_SOURCE_PERMISSIONS
  FILES_MATCHING
    PATTERN "libortools.so*"
    PATTERN "libortools_flatzinc.so*")

install(DIRECTORY launch/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY rviz/
        DESTINATION share/${PROJECT_NAME}/
        )
install(DIRECTORY config/
        DESTINATION share/${PROJECT_NAME}
      )
install(DIRECTORY data/
        DESTINATION share/${PROJECT_NAME}
      )

ament_package()
