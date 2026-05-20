cmake_minimum_required(VERSION 3.16)

project(lightning)

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 17)
endif ()

# set to release by default
set(CMAKE_BUILD_TYPE Release)

# -------------------------------------------------------------
# 导入第三方库，同时做一些全局编译选项配置
add_definitions("-DPCL_NO_PRECOMPILE")

# Base flags applied to every build type. CMake appends the per-config flags
# (CMAKE_CXX_FLAGS_<CONFIG>) on top of these, so the per-config strings must
# NOT re-embed ${CMAKE_CXX_FLAGS} — doing so passed every flag twice.
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fPIC -fopenmp -g")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC -g")

# Release: -O3 plus -DNDEBUG. -DNDEBUG is essential: without it Eigen keeps its
# internal assertions live and every assert() runs, including inside the LIO
# per-point hot loops. It was previously missing from this Release override.
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_DEBUG "-O0 -ggdb")

# Optional, numerically-aggressive math — deliberately OFF and kept isolated.
# -ffast-math relaxes IEEE-754 semantics and must never be bundled with the
# default flags; enable explicitly with -DLIGHTNING_FAST_MATH=ON.
option(LIGHTNING_FAST_MATH "Build with -ffast-math (relaxes IEEE FP; numerically aggressive)" OFF)
if (LIGHTNING_FAST_MATH)
    message(WARNING "LIGHTNING_FAST_MATH=ON: building with -ffast-math; floating-point results will differ.")
    add_compile_options(-ffast-math)
endif ()


if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/libs)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)
include(cmake/packages.cmake)

# msg 和 srv
set(MY_SRV_FILES "")
set(SRC_DIR ${PROJECT_SOURCE_DIR}/srv)
file(GLOB_RECURSE SRC_FILES ${SRC_DIR}/*.srv)
foreach (SRC_FILE ${SRC_FILES})
    file(RELATIVE_PATH REL_PATH ${SRC_DIR} ${SRC_FILE})
    list(APPEND MY_SRV_FILES ${PROJECT_SOURCE_DIR}:srv/${REL_PATH})
endforeach ()

rosidl_generate_interfaces(${PROJECT_NAME}
        ${MY_SRV_FILES}
        ${MY_MSG_FILES}
        ${MY_ACTION_FILES}
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs nav_msgs
)

include_directories(
        ${CMAKE_CURRENT_BINARY_DIR}/rosidl_generator_cpp
)

install(DIRECTORY config
        DESTINATION lib/${PROJECT_NAME}/
)

# Also install config + launch under share/ so ament_index_python's
# get_package_share_directory() can resolve them from launch files.
install(DIRECTORY config launch
        DESTINATION share/${PROJECT_NAME}/
)

# Python helper node: bridges typego_interface CustomMsg -> livox_ros_driver2
# CustomMsg so lightning-lm can consume the workspace's Livox driver output.
install(PROGRAMS scripts/typego_to_livox_bridge.py
        DESTINATION lib/${PROJECT_NAME}/
)

add_subdirectory(thirdparty/livox_ros_driver)

add_subdirectory(src)

ament_package()
