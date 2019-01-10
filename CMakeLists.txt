cmake_minimum_required(VERSION 3.5)
project(xdwa_local_planner)

# Default to C99
if(NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_options(-fPIC)
# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # remove the line when a copyright and license is present in all source files
    set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # remove the line when this package is a git repo
    set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif()

include_directories(
        include
)

add_library(dwa_local_planner
        src/xdwa_local_planner.cpp
        src/velocity_sample.cpp
        src/trajectory.cpp
        src/trajectory_generator.cpp
        src/trajectory_scorer.cpp
        src/costmap_score_function.cpp
        )

ament_target_dependencies(dwa_local_planner
        rclcpp
        geometry_msgs
        nav_msgs
        tf2
        tf2_ros
        tf2_geometry_msgs)

ament_package()