cmake_minimum_required(VERSION 3.5)
project(xdwa_local_planner)

include_directories(
        include
)

add_library(lib
        src/xdwa_local_planner.cpp
        src/velocity_sample.cpp
        src/trajectory.cpp
        src/trajectory_generator.cpp
        src/trajectory_scorer.cpp
        src/costmap_score_function.cpp
        )

add_executable(test src/trajectory_generator.cpp)
target_link_libraries(test lib)