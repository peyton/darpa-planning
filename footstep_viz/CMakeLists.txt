cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

set(CMAKE_CXX_COMPILER "ccache")

project(footstep_viz)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (footstep_viz footstep_visualization.cpp)
target_link_libraries (footstep_viz ${PCL_LIBRARIES})
