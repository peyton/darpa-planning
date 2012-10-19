cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcl_path_online_cb)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pcl_path_online_cb footstep_visualizer.cpp demo.c pcl_path_online_cb.cpp)
target_link_libraries (pcl_path_online_cb ${PCL_LIBRARIES})
