cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(kinect_data_show)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (kinect_data_show demo.c kinect_data_show.cpp)
target_link_libraries (kinect_data_show ${PCL_LIBRARIES})
