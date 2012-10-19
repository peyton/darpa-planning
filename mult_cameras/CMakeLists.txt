cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(mult_cameras)

find_package(PCL 1.3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR} CACHE STATIC "Module path")
find_package(Nestk REQUIRED)
INCLUDE(${NESTK_USE_FILE})

# Default values
SET(NESTK_USE_FREENECT 1)
SET(NESTK_USE_OPENNI 1)

find_package(Qt4 REQUIRED)
include(${QT_USE_FILE})
include_directories(${QT_INCLUDES})


add_executable (mult_cameras synchronized_multiple_kinects.cpp)
target_link_libraries (mult_cameras ${PCL_LIBRARIES} ${QT_LIBRARIES} ${NESTK_LIBRARIES})
