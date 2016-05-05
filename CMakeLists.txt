cmake_minimum_required(VERSION 2.8.3)
project(PAUVSI-M7)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
)

find_library(POINTGREY_LIB flycapture)
if(NOT POINTGREY_LIB)
  message(STATUS "libflycapture not found in system library path")
  include(cmake/DownloadFlyCap.cmake)
  download_flycap(POINTGREY_LIB POINTGREY_INCLUDE_DIR) 
  message(STATUS "libflycapture library: ${POINTGREY_LIB}")
  message(STATUS "libflycapture include: ${POINTGREY_INCLUDE_DIR}")
  include_directories(${POINTGREY_INCLUDE_DIR})  
endif()

add_executable(cameraDriver src/Utilities/PGCameraDriver.cpp)
target_link_libraries(cameraDriver ${POINTGREY_LIB} ${catkin_LIBRARIES})


