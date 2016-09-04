cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_m7)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  sensor_msgs
  std_msgs
  dynamic_reconfigure
  cv_bridge
  image_transport
)

find_package(OpenCV REQUIRED)
include_directories(
	${OpenCV_INCLUDE_DIRS}
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

#IMPORTANT
catkin_package()
include_directories(
	${catkin_INCLUDE_DIRS}
)

add_executable(cameraDriver src/Utilities/PGCameraDriver.cpp)
target_link_libraries(cameraDriver ${POINTGREY_LIB} ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(cameraCalibrator src/ComputerVision/cameraCalibrator.cpp)
target_link_libraries(cameraCalibrator ${catkin_LIBRARIES} ${OpenCV_LIBRARIES})



