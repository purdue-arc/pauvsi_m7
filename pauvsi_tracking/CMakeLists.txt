cmake_minimum_required(VERSION 2.8.3)
project(pauvsi_tracking)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  cv_bridge
  image_transport
  roscpp
  cmake_modules
  tf
)

find_package(Eigen3 REQUIRED)

find_package(OpenCV REQUIRED)


## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES pauvsi_tracking
#  CATKIN_DEPENDS cv_bridge image_transport roscpp tf
  DEPENDS eigen system_lib opencv
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_library(tracker include/pauvsi_tracking/tracker.cpp)

add_executable(roomba_tracker src/pauvsi_tracking.cpp)

target_link_libraries(tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(roomba_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES}  tracker)

add_library(lidar_tracking include/pauvsi_tracking/rplidar.cpp)
add_executable(obstacle_tracker src/obstacle_tracking.cpp)

target_link_libraries(lidar_tracking ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(obstacle_tracker ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES} lidar_tracking)

add_library(controller include/pauvsi_tracking/controller.cpp)

target_link_libraries(controller ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

add_library(kalmanFilter include/pauvsi_tracking/kalman_filter.cpp)

target_link_libraries(kalmanFilter ${catkin_LIBRARIES} ${OpenCV_LIBRARIES} ${Eigen_LIBRARIES})

## Declare a C++ library
# add_library(pauvsi_tracking
#   src/${PROJECT_NAME}/pauvsi_tracking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(pauvsi_tracking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(pauvsi_tracking_node src/pauvsi_tracking_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(pauvsi_tracking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(pauvsi_tracking_node
#   ${catkin_LIBRARIES}
# )



## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_pauvsi_tracking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
