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


