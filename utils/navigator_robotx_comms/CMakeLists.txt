cmake_minimum_required(VERSION 2.8.3)
project(navigator_robotx_comms)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  geometry_msgs
  nav_msgs
  navigator_msgs
  mil_tools
)

catkin_python_setup()

catkin_package()