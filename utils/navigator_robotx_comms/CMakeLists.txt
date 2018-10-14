cmake_minimum_required(VERSION 2.8.3)
project(navigator_robotx_comms)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  rospy
  std_msgs
  geometry_msgs
  mil_tools
  nav_msgs
  navigator_msgs
)
