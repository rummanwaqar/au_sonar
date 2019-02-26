cmake_minimum_required(VERSION 2.8.3)
project(au_sonar)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    genmsg
    actionlib_msgs
    std_msgs
    geometry_msgs
    actionlib
    angles
    au_core
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
#generate_messages(
#    DEPENDENCIES
#    std_msgs
#    geometry_msgs
#)

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES au_sonar
#  CATKIN_DEPENDS roscpp rospy
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${catkin_INCLUDE_DIRS}
)
