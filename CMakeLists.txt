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
)

catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################
add_action_files(
    DIRECTORY action
    FILES
    Sonar.action
)

add_service_files(
    FILES
    Triangulate.srv
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
    geometry_msgs
)

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

# Build the sonar_sim node
add_executable(sonar_sim src/sonar_sim_node.cpp)
add_dependencies(sonar_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sonar_sim ${catkin_LIBRARIES})