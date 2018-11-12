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
    actionlib
    angles
)

# get system information
cmake_host_system_information(RESULT SYSTEM_HOSTNAME QUERY HOSTNAME)
message(STATUS "HOSTNAME: ${SYSTEM_HOSTNAME}")
set(BBB_HOSTNAME "beaglebone")

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################


add_action_files(
    DIRECTORY action
    FILES
    Sonar.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
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
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/au_sonar.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
# add_executable(${PROJECT_NAME}_node src/au_sonar_node.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(${PROJECT_NAME}_node
#   ${catkin_LIBRARIES}
# )

# Build the sonar_sim node
add_executable(sonar_sim src/sonar_sim_node.cpp)
add_dependencies(sonar_sim ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sonar_sim ${catkin_LIBRARIES})

###############################
# Beaglebone specific targets #
###############################

if(${SYSTEM_HOSTNAME} STREQUAL ${BBB_HOSTNAME})
    message(STATUS "Building Beaglebone specific targets")

    include(cmake/FindZeroMQ.cmake)
    include(cmake/FindMsgPack.cmake)

    # pru_read executable
    add_executable(pru_read src/pru_read.cpp)
    target_link_libraries(pru_read prussdrv ${ZeroMQ_LIBRARIES} ${MSGPACK_LIBRARIES})

    # build PRU firmware
    set(PRU_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pru_firmware")
    set(PRU_FIRMWARE_OUTPUTS
            "${PRU_WORKING_DIRECTORY}/pru0-clock.bin"
            "${PRU_WORKING_DIRECTORY}/pru1-read-data.bin"
            )
    add_custom_command(
            OUTPUT ${PRU_FIRMWARE_OUTPUTS}
            COMMAND make all
            WORKING_DIRECTORY "${PRU_WORKING_DIRECTORY}"
            DEPENDS "${PRU_WORKING_DIRECTORY}/pru0-clock.p" "${PRU_WORKING_DIRECTORY}/pru1-read-data.p"
            COMMENT "Building PRU firmware"
            VERBATIM
    )
    add_custom_target(pru_firmware ALL DEPENDS ${PRU_FIRMWARE_OUTPUTS})
else()
    message(WARNING "Not building Beaglebone specific targets")
endif(${SYSTEM_HOSTNAME} STREQUAL ${BBB_HOSTNAME})