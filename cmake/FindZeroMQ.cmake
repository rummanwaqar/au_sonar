# - Find ZeroMQ includes and library
#
# This module defines
#  ZeroMQ_INCLUDE_DIRS
#  ZeroMQ_LIBRARIES, the libraries to link against to use ZeroMQ.
#  ZeroMQ_FOUND, If false, do not try to use MSGPACK
#
# Redistribution and use is allowed according to the terms of the BSD license.
# For details see the accompanying COPYING-CMAKE-SCRIPTS file.


find_package(PkgConfig)
## use pkg-config to get hints for 0mq locations
pkg_check_modules(PC_ZeroMQ QUIET zmq)

## use the hint from above to find where 'zmq.hpp' is located
find_path(ZeroMQ_INCLUDE_DIR
        NAMES zmq.hpp
        PATHS ${PC_ZeroMQ_INCLUDE_DIRS}
        )

## use the hint from about to find the location of libzmq
find_library(ZeroMQ_LIBRARY
        NAMES zmq
        PATHS ${PC_ZeroMQ_LIBRARY_DIRS}
        )

# Copy the results to the output variables.
IF (ZeroMQ_INCLUDE_DIR AND ZeroMQ_LIBRARY)
    SET(ZeroMQ_FOUND 1)
    SET(ZeroMQ_LIBRARIES ${ZeroMQ_LIBRARY})
    SET(ZeroMQ_INCLUDE_DIRS ${ZeroMQ_INCLUDE_DIR})

    MESSAGE(STATUS "Found these zeroMq libs: ${ZeroMQ_LIBRARIES}")

ELSE (ZeroMQ_INCLUDE_DIR AND ZeroMQ_LIBRARY)
    SET(ZeroMQ_FOUND 0)
    SET(ZeroMQ_LIBRARIES)
    SET(ZeroMQ_INCLUDE_DIRS)
ENDIF (ZeroMQ_INCLUDE_DIR AND ZeroMQ_LIBRARY)

# Report the results.
IF (NOT ZeroMQ_FOUND)
    SET(ZeroMQ_DIR_MESSAGE "ZeroMQ was not found. Make sure ZeroMQ_LIBRARY and ZeroMQ_INCLUDE_DIR are set.")
    MESSAGE(FATAL_ERROR "${ZeroMQ_DIR_MESSAGE}")
ENDIF (NOT ZeroMQ_FOUND)


MARK_AS_ADVANCED(
        ZeroMQ_INCLUDE_DIRS
        ZeroMQ_LIBRARIES
)
