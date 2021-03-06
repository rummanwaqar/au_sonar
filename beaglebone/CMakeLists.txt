cmake_minimum_required(VERSION 3.5)
project(au_sonar_bbb)

## use this to globally use C++17 with in our project
add_compile_options(-std=c++17)

#########
# Flags #
#########

# debug option
option(DEBUG "Build with debug flags" 0)
if(DEBUG)
    message(WARNING "Building with debug flags")
    set(CMAKE_BUILD_TYPE Debug)
else()
    message(STATUS "Building without debug flags")
endif()

IF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7")
    set(CROSS_COMPILE_FLAGS -march=armv7-a -mtune=cortex-a8)
    message(STATUS "Building with ARM flags: ${CROSS_COMPILE_FLAGS}")
ELSE(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7")
    message(STATUS "Not building with ARM flags")
    set(CROSS_COMPILE_FLAGS "")
ENDIF(${CMAKE_SYSTEM_PROCESSOR} MATCHES "armv7")

#########################
# External Dependencies #
#########################

include(ExternalProject)
set(EXTERNAL_INSTALL_LOCATION ${CMAKE_BINARY_DIR}/external)
file(MAKE_DIRECTORY ${EXTERNAL_INSTALL_LOCATION}/bin)

# libzmq & cppzmq
ExternalProject_Add(libzmq
    GIT_REPOSITORY https://github.com/zeromq/libzmq
    GIT_TAG v4.3.1
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DBUILD_TESTS=OFF
    UPDATE_DISCONNECTED 1
)
ExternalProject_Add(cppzmq
    GIT_REPOSITORY https://github.com/zeromq/cppzmq
    GIT_TAG v4.3.0
    DEPENDS libzmq
    CMAKE_ARGS -DCMAKE_INSTALL_PREFIX=${EXTERNAL_INSTALL_LOCATION} -DCPPZMQ_BUILD_TESTS=OFF
    UPDATE_DISCONNECTED 1
)

# prussdrv
ExternalProject_Add(pruss_download
    GIT_REPOSITORY    https://github.com/beagleboard/am335x_pru_package
    PREFIX            ${CMAKE_CURRENT_BINARY_DIR}/pruss_download
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     ""
    INSTALL_COMMAND   ""
    UPDATE_DISCONNECTED 1
)
set(PRUSSDRV_DIR "${CMAKE_CURRENT_BINARY_DIR}/pruss_download/src/pruss_download/pru_sw/app_loader")
set(PRUSSDRV_LIBS "${PRUSSDRV_DIR}/lib/libprussdrv.a" "${PRUSSDRV_DIR}/lib/libprussdrv.so")
add_custom_command(OUTPUT ${PRUSSDRV_LIBS}
                   DEPENDS pruss_download
		   COMMAND make CROSS_COMPILE="" ARM_COMPILE_FLAGS="${CROSS_COMPILE_FLAGS}" release sorelease
                   WORKING_DIRECTORY ${PRUSSDRV_DIR}/interface
)
add_custom_target(prussdrv ALL DEPENDS ${PRUSSDRV_LIBS})
include_directories(${PRUSSDRV_DIR}/include)
link_directories(${PRUSSDRV_DIR}/lib)

# pasm
set(PASM_SOURCE "${CMAKE_CURRENT_BINARY_DIR}/pruss_download/src/pruss_download/pru_sw/utils/pasm_source")
set(PASM_TOOL "${EXTERNAL_INSTALL_LOCATION}/bin/pasm")
add_custom_command(OUTPUT ${PASM_TOOL}
                   DEPENDS pruss_download
                   COMMAND gcc -Wall -D_UNIX_ pasm.c pasmpp.c pasmexp.c pasmop.c pasmdot.c pasmstruct.c pasmmacro.c path_utils.c -o ${PASM_TOOL}
                   WORKING_DIRECTORY ${PASM_SOURCE}
)
add_custom_target(pasm ALL DEPENDS ${PASM_TOOL})

include_directories(${EXTERNAL_INSTALL_LOCATION}/include )
link_directories(${EXTERNAL_INSTALL_LOCATION}/lib)

# plog
ExternalProject_Add(plog
    GIT_REPOSITORY    https://github.com/SergiusTheBest/plog
    GIT_TAG           1.1.4
    PREFIX            ${CMAKE_CURRENT_BINARY_DIR}/plog
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     ""
    INSTALL_COMMAND   ""
    UPDATE_DISCONNECTED 1
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/plog/src/plog/include)

# json
ExternalProject_Add(json
    GIT_REPOSITORY    https://github.com/nlohmann/json
    GIT_TAG           v3.5.0
    PREFIX            ${CMAKE_CURRENT_BINARY_DIR}/json
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     ""
    INSTALL_COMMAND   ""
    UPDATE_DISCONNECTED 1
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/json/src/json/single_include/nlohmann)

# cxxopts
ExternalProject_Add(cxxopts
    GIT_REPOSITORY    https://github.com/jarro2783/cxxopts
    GIT_TAG           v2.1.2
    PREFIX            ${CMAKE_CURRENT_BINARY_DIR}/cxxopts
    CONFIGURE_COMMAND ""
    BUILD_COMMAND     ""
    INSTALL_COMMAND   ""
    UPDATE_DISCONNECTED 1
)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/cxxopts/src/cxxopts/include)

# boost
find_package(Boost COMPONENTS system REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})

#####################
# Build Custom Code #
#####################
include_directories(include)

# build pru_reader
add_library(pru_reader src/pru_reader.cpp)
add_dependencies(pru_reader prussdrv plog json)
target_link_libraries(pru_reader prussdrv.so pthread)

# build preprocessor
if(Boost_FOUND)
  add_library(preprocessor src/serial.cpp src/preprocessor.cpp)
  add_dependencies(preprocessor plog json)
  target_link_libraries(preprocessor ${Boost_LIBRARIES})
endif()

# build main executable
add_executable(sonar_daq src/main.cpp)
add_dependencies(sonar_daq preprocessor pru_reader cppzmq plog json cxxopts)
target_link_libraries(sonar_daq libzmq.so preprocessor pru_reader pthread)

# build pru firmware
set(PRU_WORKING_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/pru_firmware")
set(PRU_FIRMWARE_OUTPUTS
    "${PRU_WORKING_DIRECTORY}/pru0-clock.bin"
    "${PRU_WORKING_DIRECTORY}/pru1-read-data.bin"
)
add_custom_command(
        OUTPUT ${PRU_FIRMWARE_OUTPUTS}
        COMMAND make all PASM=${PASM_TOOL} BUILD_DIR=${CMAKE_CURRENT_BINARY_DIR}
        WORKING_DIRECTORY "${PRU_WORKING_DIRECTORY}"
        DEPENDS pasm "${PRU_WORKING_DIRECTORY}/pru0-clock.p" "${PRU_WORKING_DIRECTORY}/pru1-read-data.p"
        COMMENT "Building PRU firmware"
        VERBATIM
)
add_custom_target(pru_firmware ALL DEPENDS ${PRU_FIRMWARE_OUTPUTS})

# build test
option(TESTS "Build unit tests" 0)
if(TESTS)
    message(STATUS "Building tests.")
    add_subdirectory(tests)
else()
    message(STATUS "Not building tests.")
endif()
