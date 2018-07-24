cmake_minimum_required(VERSION 3.3)
project(au_sonar)

# get system information
cmake_host_system_information(RESULT SYSTEM_HOSTNAME QUERY HOSTNAME)
message(STATUS "HOSTNAME: ${SYSTEM_HOSTNAME}")
set(BBB_HOSTNAME "beaglebone")

## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 11)

include_directories(include)

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

