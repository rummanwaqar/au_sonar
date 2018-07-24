cmake_minimum_required(VERSION 3.3)
project(au_sonar)

## use this to globally use C++11 with in our project
set(CMAKE_CXX_STANDARD 11)

include_directories(include)

# pru_read executable
add_executable(pru_read src/pru_read.cpp)
target_link_libraries(pru_read prussdrv)