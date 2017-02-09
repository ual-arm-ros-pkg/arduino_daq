cmake_minimum_required(VERSION 3.1.0 FATAL_ERROR)
project(arduinodaq CXX)

set (CMAKE_CXX_STANDARD 11)

# System dependencies are found with CMake's conventions
find_package(MRPT REQUIRED hwdrivers)

include_directories(
  ${${PROJECT_NAME}_SOURCE_DIR}/../arduino_firmware/arduino_daq/arduino_daq/
  ${${PROJECT_NAME}_SOURCE_DIR}/../include
)

# Declare a C++ library
file(GLOB lib_srcs  ${${PROJECT_NAME}_SOURCE_DIR}/../src/arduino_daq/*.cpp include/arduino_daq/*.h)

if (UNIX)
	set (DEFLIBTYPE SHARED)
else()
	set(DEFLIBTYPE STATIC)
endif()

add_library(${PROJECT_NAME} ${DEFLIBTYPE}
   ${lib_srcs}
)

target_link_libraries(${PROJECT_NAME}
   ${MRPT_LIBS}
   )

