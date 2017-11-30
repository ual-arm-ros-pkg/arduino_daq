CMAKE_MINIMUM_REQUIRED(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)  # Require C++14
project(arduino_daq_app)

# System dependencies are found with CMake's conventions
find_package(MRPT REQUIRED hwdrivers)

include_directories("${arduino_daq_SOURCE_DIR}/include")
include_directories("${arduino_daq_SOURCE_DIR}/arduino_firmware/arduino_daq/arduino_daq")  # comms frames
include_directories("${MRPT_INCLUDE_DIRS}")

add_executable(${PROJECT_NAME} arduino_daq_main_standalone.cpp)

target_link_libraries(${PROJECT_NAME}
	${MRPT_LIBS}
	arduinodaq
	)