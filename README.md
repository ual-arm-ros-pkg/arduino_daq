arduino_daq
==================

This package contains an ARM firmware, a host standalone C++ library,
and a ROS node for UAL eCAR's Arduino-based DAQ system, with 4 analog
outputs (MAX5500), ADC inputs, GPIO, PWM and quadrature encoder
decoding support.

The used microcontroller is atmega328P, although it could be recompiled
for other larger versions with more I/O pins.

Compiling
=====================

In GNU/Linux or Windows, use CMake to generate the Makefiles or project files as usual.
Set CMake variables `BUILD_ROS` and `BUILD_STANDALONE_LIB` to ON/OFF depending on whether
you want to compile a ROS node or a standalone C++ library.

For compilation within a [ROS catkin](http://wiki.ros.org/catkin) environment, simple clone
this repository inside your `~/catkin_ws/src/` and run `catkin_make`.

ROS examples of use
=====================

Note: Numering of pins follows the `Arduino pin number` convention
(for example: Arduino ProMini,ATmega328, pin: PB1 <=> chip pin number: 13 <=> Arduino pin number: 9 ).

* **First**: Launch the node with

       roscore     # In one terminal
       rosrun arduino_daq arduino_daq_node \_SERIAL_PORT:=/dev/ttyUSB0    # In another terminal

Next, in another terminal try any of the following:

* Setting a DAC analog value (volts):

        rostopic pub /arduino_daq_dac0 std_msgs/Float64 1.5

* Setting a GPIO digital output pin:

        rostopic pub /arduino_daq_GPIO_output7 std_msgs/Bool 1

* Setting a PWM output (0...255 maps to 0%...100% duty cycle):

        rostopic pub /arduino_daq_pwm3 std_msgs/UInt8 220


ROS graph
=======================

<img width="300" src="https://raw.githubusercontent.com/ual-arm-ros-pkg/arduino-daq-ros-pkg/master/doc/rosgraph_arduino_daq.png" />


Source code directories
=======================
* `arduino_firmware`: Contains an AtmelStudio project for the firmware. Written in C++ using the Arduino library.
* `include` and `libarduinodaq`: Standalone C++ library.
* `src`: ROS node


Frame format
=====================

Communication PC <-> arduino happens based on data frames as follow:

         START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
           0x69          1 byte      1 byte       N bytes       =sum(data)       0x96

The complete list of OPCODES and associated payload `DATA` fields are defined in the C++ header
file [arduinodaq2pc-structs.h](https://github.com/ual-arm-ros-pkg/arduino-daq-ros-pkg/blob/master/arduino_firmware/arduino_daq/arduino_daq/arduinodaq2pc-structs.h),
which is `#include`d in both, host C++ libraries and the embedded firmware.
