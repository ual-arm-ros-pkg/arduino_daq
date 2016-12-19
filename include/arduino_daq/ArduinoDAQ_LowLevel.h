/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <ros/ros.h>
#include <mrpt/hwdrivers/CSerialPort.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "arduinodaq2pc-structs.h"


class ArduinoDAQ_LowLevel
{
public:
	ArduinoDAQ_LowLevel();
	virtual ~ArduinoDAQ_LowLevel();

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_params;

	//ros::Publisher  m_pub_contr_status;
	std::vector<ros::Subscriber> m_sub_auto_pos, m_sub_dac;

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

protected:
	// Local class members:
	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	mrpt::hwdrivers::CSerialPort m_serial;  //!< The serial COMMS object

	// Local methods:
	bool AttemptConnection();   //!< Returns true if connected OK, false on error.
	bool IsConnected() const; 	//!< Returns true if the serial comms are working
	bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data); //!< Tries to get a framed chunk of data from the controller.
	bool WriteBinaryFrame( const uint8_t *full_frame, const size_t full_frame_len); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)

        bool CMD_GPIO_output(int pin, bool pinState);
        bool CMD_DAC(int dac_index,double dac_value_volts);

       
	void daqSetDigitalPinCallback(int index, const std_msgs::Bool::ConstPtr& msg);
	void daqSetDACCallback(int dac_index, const std_msgs::Float64::ConstPtr& msg);

};
