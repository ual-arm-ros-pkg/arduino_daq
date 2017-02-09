/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#pragma once

#include <mrpt/hwdrivers/CSerialPort.h>

#ifdef HAVE_ROS
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>
#endif

#include <mrpt/utils/COutputLogger.h>
#include <arduinodaq2pc-structs.h>
#include <functional>


class ArduinoDAQ_LowLevel : public mrpt::utils::COutputLogger
{
public:
	ArduinoDAQ_LowLevel();
	virtual ~ArduinoDAQ_LowLevel();

#ifdef HAVE_ROS
	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_params;

	//ros::Publisher  m_pub_contr_status;
	std::vector<ros::Subscriber> m_sub_auto_pos, m_sub_dac;
#endif

	void setSerialPort(const std::string &sSerialName) {
		m_serial_port_name = sSerialName;
	}
	std::string getSerialPort() const {
		return m_serial_port_name;
	}

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

	bool CMD_GPIO_output(int pin, bool pinState);
	bool CMD_DAC(int dac_index, double dac_value_volts);
	bool CMD_ADC_START(const TFrameCMD_ADC_start_payload_t &adc_config);
	bool CMD_ADC_STOP();

	void set_ADC_readings_callback(const std::function<void(TFrame_ADC_readings_payload_t)> &f) {
		m_adc_callback = f;
	}

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

	std::function<void(TFrame_ADC_readings_payload_t)> m_adc_callback;

#ifdef HAVE_ROS
	void daqSetDigitalPinCallback(int index, const std_msgs::Bool::ConstPtr& msg);
	void daqSetDACCallback(int dac_index, const std_msgs::Float64::ConstPtr& msg);
#endif

};
