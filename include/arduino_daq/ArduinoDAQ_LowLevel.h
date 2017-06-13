/*
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016-17, Universidad de Almeria
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */


#pragma once

#include <mrpt/hwdrivers/CSerialPort.h>

#ifdef HAVE_ROS
#include <ros/ros.h>
#include <std_msgs/UInt8.h>
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

	std::vector<ros::Subscriber> m_sub_GPIO_outputs, m_sub_dac, m_sub_PWM_outputs, m_sub_PWM_steer_controller;
	ros::Publisher  m_pub_ADC, m_pub_ENC;
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
	bool CMD_PWMSteering(int 3, bool pinState); /* Error después del 3 ¿Por qué?: Expected ',' or '...' before numeric constant*/
												/* Note: Candidate: bool ArduinoDAQ_LowLevel:: CMD_PWMSteering(int)*/
												/* Note: candidate expects 1 argument, 2 provided*/
												/* Error: Candidate is: bool ArduinoDAQ_LowLevel:: CMD_PWMSteering(int)*/
	bool CMD_ADC_START(const TFrameCMD_ADC_start_payload_t &enc_config);
	bool CMD_ADC_STOP();
	bool CMD_PWM(int pin_index, uint8_t pwm_value);
	bool CMD_ENCODERS_START(const TFrameCMD_ENCODERS_start_payload_t &enc_config);
	bool CMD_ENCODERS_STOP();

	void set_ADC_readings_callback(const std::function<void(TFrame_ADC_readings_payload_t)> &f) {
		m_adc_callback = f;
	}

	void set_ENCODERS_readings_callback(const std::function<void(TFrame_ENCODERS_readings_payload_t)> &f) {
		m_enc_callback = f;
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
	std::function<void(TFrame_ENCODERS_readings_payload_t)> m_enc_callback;

#ifdef HAVE_ROS
	void daqSetDigitalPinCallback(int index, const std_msgs::Bool::ConstPtr& msg);
	void daqSetDACCallback(int dac_index, const std_msgs::Float64::ConstPtr& msg);
	void daqSetPWMSteeringCallback(int pwm_index, const std_msgs::Float64::ConstPtr& msg);
	void daqSetPWMCallback(int pwm_pin_index, const std_msgs::UInt8::ConstPtr& msg);
	void daqOnNewADCCallback(const TFrame_ADC_readings_payload_t &data);
	void daqOnNewENCCallback(const TFrame_ENCODERS_readings_payload_t &data);
#endif

};
