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

class CSteerControllerLowLevel
{
public:
	CSteerControllerLowLevel();
	virtual ~CSteerControllerLowLevel();

	/**
	* NodeHandle is the main access point to communications with the ROS system.
	* The first NodeHandle constructed will fully initialize this node, and the last
	* NodeHandle destructed will close down the node.
	*/
	ros::NodeHandle m_nh;
	ros::NodeHandle m_nh_params;

	ros::Publisher  m_pub_contr_status;
	ros::Subscriber m_sub_auto_pos, m_sub_pwm;

	/** called at startup, load params from ROS launch file and attempts to connect to the USB device
	  * \return false on error */
	bool initialize();

	/** called when work is to be done */
	bool iterate();

protected:
	// Local class members:
	std::string m_serial_port_name;
	int         m_serial_port_baudrate;
	double      m_steer_report_freq;
	mrpt::hwdrivers::CSerialPort m_serial;  //!< The serial COMMS object

	// Local methods:
	bool AttemptConnection();   //!< Returns true if connected OK, false on error.
	bool IsConnected() const; 	//!< Returns true if the serial comms are working
	bool ReceiveFrameFromController(std::vector<uint8_t> &rx_data); //!< Tries to get a framed chunk of data from the controller.
	bool WriteBinaryFrame( const uint8_t *data, uint16_t len); //!< Sends a binary packet, in the expected format  (returns false on COMMS error)

	bool CMD_EnableAutoControl(bool enable); //!< Sets the automatic/manual control in the steering controller board. Return false on COMMS error.
	bool CMD_SetClutch(bool enable); //!< Sets the Clutch relay.
	bool CMD_SetReportFreq(double freq); //!< Sets the report frequency
	bool CMD_SetPWMValue(double pwm_duty_cycle); //!< Sets the PWM value (-1.0 to 1.0)
	bool CMD_SetPosControlSetPoint(int pos_ticks); //!< Sets the position setpoint


	/** Convert encoder ticks into Ackermann central angle (radians) */
	double ackermannAngle(int32_t ticks) const;

	/** Convert Ackermann central angle (radians) to encoder ticks: */
	int32_t ackermannAngle_inverse(double angle_radians) const;

	void autoPosEnableCallback(const std_msgs::Bool::ConstPtr& msg);
	void pwmCallback(const std_msgs::Float64::ConstPtr& msg);

};
