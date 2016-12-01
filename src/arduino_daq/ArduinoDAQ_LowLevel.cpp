/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <arduino_daq/ArduinoDAQ_LowLevel.h>
#include <mrpt/system/threads.h> // for sleep()
#include <ros/console.h>
#include <steer_controller/SteerControllerStatus.h>
#include <functional>

#include "arduinodaq2pc-structs.h"

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;


ArduinoDAQ_LowLevel::ArduinoDAQ_LowLevel() :
	m_nh_params("~"),
	m_serial_port_name("ttyUSB0"),
	m_serial_port_baudrate(1000000),
	m_steer_report_freq(30)
{

}

ArduinoDAQ_LowLevel::~ArduinoDAQ_LowLevel()
{
}

bool ArduinoDAQ_LowLevel::initialize()
{
	m_nh_params.getParam("SERIAL_PORT",m_serial_port_name);
	m_nh_params.getParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);
	m_nh_params.getParam("STEERPOS_STATUS_FREQ",m_steer_report_freq);

	// Try to connect...
	if (this->AttemptConnection())
	{
		ROS_INFO("Connection OK to steering controller.");

		CMD_SetReportFreq(m_steer_report_freq);
		CMD_SetPWMValue(0);

		// TODO: Send control parameters to controller?
	}

	m_pub_contr_status = m_nh.advertise<steer_controller::SteerControllerStatus>("steer_controller_status", 10);

	// Subscribers: GPIO outputs
	m_sub_auto_pos.resize(13);
	for (int i=0;i<13;i++) {
		auto fn = std::bind(&ArduinoDAQ_LowLevel::daqSetDigitalPinCallback, this, i, _1);
		m_sub_auto_pos[i] = m_nh.subscribe<std_msgs::Bool>( mrpt::format("arduino_daq_GPIO_output%i",i), 10, fn);
	}

	// Subscribers: DAC outputs
	m_sub_dac.resize(4);
	for (int i=0;i<4;i++) {
		auto fn = std::bind(&ArduinoDAQ_LowLevel::daqSetDACCallback, this, i, _1);
		m_sub_auto_pos[i] = m_nh.subscribe<std_msgs::Float64>( mrpt::format("arduino_daq_dac%i",i), 10, fn);
	}
}

bool ArduinoDAQ_LowLevel::iterate()
{
	// Main module loop code.
	const size_t MAX_FRAMES_PER_ITERATE = 20;
	size_t nFrames = 0;

	if (!m_serial.isOpen())
		return false;

	std::vector<uint8_t> rxFrame;
	while (++nFrames<MAX_FRAMES_PER_ITERATE && ReceiveFrameFromController(rxFrame))
	{
		// Process them:
		//ROS_INFO_STREAM << "Rx frame, len=" << rxFrame.size();
	}

	return true;
}

void ArduinoDAQ_LowLevel::daqSetDigitalPinCallback(int pin, const std_msgs::Bool::ConstPtr& msg)
{
	ROS_INFO("Setting autocontrol mode: %s", msg->data ? "true":"false" );
/*
 * 	if (!CMD_EnableAutoControl(msg->data)) {
		ROS_ERROR("*** Error sending control auto mode!!! ***");
	}
*/
}

void ArduinoDAQ_LowLevel::daqSetDACCallback(int dac_index, const std_msgs::Float64::ConstPtr& msg)
{

}

bool ArduinoDAQ_LowLevel::AttemptConnection()
{
	if (m_serial.isOpen()) return true; // Already open.

	try {
		m_serial.open(m_serial_port_name);

		// Set basic params:
		m_serial.setConfig(m_serial_port_baudrate);
		m_serial.setTimeouts(100,0,10,0,50);

		return true;
	}
	catch (exception &e)
	{
		ROS_ERROR("[ArduinoDAQ_LowLevel::AttemptConnection] COMMS error: %s", e.what() );
		return false;
	}
}


/** Sends a binary packet, in the expected format  (returns false on COMMS error) */
bool ArduinoDAQ_LowLevel::WriteBinaryFrame( const uint8_t *data, uint16_t len)
{
	if (!AttemptConnection()) return false;

	std::vector<uint8_t> buf;

	buf.resize(len+1+2+1);
	buf[0] = STEERCONTROL_COMMS_FRAME_START_FLAG;
	buf[1] = len & 0xFF;
	buf[2] = (len>>8) & 0xFF;
	memcpy( &buf[3], data, len);
	buf[len+3] = STEERCONTROL_COMMS_FRAME_END_FLAG;

	try
	{
		m_serial.WriteBuffer(&buf[0],buf.size());
		return true;
	}
	catch (std::exception &)
	{
		return false;
	}
}

bool ArduinoDAQ_LowLevel::ReceiveFrameFromController(std::vector<uint8_t> &rxFrame)
{
	rxFrame.clear();
	size_t	nFrameBytes = 0;
	std::vector<uint8_t> buf;
	buf.resize(0x10000);
	buf[0] = buf[1] = 0;

	size_t	lengthField;

	/*
	START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
	  0x69          1 byte      1 byte       N bytes       =sum(data)       0x96
	*/

	//                                   START_FLAG     OPCODE + LEN       DATA      CHECKSUM +  END_FLAG
	while ( nFrameBytes < (lengthField=(    1        +     1   +  1    +  buf[2]  +     1     +     1      )  )
	{
		if (lengthField>200)
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[1]=buf[2]=0;
			cout << "[rx] Reset frame (invalid len)\n";
		}

		size_t nBytesToRead;
		if (nFrameBytes<3)
			nBytesToRead = 1;
		else
			nBytesToRead = (lengthField) - nFrameBytes;

		size_t 	nRead;
		try
		{
			nRead = m_serial.Read( &buf[0] + nFrameBytes, nBytesToRead );
		}
		catch (std::exception &e)
		{
			// Disconnected?
			cerr << "[ArduinoDAQ_LowLevel::ReceiveFrameFromController] Comms error: " << e.what() << endl;
			return false;
		}

		if ( !nRead && !nFrameBytes )
		{
			//cout << "[rx] No frame (buffer empty)\n";
			return false;
		}

		if (nRead<nBytesToRead)
			mrpt::system::sleep(1);

		// Lectura OK:
		// Check start flag:
		bool is_ok = true;

		if (!nFrameBytes && buf[0]!= FRAME_START_FLAG )
		{
			is_ok = false;
			//cout << "[rx] Reset frame (start flag)\n";
		}

		if (nFrameBytes>2 && nFrameBytes+nRead==lengthField)
		{
			if (buf[nFrameBytes+nRead-1]!=FRAME_END_FLAG)
			{
				is_ok= false;
				//cout << "[rx] Reset frame (end flag)\n";
			}
			//else { cout << "[rx] Frame OK\n"; }
		}

		MRPT_TODO("Checksum");

		if (is_ok)
		{
			nFrameBytes+=nRead;
		}
		else
		{
			nFrameBytes = 0;	// No es cabecera de trama correcta
			buf[1]=buf[2]=0;
		}
	}

	// Frame received
	lengthField= buf[2]+5;
	rxFrame.resize(lengthField);
	memcpy( &rxFrame[0], &buf[0], lengthField);


	// All OK
	return true;
}


//!< Sets the automatic/manual control in the steering controller board. Return false on COMMS error
bool ArduinoDAQ_LowLevel::CMD_EnableAutoControl(bool enable)
{
	TCmdSetAutoMode cmd;
	cmd.enable_pos_control = enable ? 1:0;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the clutch
bool ArduinoDAQ_LowLevel::CMD_SetClutch(bool enable)
{
	TCmdSetClutch cmd;
	cmd.relay_state = enable ? 1:0;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the report frequency
bool ArduinoDAQ_LowLevel::CMD_SetReportFreq(double freq)
{
	const TFirmwareParams params;
	TCmdSetReportDecimation cmd;
	double dec = (freq>0) ? (1.0/(params.READ_ENCODERS_PERIOD*1e-4*freq)) : 10;
	cmd.report_decimation = dec;

	ROS_INFO("[ArduinoDAQ_LowLevel] Setting report freq to %.02f Hz (dec=%f)\n", freq,dec);

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the PWM value (-1023,1023)
bool ArduinoDAQ_LowLevel::CMD_SetPWMValue(double pwm_duty_cycle)
{
	TCmdSetPWMValue cmd;
	cmd.pwm_value = pwm_duty_cycle*1023;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the position setpoint
bool ArduinoDAQ_LowLevel::CMD_SetPosControlSetPoint(int pos_ticks)
{
	TCmdSetPosControlSetPoint cmd;
	cmd.setpoint_ticks = pos_ticks;
	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}
