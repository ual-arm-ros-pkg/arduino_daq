/* +---------------------------------------------------------------------------+
   |                  University of Almeria ARM-eCar module                    |
   |                                                                           |
   |   Copyright (C) 2014-16  University of Almeria                            |
   +---------------------------------------------------------------------------+ */

#include <arduino_daq/ArduinoDAQ_LowLevel.h>
#include <mrpt/system/threads.h> // for sleep()
#include <ros/console.h>
#include <arduino_daq/ArduinoDAQ_LowLevel.h>
#include <functional>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;


ArduinoDAQ_LowLevel::ArduinoDAQ_LowLevel() :
	m_nh_params("~"),
	m_serial_port_name("ttyUSB0"),
	m_serial_port_baudrate(1000000)
{

}

ArduinoDAQ_LowLevel::~ArduinoDAQ_LowLevel()
{
}

bool ArduinoDAQ_LowLevel::initialize()
{
	m_nh_params.getParam("SERIAL_PORT",m_serial_port_name);
	m_nh_params.getParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);

	// Try to connect...
	if (this->AttemptConnection())
	{
		ROS_INFO("Connection OK to ArduinoDAQ.");
	}

	//m_pub_contr_status = m_nh.advertise<steer_controller::SteerControllerStatus>("steer_controller_status", 10);

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
    ROS_INFO("GPIO: output[%i]=%s", pin, msg->data ? "true":"false" );

    if (!CMD_GPIO_output(pin,msg->data)) {
        ROS_ERROR("*** Error sending CMD_GPIO_output!!! ***");
    }
}

void ArduinoDAQ_LowLevel::daqSetDACCallback(int dac_index, const std_msgs::Float64::ConstPtr& msg)
{
    ROS_INFO("DAC: channel[%i]=%f V", dac_index, msg->data);

    if (!CMD_DAC(dac_index,msg->data)) {
        ROS_ERROR("*** Error sending CMD_DAC!!! ***");
    }
    
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


/** Sends a binary packet (returns false on COMMS error) */
bool ArduinoDAQ_LowLevel::WriteBinaryFrame(const uint8_t *full_frame, const size_t full_frame_len)
{
	if (!AttemptConnection()) return false;

	try
	{
		m_serial.WriteBuffer(full_frame,full_frame_len);
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
	while ( nFrameBytes < (lengthField=(    1        +     1   +  1    +  buf[2]  +     1     +     1      )  ) )
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


bool ArduinoDAQ_LowLevel::CMD_GPIO_output(int pin, bool pinState)
{
    TFrameCMD_GPIO_output cmd;
    cmd.payload.pin_index = pin;
    cmd.payload.pin_value = pinState ? 1:0;
    
    cmd.calc_and_update_checksum();
    
    return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

//!< Sets the clutch
bool ArduinoDAQ_LowLevel::CMD_DAC(int dac_index,double dac_value_volts)
{
    uint16_t dac_counts = 4096 * dac_value_volts / 5.0;
    mrpt::utils::saturate(dac_counts, uint16_t(0), uint16_t(4095));
    
    TFrameCMD_SetDAC cmd;
    cmd.payload.dac_index = dac_index;
    cmd.payload.dac_value_HI = dac_counts >> 8;
    cmd.payload.dac_value_LO = dac_counts & 0x00ff;
    
    cmd.calc_and_update_checksum();
    
    return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd),sizeof(cmd));
}

