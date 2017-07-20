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


#include <arduino_daq/ArduinoDAQ_LowLevel.h>
#include <arduino_daq/ArduinoDAQ_LowLevel.h>
#include <arduino_daq/AnalogReading.h>
#include <arduino_daq/EncodersReading.h>
#include <functional>
#include <cstring>
#include <array>
#include <thread>
#include <chrono>

#ifdef HAVE_ROS
#include <ros/console.h>
#endif

#include <iostream>

using namespace std;
using namespace mrpt;
using namespace mrpt::utils;

#define DEBUG_TRACES

#ifdef HAVE_ROS
void log_callback(const std::string &msg, const mrpt::utils::VerbosityLevel level, const std::string &loggerName, const mrpt::system::TTimeStamp timestamp, void *userParam)
{
	ROS_INFO("%s",msg.c_str());
}
#endif

ArduinoDAQ_LowLevel::ArduinoDAQ_LowLevel() :
	mrpt::utils::COutputLogger("ArduinoDAQ_LowLevel"),
#ifdef HAVE_ROS
	m_nh_params("~"),
#endif
#ifndef _WIN32
	m_serial_port_name("/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"),
#else
	m_serial_port_name("COM3"),
#endif
	m_serial_port_baudrate(115200)
{
#ifdef HAVE_ROS
	this->logRegisterCallback(&log_callback, this);
#endif
}

ArduinoDAQ_LowLevel::~ArduinoDAQ_LowLevel()
{
}

bool ArduinoDAQ_LowLevel::initialize()
{
#ifdef HAVE_ROS
	m_nh_params.getParam("SERIAL_PORT",m_serial_port_name);
	m_nh_params.getParam("SERIAL_PORT_BAUDRATE",m_serial_port_baudrate);
#endif

	// Try to connect...
	if (this->AttemptConnection())
	{
		MRPT_LOG_INFO("Connection OK to ArduinoDAQ.");
	}
	else
	{
		MRPT_LOG_ERROR("Error in ArduinoDAQ_LowLevel::AttemptConnection()!");
		return false;
	}

#ifdef HAVE_ROS
	// Subscribers: GPIO outputs
	m_sub_GPIO_outputs.resize(13);
	for (int i=0;i<13;i++) {
		auto fn = boost::bind(&ArduinoDAQ_LowLevel::daqSetDigitalPinCallback, this, i, _1);
		m_sub_GPIO_outputs[i] = m_nh.subscribe<std_msgs::Bool>( mrpt::format("arduino_daq_GPIO_output%i",i), 10, fn);
	}

	// Subscribers: DAC outputs
	m_sub_dac.resize(4);
	for (int i=0;i<4;i++) {
		auto fn = boost::bind(&ArduinoDAQ_LowLevel::daqSetDACCallback, this, i, _1);
		m_sub_dac[i] = m_nh.subscribe<std_msgs::Float64>( mrpt::format("arduino_daq_dac%i",i), 10, fn);
	}


	// Subscribers: PWM outputs
	// (From: https://www.arduino.cc/en/Reference/analogWrite)
	// On most Arduino boards (those with the ATmega168 or ATmega328), this function works on pins 3, 5, 6, 9, 10, and 11.
	const int PWM_pins[] = {3, 5, 6, 9, 10, 11};
	const int num_PWM_pins = sizeof(PWM_pins)/sizeof(PWM_pins[0]);

	m_sub_PWM_outputs.resize(num_PWM_pins);
	for (int i=0;i<num_PWM_pins;i++) {
		int pin = PWM_pins[i];
		auto fn = boost::bind(&ArduinoDAQ_LowLevel::daqSetPWMCallback, this, pin, _1);
		m_sub_PWM_outputs[i] = m_nh.subscribe<std_msgs::UInt8>( mrpt::format("arduino_daq_pwm%i",pin), 10, fn);
	}

	// Publisher: ADC data
	m_pub_ADC = m_nh.advertise<arduino_daq::AnalogReading>("arduino_daq_adc", 10);

	// Publisher: ENC data
	m_pub_ENC = m_nh.advertise<arduino_daq::EncodersReading>("arduino_daq_encoders", 10);

	// Only for ROS:
	// If provided via params, automatically start ADC conversion:
	{
		int ADC_INTERNAL_REFVOLT = 0;
		m_nh_params.getParam("ADC_INTERNAL_REFVOLT",ADC_INTERNAL_REFVOLT);

		int ADC_MEASURE_PERIOD_MS = 100;
		m_nh_params.getParam("ADC_MEASURE_PERIOD_MS",ADC_MEASURE_PERIOD_MS);

		TFrameCMD_ADC_start_payload_t cmd;
		bool any_active = false;
		for (int i=0;i<8;i++)
		{
			int ch = -1;
			m_nh_params.getParam(mrpt::format("ADC_CHANNEL%i",i),ch);
			cmd.active_channels[i] = ch;
			if (ch!=-1) any_active=true;
		}

		if (any_active)
		{
			cmd.use_internal_refvolt = ADC_INTERNAL_REFVOLT ? 1:0;
			cmd.measure_period_ms = ADC_MEASURE_PERIOD_MS;

			ROS_INFO("Starting continuous ADC readings with: "
				"int_ref_volt=%i "
				"measure_period_ms=%i ms"
				"channels: %i %i %i %i %i %i %i %i"
				,
				cmd.use_internal_refvolt,
				cmd.measure_period_ms,
				cmd.active_channels[0],cmd.active_channels[1],cmd.active_channels[2],cmd.active_channels[3],
				cmd.active_channels[4],cmd.active_channels[5],cmd.active_channels[6],cmd.active_channels[7]
			);
			this->CMD_ADC_START(cmd);
		}
	}

	// If provided via params, automatically start ENCODERS decoding:
	{
		std::array<int,TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS> ENC_PIN_A,ENC_PIN_B,ENC_PIN_Z;
		ENC_PIN_A.fill(0);
		ENC_PIN_B.fill(0);
		ENC_PIN_Z.fill(0);

		bool any_active = false;
		for (int i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++)
		{
			m_nh_params.getParam(mrpt::format("ENC%i_PIN_A",i),ENC_PIN_A[i]);
			m_nh_params.getParam(mrpt::format("ENC%i_PIN_B",i),ENC_PIN_B[i]);
			m_nh_params.getParam(mrpt::format("ENC%i_PIN_Z",i),ENC_PIN_Z[i]);
			if (ENC_PIN_A[i]!=0) {
				any_active = true;
			}
		}

		int ENC_MEASURE_PERIOD_MS = 100;
		m_nh_params.getParam("ENC_MEASURE_PERIOD_MS",ENC_MEASURE_PERIOD_MS);

		TFrameCMD_ENCODERS_start_payload_t cmd;
		for (int i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++)
		{
			cmd.encA_pin[i] = ENC_PIN_A[i];
			cmd.encB_pin[i] = ENC_PIN_B[i];
			cmd.encZ_pin[i] = ENC_PIN_Z[i];
		}
		if (any_active)
		{
			cmd.sampling_period_ms = ENC_MEASURE_PERIOD_MS;

			ROS_INFO("Starting ENCODERS readings with: ");
			for (int i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++) {
				ROS_INFO(" ENC%i: A_pin=%i  B_pin=%i  Z_pin=%i",i,ENC_PIN_A[i],ENC_PIN_B[i],ENC_PIN_Z[i]);
			}
			this->CMD_ENCODERS_START(cmd);
		}
	}

#endif

	return true;
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
		//MRPT_LOG_INFO_STREAM  << "Rx frame, len=" << rxFrame.size();
		if (rxFrame.size() >= 5)
		{
			switch (rxFrame[1])
			{
				case RESP_ADC_READINGS:
				{
					TFrame_ADC_readings rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));

					if (m_adc_callback) {
						m_adc_callback(rx.payload);
					}
					daqOnNewADCCallback(rx.payload);
				}
				break;

				case RESP_ENCODER_READINGS:
				{
					TFrame_ENCODERS_readings rx;
					::memcpy((uint8_t*)&rx, &rxFrame[0], sizeof(rx));

					if (m_enc_callback) {
						m_enc_callback(rx.payload);
					}
					daqOnNewENCCallback(rx.payload);
				}
				break;
			};
		}
	}

	return true;
}

#ifdef HAVE_ROS
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

void ArduinoDAQ_LowLevel::daqSetPWMCallback(int pwm_pin_index, const std_msgs::UInt8::ConstPtr& msg)
{
	ROS_INFO("PWM: pin%i=%i ", pwm_pin_index, (int)msg->data);

    if (!CMD_PWM(pwm_pin_index,msg->data)) {
        ROS_ERROR("*** Error sending CMD_PWM!!! ***");
    }
}

void ArduinoDAQ_LowLevel::daqOnNewADCCallback(const TFrame_ADC_readings_payload_t &data)
{
	arduino_daq::AnalogReading msg;

	msg.timestamp_ms = data.timestamp_ms;
	for (int i=0;i<sizeof(data.adc_data)/sizeof(data.adc_data[0]);i++) {
		 msg.adc_data[i] = data.adc_data[i];
	}

	m_pub_ADC.publish(msg);
}

void ArduinoDAQ_LowLevel::daqOnNewENCCallback(const TFrame_ENCODERS_readings_payload_t &data)
{
	arduino_daq::EncodersReading msg;

	msg.timestamp_ms = data.timestamp_ms;
	msg.period_ms = data.period_ms;
	const int N =sizeof(data.encoders)/sizeof(data.encoders[0]);

	msg.encoder_values.resize(N);
	for (int i=0;i<N;i++) {
		 msg.encoder_values[i] = data.encoders[i];
	}

	m_pub_ENC.publish(msg);
}

#endif

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
	catch (std::exception &e)
	{
		MRPT_LOG_ERROR_FMT("[ArduinoDAQ_LowLevel::AttemptConnection] COMMS error: %s", e.what() );
		return false;
	}
}


/** Sends a binary packet (returns false on COMMS error) */
bool ArduinoDAQ_LowLevel::WriteBinaryFrame(const uint8_t *full_frame, const size_t full_frame_len)
{
	if (!AttemptConnection()) return false;

	ASSERT_(full_frame!=NULL);

	try
	{
#ifdef DEBUG_TRACES
		{
			std::string s;
			s+=mrpt::format("TX frame (%u bytes): ", (unsigned int) full_frame_len);
			for (size_t i=0;i< full_frame_len;i++)
				s+=mrpt::format("%02X ", full_frame[i]);
			ROS_INFO("Tx frame: %s", s.c_str());
		}
#endif

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
			MRPT_LOG_INFO("[rx] Reset frame (invalid len)");
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
			std::cerr << "[ArduinoDAQ_LowLevel::ReceiveFrameFromController] Comms error: " << e.what() << std::endl;
			return false;
		}

		if ( !nRead && !nFrameBytes )
		{
			//cout << "[rx] No frame (buffer empty)\n";
			return false;
		}

		if (nRead<nBytesToRead)
		{
			using namespace std::chrono_literals;
			std::this_thread::sleep_for(1ms);
		}

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
	::memcpy( &rxFrame[0], &buf[0], lengthField);

#ifdef DEBUG_TRACES
		{
			std::string s;
			s+=mrpt::format("RX frame (%u bytes): ", (unsigned int) lengthField);
			for (size_t i=0;i< lengthField;i++)
				s+=mrpt::format("%02X ", rxFrame[i]);
			ROS_INFO("%s", s.c_str());
		}
#endif

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

bool ArduinoDAQ_LowLevel::IsConnected() const
{
	return m_serial.isOpen();
}

bool ArduinoDAQ_LowLevel::CMD_ADC_START(const TFrameCMD_ADC_start_payload_t &adc_config)
{
	TFrameCMD_ADC_start cmd;
	cmd.payload = adc_config;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}
bool ArduinoDAQ_LowLevel::CMD_ADC_STOP()
{
	TFrameCMD_ADC_stop cmd;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}

bool ArduinoDAQ_LowLevel::CMD_ENCODERS_START(const TFrameCMD_ENCODERS_start_payload_t &enc_config)
{
	TFrameCMD_ENCODERS_start cmd;
	cmd.payload = enc_config;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}
bool ArduinoDAQ_LowLevel::CMD_ENCODERS_STOP()
{
	TFrameCMD_ENCODERS_stop cmd;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}


bool ArduinoDAQ_LowLevel::CMD_PWM(int pin_index, uint8_t pwm_value)
{
	TFrameCMD_SET_PWM cmd;
	cmd.payload.pin_index = pin_index;
	cmd.payload.analog_value = pwm_value;
	cmd.calc_and_update_checksum();

	return WriteBinaryFrame(reinterpret_cast<uint8_t*>(&cmd), sizeof(cmd));
}
