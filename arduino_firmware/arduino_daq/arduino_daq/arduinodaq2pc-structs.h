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

#include <stdlib.h>

#if !defined(__AVR_MEGA__)
#	pragma pack(push, 1) // exact fit - no padding
#endif

/*
START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
  0x69          1 byte      1 byte       N bytes       =sum(data)       0x96

## Computer => controller
* 0x10: Set DAC value. DATA_LEN = 3
	* DATA[0]   = DAC index
	* DATA[1:2] = DAC value (0x0000-0xffff)  (MSByte first!)
* 0x11: Set GPIO pin. DATA_LEN = 2
	* DATA[0]   = Arduino-based pin number
	* DATA[1]   = 0/1
* 0x12: Read GPIO pin. DATA_LEN = 1
	* DATA[0]   = Arduino-based pin number
* 0x20: Start ADC continuous acquisition task
* 0x21: Stop ADC task
*/

#define FRAME_START_FLAG  0x69
#define FRAME_END_FLAG    0x96

enum opcode_t {
	// -----------------------------
	// COMMANDS PC -> Arduino
	// -----------------------------
	OP_SET_DAC         = 0x10,
	OP_SET_GPIO        = 0x11,
	OP_GET_GPIO        = 0x12,
	OP_START_CONT_ADC  = 0x20,
	OP_STOP_CONT_ADC   = 0x21,
	OP_SET_PWM         = 0x25,
	
	// -----------------------------
	// Responses Arduino -> PC
	// -----------------------------
	RESP_OFFSET = 0x70,
	// -----------------------------
	RESP_SET_DAC          = OP_SET_DAC + RESP_OFFSET,
	RESP_SET_GPIO         = OP_SET_GPIO + RESP_OFFSET,
	RESP_GET_GPIO         = OP_GET_GPIO + RESP_OFFSET,
	RESP_START_CONT_ADC   = OP_START_CONT_ADC + RESP_OFFSET,
	RESP_STOP_CONT_ADC    = OP_STOP_CONT_ADC + RESP_OFFSET,
	RESP_SET_PWM          = OP_SET_PWM + RESP_OFFSET,


	// error code:
	RESP_ERROR    = 0xfe
};


template <typename Payload>
struct TBaseFrame
{
    const uint8_t  START_FLAG;
    const uint8_t  OPCODE;
    const uint8_t  DATALEN;
    // --------- Payload -------
    Payload  payload;
    // -------------------------
    uint8_t  CHECKSUM;
    const uint8_t  END_FLAG;

    // Defaults:
    TBaseFrame(uint8_t opcode) :
            START_FLAG(FRAME_START_FLAG),
            OPCODE(opcode),
            DATALEN(sizeof(Payload)),
            END_FLAG(FRAME_END_FLAG)
    {
    }

    void calc_and_update_checksum()
    {
        CHECKSUM = calc_checksum();    
    }
    
    uint8_t calc_checksum() const
    {
        const uint8_t len   = DATALEN; //reinterpret_cast<const uint8_t*>(ptr_frame)[2];
        const uint8_t *data = reinterpret_cast<const uint8_t*>(&payload);
        uint8_t ret =0;
        for (unsigned int i=0;i<len;i++) ret+=*data++;
        return ret;
    }
};

struct TFrameCMD_SetDAC_payload_t
{
    uint8_t  dac_index;
    uint8_t  dac_value_HI, dac_value_LO;
};
struct TFrameCMD_SetDAC : public TBaseFrame<TFrameCMD_SetDAC_payload_t>
{
    // Defaults:
    TFrameCMD_SetDAC() : TBaseFrame(OP_SET_DAC)
    {
    }
};

struct TFrameCMD_GPIO_output_payload_t
{
    uint8_t  pin_index;
    uint8_t  pin_value;
};
struct TFrameCMD_GPIO_output : public TBaseFrame<TFrameCMD_GPIO_output_payload_t>
{
    // Defaults:
    TFrameCMD_GPIO_output() : TBaseFrame(OP_SET_GPIO)
    {
    }
};

struct TFrameCMD_GPIO_read_payload_t
{
	uint8_t  pin_index;
};
struct TFrameCMD_GPIO_read : public TBaseFrame<TFrameCMD_GPIO_read_payload_t>
{
	// Defaults:
	TFrameCMD_GPIO_read() : TBaseFrame(OP_GET_GPIO)
	{
	}
};


struct TFrameCMD_ADC_start_payload_t
{
	/** Fill all the pins ("Arduino-based numbering") that want to get read with an ADC. Default values = -1, means ignore that channel.
	  */
	int8_t   active_channels[8];
	uint16_t measure_period_ms; //!< Default = 200
	uint8_t  use_internal_refvolt; //!< 0 or 1. Default=0

	TFrameCMD_ADC_start_payload_t() :
		measure_period_ms(200),
		use_internal_refvolt(0)
	{
		for (int i=0;i<8;i++) { 
			active_channels[i]=-1; 
		}
	}
};
struct TFrameCMD_ADC_start : public TBaseFrame<TFrameCMD_ADC_start_payload_t>
{
	// Defaults:
	TFrameCMD_ADC_start() : TBaseFrame(OP_START_CONT_ADC)
	{
	}
};

struct TFrameCMD_ADC_stop_payload_t
{
};
struct TFrameCMD_ADC_stop : public TBaseFrame<TFrameCMD_ADC_stop_payload_t>
{
	// Defaults:
	TFrameCMD_ADC_stop() : TBaseFrame(OP_STOP_CONT_ADC)
	{
	}
};

struct TFrameCMD_SET_PWM_payload_t
{
	uint8_t  pin_index;
	uint8_t  analog_value; //!< 0-255 maps to 0% to 100% duty cycle
	TFrameCMD_SET_PWM_payload_t() : 
		pin_index(0),
		analog_value(0)
	{
	}
};
struct TFrameCMD_SET_PWM : public TBaseFrame<TFrameCMD_SET_PWM_payload_t>
{
	// Defaults:
	TFrameCMD_SET_PWM() : TBaseFrame(RESP_SET_PWM)
	{
	}
};

struct TFrame_ADC_readings_payload_t
{
	uint32_t timestamp_ms;
	uint16_t adc_data[8];
};
struct TFrame_ADC_readings : public TBaseFrame<TFrame_ADC_readings_payload_t>
{
	// Defaults:
	TFrame_ADC_readings() : TBaseFrame(0x92)
	{
	}
};

struct TFrame_PULSE_COUNTER_readings_payload_t
{
	uint32_t timestamp_ms;
	uint16_t pulse_counter;
	uint32_t period_ms;
};
struct TFrame_PULSE_COUNTER_readings : public TBaseFrame<TFrame_PULSE_COUNTER_readings_payload_t>
{
	// Defaults:
	TFrame_PULSE_COUNTER_readings() : TBaseFrame(0x93)
	{
	}
};


#if !defined(__AVR_MEGA__)
#	pragma pack(pop)
#endif
