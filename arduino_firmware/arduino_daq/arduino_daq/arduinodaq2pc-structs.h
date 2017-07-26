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
	OP_NOP             = 0x00,
	OP_SET_DAC         = 0x10,
	OP_SET_GPIO        = 0x11,
	OP_GET_GPIO        = 0x12,
	OP_START_CONT_ADC  = 0x20,
	OP_STOP_CONT_ADC   = 0x21,
	OP_SET_PWM         = 0x25,
	OP_START_ENCODERS  = 0x30,
	OP_STOP_ENCODERS   = 0x31,
	OP_START_EMS22A	   = 0x40, //
	OP_STOP_EMS22A     = 0x41, //

	// -----------------------------
	// Responses Arduino -> PC
	// -----------------------------
	RESP_OFFSET = 0x70,
	// -----------------------------
	RESP_NOP              = OP_NOP + RESP_OFFSET,
	RESP_SET_DAC          = OP_SET_DAC + RESP_OFFSET,
	RESP_SET_GPIO         = OP_SET_GPIO + RESP_OFFSET,
	RESP_GET_GPIO         = OP_GET_GPIO + RESP_OFFSET,
	RESP_START_CONT_ADC   = OP_START_CONT_ADC + RESP_OFFSET,
	RESP_STOP_CONT_ADC    = OP_STOP_CONT_ADC + RESP_OFFSET,
	RESP_START_ENCODERS   = OP_START_ENCODERS + RESP_OFFSET,
	RESP_STOP_ENCODERS    = OP_STOP_ENCODERS + RESP_OFFSET,
	RESP_START_EMS22A     = OP_START_EMS22A + RESP_OFFSET, //
	RESP_STOP_EMS22A      = OP_STOP_EMS22A + RESP_OFFSET,  //
	RESP_ADC_READINGS     = 0x92,
	RESP_ENCODER_READINGS = 0x93,
	RESP_EMS22A_READINGS  = 0x94,
	RESP_SET_PWM          = OP_SET_PWM + RESP_OFFSET,


	// error codes:
	RESP_CHECKSUM_ERROR    = 0xfa,
	RESP_FRAME_ERROR       = 0xfb,
	RESP_INVALID_PARAMS    = 0xfc,
	RESP_WRONG_LEN         = 0xfd,
	RESP_UNKNOWN_OPCODE    = 0xfe
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

struct TFrameCMD_NOP_payload_t
{
};
struct TFrameCMD_NOP : public TBaseFrame<TFrameCMD_NOP_payload_t>
{
	TFrameCMD_NOP() : TBaseFrame(OP_NOP)
	{
	}
};

struct TFrameCMD_SetDAC_payload_t
{
    uint8_t  dac_index;
    uint8_t  dac_value_HI, dac_value_LO;
    uint8_t  flag_enable_timeout : 1;   // bitfield: 1 bit flag
    TFrameCMD_SetDAC_payload_t() :
        dac_index(0), dac_value_HI(0), dac_value_LO(0),flag_enable_timeout(0)
    {
    }
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
	uint8_t  flag_enable_timeout : 1;   // bitfield: 1 bit flag
	TFrameCMD_SET_PWM_payload_t() :
		pin_index(0),
		analog_value(0),
		flag_enable_timeout(0)
	{
	}
};
struct TFrameCMD_SET_PWM : public TBaseFrame<TFrameCMD_SET_PWM_payload_t>
{
	// Defaults:
	TFrameCMD_SET_PWM() : TBaseFrame(OP_SET_PWM)
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
	TFrame_ADC_readings() : TBaseFrame(RESP_ADC_READINGS)
	{
	}
};


struct TFrameCMD_ENCODERS_start_payload_t
{
	static const uint8_t NUM_ENCODERS = 2;

	/** Fill pin numbers ("Arduino-based numbering") that want to get used as quadrature encoder A,B & Z channels. 
	  * Leave to "0" if don't need Z channels or one of the A/B encoder channels.
	  */
	int8_t encA_pin[NUM_ENCODERS], encB_pin[NUM_ENCODERS], encZ_pin[NUM_ENCODERS];
	uint16_t sampling_period_ms;

	TFrameCMD_ENCODERS_start_payload_t() :
		sampling_period_ms(250)
	{
		for (uint8_t i=0;i<NUM_ENCODERS;i++) {
			encA_pin[i]=encB_pin[i]=encZ_pin[i]=0;
		}
	}
};
struct TFrameCMD_ENCODERS_start : public TBaseFrame<TFrameCMD_ENCODERS_start_payload_t>
{
	// Defaults:
	TFrameCMD_ENCODERS_start() : TBaseFrame(OP_START_ENCODERS)
	{
	}
};

struct TFrameCMD_ENCODERS_stop_payload_t
{
};
struct TFrameCMD_ENCODERS_stop : public TBaseFrame<TFrameCMD_ENCODERS_stop_payload_t>
{
	// Defaults:
	TFrameCMD_ENCODERS_stop() : TBaseFrame(OP_STOP_ENCODERS)
	{
	}
};


struct TFrame_ENCODERS_readings_payload_t
{
	uint32_t timestamp_ms;
	int32_t  encoders[2];
	uint32_t period_ms;
};
struct TFrame_ENCODERS_readings : public TBaseFrame<TFrame_ENCODERS_readings_payload_t>
{
	// Defaults:
	TFrame_ENCODERS_readings() : TBaseFrame(RESP_ENCODER_READINGS)
	{
	}
};

struct TFrame_ENCODER_ABS_reading_payload_t
{
	uint32_t timestamp_ms;
	uint16_t enc_pos; //!< Absolute value read from the encoder (10 bits resolution)
	uint8_t  enc_status; //!< See EMS22A datasheet for the bit map
};
struct TFrame_ENCODER_ABS_reading : public TBaseFrame<TFrame_ENCODER_ABS_reading_payload_t>
{
	// Defaults:
	TFrame_ENCODER_ABS_reading() : TBaseFrame(RESP_EMS22A_READINGS)
	{
	}
};

struct TFrameCMD_EMS22A_start_payload_t
{
	int8_t ENCODER_ABS_CS, ENCODER_ABS_CLK, ENCODER_ABS_DO;
	uint16_t sampling_period_ms;

	TFrameCMD_EMS22A_start_payload_t() :
		sampling_period_ms(50)
	{
		ENCODER_ABS_CS=ENCODER_ABS_CLK=ENCODER_ABS_DO=0;
	}
};

struct TFrameCMD_EMS22A_start : public TBaseFrame<TFrameCMD_EMS22A_start_payload_t>
{
	// Defaults:
	TFrameCMD_EMS22A_start() : TBaseFrame(OP_START_EMS22A)
	{
	}
};

struct TFrameCMD_EMS22A_stop_payload_t
{
};
struct TFrameCMD_EMS22A_stop : public TBaseFrame<TFrameCMD_EMS22A_stop_payload_t>
{
	// Defaults:
	TFrameCMD_EMS22A_stop() : TBaseFrame(OP_STOP_EMS22A)
	{
	}
};


#if !defined(__AVR_MEGA__)
#	pragma pack(pop)
#endif
