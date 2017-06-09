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


/*Beginning of Auto generated code by Atmel studio */
#include <Arduino.h>
/*End of auto generated code by Atmel studio */

#include "arduinodaq_declarations.h"
#include "arduinodaq2pc-structs.h"

#include "mod_dac_max5500.h"

// Originally designed for atmega328P.
// --------------------------------------

#include <Wire.h>
#include <SPI.h>

//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// Fixed pins configuration for this hardware:
#include "config.h"


void flash_led(int ntimes, int nms)
{
	pinMode(PIN_LED,OUTPUT);
	for (int i=0;i<ntimes;i++)
	{
		digitalWrite(PIN_LED, HIGH); delay(nms);
		digitalWrite(PIN_LED, LOW); delay(nms);
	}
}

static void send_simple_opcode_frame(const uint8_t op)
{
	const uint8_t rx[] = { FRAME_START_FLAG, op, 0x00, 0x00, FRAME_END_FLAG };
	Serial.write(rx,sizeof(rx));
}

void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data)
{
	switch (opcode)
	{
	case OP_SET_DAC:
	{
		if (datalen!=sizeof(TFrameCMD_SetDAC_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);
		
		// Init upon first usage:
		static bool dac_init = false;
		if (!dac_init)
		{
			mod_dac_max5500_init();
			dac_init = true;
		}
		const uint8_t dac_idx = data[0];
		const uint16_t dac_value = (uint16_t(data[1]) << 8)  | data[2];
		mod_dac_max5500_update_single_DAC(dac_idx,dac_value);

		// send answer back:
		send_simple_opcode_frame(RESP_SET_DAC);
	}
	break;
	case OP_SET_GPIO:
	{
		if (datalen!=sizeof(TFrameCMD_GPIO_output_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		const uint8_t pin_no = data[0];
		const uint8_t pin_val = data[1];
		pinMode(pin_no, OUTPUT);
		digitalWrite(pin_no, pin_val);

		// send answer back:
		send_simple_opcode_frame(RESP_SET_GPIO);
	}
	break;
	case OP_GET_GPIO:
	{
		if (datalen!=sizeof(TFrameCMD_GPIO_read_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		const uint8_t pin_no = data[0];
		pinMode(pin_no, INPUT);
		const uint8_t val = digitalRead(pin_no);

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, RESP_GET_GPIO, 0x01, pin_no, val, 0x00 +pin_no+ val/*checksum*/, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;

	case OP_START_CONT_ADC:
	{
		if (datalen!=sizeof(TFrameCMD_ADC_start_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_ADC_start_payload_t adc_req;
		memcpy(&adc_req,data, sizeof(adc_req));

		// Setup vars for ADC task:
		num_active_ADC_channels = 0;
		for (int i=0;i<8;i++) {
			ADC_active_channels[i] = 0;
			if (adc_req.active_channels[i]>=0) {
				ADC_active_channels[i] = adc_req.active_channels[i];
				num_active_ADC_channels++;
			}
		}
		ADC_sampling_period_ms = adc_req.measure_period_ms;
		
		// Enable ADC with internal/default reference:
		analogReference( adc_req.use_internal_refvolt ? INTERNAL : DEFAULT );

		// send answer back:
		send_simple_opcode_frame(RESP_START_CONT_ADC);
	}
	break;

	case OP_STOP_CONT_ADC:
	{
		if (datalen!=sizeof(TFrameCMD_ADC_stop_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		num_active_ADC_channels = 0;

		// send answer back:
		send_simple_opcode_frame(RESP_STOP_CONT_ADC);
	}
	break;

	case OP_SET_PWM:
	{
		if (datalen!=sizeof(TFrameCMD_SET_PWM_payload_t)) return send_simple_opcode_frame(RESP_WRONG_LEN);

		TFrameCMD_SET_PWM_payload_t pwm_req;
		memcpy(&pwm_req,data, sizeof(pwm_req));

		analogWrite(pwm_req.pin_index, pwm_req.analog_value);

		// send answer back:
		send_simple_opcode_frame(RESP_SET_PWM);
	}
	break;

	default:
	{
		// Error:
		send_simple_opcode_frame(RESP_UNKNOWN_OPCODE);
	}
	break;
	};
}
