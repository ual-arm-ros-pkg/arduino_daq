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

#include "arduinodaq2pc-structs.h"
#include "config.h"

#include <Wire.h>
//#include <SPI.h>

unsigned long  PC_last_millis = 0;
uint16_t       PC_sampling_period_ms = 500;
bool           ENCODERS_active       = false;

struct EncoderStatus
{
	// Config:
	volatile int8_t encA_pin=0;  // =0 means disabled.
	volatile int8_t encB_valid=0,encB_bit=0, encB_port=0;
	volatile int8_t encZ_valid=0,encZ_bit=0, encZ_port=0; // encZ_port=0 means no Z
	volatile bool led = false;
	
	// Current encoder tick counter value:
	volatile int32_t  COUNTER = 0;
	// Init:
	EncoderStatus()
	{ }
};

EncoderStatus ENC_STATUS[TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS];

// Forward:
//        ------      ------
// A:     |    |      |    |
//    ____|    |______|    |_____
//          ------      ------
// B:       |    |      |    |
//   _______|    |______|    |_____
//
//
template <uint8_t index>  // Generic template to generate N functions for different index of encoder=0,1,...
static void onEncoder_Raising_A()
{
#ifdef USE_ENCODER_DEBUG_LED
	ENC_STATUS[index].led = !ENC_STATUS[index].led;
#endif

	// Avoid: digitalRead() "slow" call
	if (ENC_STATUS[index].encB_valid)
	{
		const bool B = (*portInputRegister(ENC_STATUS[index].encB_port) & ENC_STATUS[index].encB_bit);
		if (B)
		ENC_STATUS[index].COUNTER--;
		else ENC_STATUS[index].COUNTER++;	
	}
	else 
	{
		ENC_STATUS[index].COUNTER++;
	}

#if 0
	if (ENC_STATUS[index].encZ_valid) 
	{
		const bool Z = (*portInputRegister(ENC_STATUS[index].encZ_port) & ENC_STATUS[index].encZ_bit);
		if (Z) {
			ENC_STATUS[index].COUNTER=0;
		}
	}
#endif
}

// List of function pointers, required by Arduino attachInterrupt() and also 
// to be able to dynamically get a pointer by index, which is not directly allowed
// with template arguments.
typedef void (*func_ptr)(void);
func_ptr my_encoder_ISRs[TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS] = 
{
	&onEncoder_Raising_A<0>,
	&onEncoder_Raising_A<1>
	//... Add more if needed in the future
};

void init_encoders(const TFrameCMD_ENCODERS_start_payload_t &cmd)
{
	PC_sampling_period_ms = cmd.sampling_period_ms;

	// For each software-based encoder:
	for (uint8_t i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++)
	{
		ENC_STATUS[i] = EncoderStatus();

		ENC_STATUS[i].encA_pin = cmd.encA_pin[i];
		
		// Is it enabled by the user?
		if (cmd.encA_pin[i]>0) 
		{
			if (cmd.encB_pin[i]>0)
			{
				// Cache these calls to avoid repeating them in readDigital() inside the interrupt vector ;-)
				ENC_STATUS[i].encB_bit = digitalPinToBitMask( cmd.encB_pin[i] );
				ENC_STATUS[i].encB_port = digitalPinToPort(cmd.encB_pin[i]);
				ENC_STATUS[i].encB_valid = true;
			}
			else 				
				ENC_STATUS[i].encB_valid = false;

			if (cmd.encZ_pin>0)
			{
				ENC_STATUS[i].encZ_bit = digitalPinToBitMask(cmd.encZ_pin[i]);
				ENC_STATUS[i].encZ_port = digitalPinToPort(cmd.encZ_pin[i]);
				ENC_STATUS[i].encZ_valid = true;
			}
			else
			{
				ENC_STATUS[i].encZ_valid = false;
				ENC_STATUS[i].encZ_bit = ENC_STATUS[i].encZ_port = 0;
			}
			
			attachInterrupt(digitalPinToInterrupt(cmd.encA_pin[i]), my_encoder_ISRs[i], RISING );
		}
	}
}

void processEncoders()
{
	if (!ENCODERS_active)
		return;

#ifdef USE_ENCODER_DEBUG_LED
	pinMode(PIN_ENCODER_DEBUG_LED, OUTPUT);
	digitalWrite(PIN_ENCODER_DEBUG_LED,ENC_STATUS[0].led);
#endif

	const unsigned long tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms)
	return;

	PC_last_millis = tnow;

	TFrame_ENCODERS_readings tx;

	// Atomic read: used to avoid race condition while reading if an interrupt modified the mid-read data.
	noInterrupts();
	for (uint8_t i=0;i<TFrameCMD_ENCODERS_start_payload_t::NUM_ENCODERS;i++)
	{
		tx.payload.encoders[i] = ENC_STATUS[i].COUNTER;
	}
	interrupts();

	// send answer back:
	tx.payload.timestamp_ms = millis();
	tx.payload.period_ms = PC_sampling_period_ms;
	tx.calc_and_update_checksum();

	Serial.write((uint8_t*)&tx,sizeof(tx));
}
