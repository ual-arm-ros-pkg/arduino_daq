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

#include <Wire.h>
//#include <SPI.h>


unsigned long  PC_last_millis = 0;
uint16_t       PC_sampling_period_ms = 500;
int8_t enc0A_pin=0, enc1A_pin=0;

uint8_t enc0B_bit, enc0B_port;
uint8_t enc0Z_bit, enc0Z_port;

uint8_t enc1B_bit, enc1B_port;
uint8_t enc1Z_bit, enc1Z_port;

volatile int32_t  ENC_COUNTERS[2] = {0,0};

// Forward:
//        ------      ------
// A:     |    |      |    |
//    ____|    |______|    |_____
//          ------      ------
// B:       |    |      |    |
//   _______|    |______|    |_____
//
//

void onEncoder_Raising_A0()
{
	// Avoid: digitalRead() "slow" call
	const bool B = (*portInputRegister(enc0B_port) & enc0B_bit);
	if (B) 
	     ENC_COUNTERS[0]--;
	else ENC_COUNTERS[0]++;

	if (enc0Z_port) {
		const bool Z = (*portInputRegister(enc0Z_port) & enc0Z_bit);
		if (Z) {
			ENC_COUNTERS[0]=0;
		}
	}
}

void onEncoder_Raising_A1()
{
	// Avoid: digitalRead() "slow" call
	const bool B = (*portInputRegister(enc1B_port) & enc1B_bit);
	if (B)
	     ENC_COUNTERS[1]--;
	else ENC_COUNTERS[1]++;

	if (enc1Z_port) {
		const bool Z = (*portInputRegister(enc1Z_port) & enc1Z_bit);
		if (Z) {
			ENC_COUNTERS[1]=0;
		}
	}
}

void init_encoders(
	int8_t _enc0A_pin, int8_t _enc0B_pin, int8_t _enc0Z_pin, 
	int8_t _enc1A_pin, int8_t _enc1B_pin, int8_t _enc1Z_pin,
	uint16_t sampling_period_ms)
{
	enc0A_pin=_enc0A_pin;
	enc1A_pin=_enc1A_pin;
	PC_sampling_period_ms = sampling_period_ms;

	if (enc0A_pin>0) {
		// Cache these calls to avoid repeating them in readDigital() inside the interrupt vector ;-)
		enc0B_bit = digitalPinToBitMask(_enc0B_pin);
		enc0B_port = digitalPinToPort(_enc0B_pin);
		if (_enc0Z_pin!=0)
		{
			enc0Z_bit = digitalPinToBitMask(_enc0Z_pin);
			enc0Z_port = digitalPinToPort(_enc0Z_pin);
		}
		else
		{
			enc0Z_bit = enc0Z_port = 0;
		}

		attachInterrupt(digitalPinToInterrupt(enc0A_pin), &onEncoder_Raising_A0, RISING );
	}
	if (enc1A_pin>0) {
		// Cache these calls to avoid repeating them in readDigital() inside the interrupt vector ;-)
		enc1B_bit = digitalPinToBitMask(_enc1B_pin);
		enc1B_port = digitalPinToPort(_enc1B_pin);
		if (_enc1Z_pin!=0)
		{
			enc1Z_bit = digitalPinToBitMask(_enc1Z_pin);
			enc1Z_port = digitalPinToPort(_enc1Z_pin);
		}
		else
		{
			enc0Z_bit = enc0Z_port = 0;
		}

		attachInterrupt(digitalPinToInterrupt(enc1A_pin), &onEncoder_Raising_A1, RISING );
	}
}


void processEncoders()
{
	const unsigned long tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms)
	return;

	PC_last_millis = tnow;

	TFrame_ENCODERS_readings tx;

	// Atomic read: used to avoid race condition while reading if an interrupt modified the mid-read data.
	noInterrupts();
	tx.payload.encoders[0] = ENC_COUNTERS[0];
	tx.payload.encoders[1] = ENC_COUNTERS[1];
	interrupts();

	// send answer back:
	tx.payload.timestamp_ms = millis();
	tx.payload.period_ms = PC_sampling_period_ms;
	tx.calc_and_update_checksum();

	Serial.write((uint8_t*)&tx,sizeof(tx));
}
