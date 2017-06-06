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

#include "mod_dac_max5500.h"

// Originally designed for atmega328P.
// --------------------------------------

#include <Wire.h>
#include <SPI.h>

//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// Fixed pins configuration for this hardware:
#include "config.h"

unsigned long  PC_last_millis = 0;
uint16_t       PC_sampling_period_ms = 500;

//	attachInterrupt(digitalPinToInterrupt(PIN_PULSE_COUNTER), &onPulseEdge, RISING );

void processPulseCounter()
{
	const unsigned long tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms)
	return;

	PC_last_millis = tnow;

	uint16_t read_pulses;

	// ATOMIC READ:
	noInterrupts();
//	read_pulses = PULSE_COUNTER;
//	PULSE_COUNTER=0;
	interrupts();

	// send answer back:
	TFrame_PULSE_COUNTER_readings tx;
	tx.payload.timestamp_ms = millis();
	tx.payload.period_ms = PC_sampling_period_ms;
	tx.payload.pulse_counter = read_pulses;
	
	tx.calc_and_update_checksum();

	Serial.write((uint8_t*)&tx,sizeof(tx));
}
