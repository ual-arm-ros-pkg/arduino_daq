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

// ADC reading subsystem:
uint8_t        num_active_ADC_channels = 0;
uint8_t        ADC_active_channels[MAX_ADC_CHANNELS] = {0,0,0,0,0,0,0,0};
unsigned long  ADC_last_millis = 0;
uint16_t       ADC_sampling_period_ms = 200;

void processADCs()
{
	if (!num_active_ADC_channels)
		return;

	const unsigned long tnow = millis();

	if (tnow-ADC_last_millis < ADC_sampling_period_ms)
		return;

	ADC_last_millis = tnow;

	uint16_t ADC_readings[MAX_ADC_CHANNELS] =  {0,0,0,0,0,0,0,0};
	
	for (uint8_t i=0;i<num_active_ADC_channels;i++)
	{
		ADC_readings[i] = analogRead(ADC_active_channels[i]);
	}

	// send answer back:
	TFrame_ADC_readings tx;
	tx.payload.timestamp_ms = millis();
	for (int i=0;i<8;i++) {
		tx.payload.adc_data[i] = ADC_readings[i];
	}
	tx.calc_and_update_checksum();

	Serial.write((uint8_t*)&tx,sizeof(tx));
}
