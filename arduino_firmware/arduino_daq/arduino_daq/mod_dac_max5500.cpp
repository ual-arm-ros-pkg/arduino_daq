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

#include <Wire.h>
#include <SPI.h>

#include "mod_dac_max5500.h"

#include "config.h" // Fixed pins configuration for this hardware

/* SPI frames for the MAX5500 chip
           16 bits
     MSB --- LSB (MSB first)
|A1 A0  |  C1 C0 | D11 ... D0 |

Commands C1.C0:
 * 0 1: Load DAC, do NOT update DAC output.
 * 1 1: Load DAC, update ALL DAC outputs.

- POLARITY: CPOL=0 (inactive SCK=low level)
- PHASE: CPHA=1 (master changes data on the falling edge of clock)
 ==> SPI_MODE_0 (1?)

*/

void mod_dac_max5500_init()
{
	// start the SPI library:
	SPI.begin();
	digitalWrite(PIN_DAC_MAX5500_CS, HIGH);  // /CS=1
	pinMode(PIN_DAC_MAX5500_CS, OUTPUT);
	// Set all chip outputs to 0V
	mod_dac_max5500_send_spi_word(0x8000);
}

void mod_dac_max5500_send_spi_word(uint16_t tx_word)
{
	// Send HiByte:
	SPI.beginTransaction(SPISettings(8000000, MSBFIRST, SPI_MODE0));

	// nCS -> 0
	digitalWrite(PIN_DAC_MAX5500_CS, LOW);
	delayMicroseconds(1); // Min. 40ns: nCS->0 to SCK

	SPI.transfer16 (tx_word);

	delayMicroseconds(1);

	// nCS -> 1
	digitalWrite(PIN_DAC_MAX5500_CS, HIGH);

	SPI.endTransaction();
}

void mod_dac_max5500_update_single_DAC(uint8_t dac_idx, uint16_t dac_value)
{
	// See word format at the top of this file
	const uint16_t tx_word =
	(((uint16_t)dac_idx) << 14) |
	(((uint16_t)0x03)    << 12) |
	(dac_value & 0x0fff);

	mod_dac_max5500_send_spi_word(tx_word);
}

