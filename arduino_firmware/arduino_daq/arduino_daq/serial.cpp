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
#include "arduinodaq_declarations.h"

// Frame format: see README.md
uint8_t rx_buf_len = 0;
uint8_t rx_buf[30];

void reset_rx_buf()
{
	rx_buf_len = 0;
}

void processIncommingPkts()
{
	while (Serial.available())
	{
		const uint8_t b = Serial.read();

		// sanity:
		if (rx_buf_len==0)
			if (b!=FRAME_START_FLAG) {
				reset_rx_buf();
				send_simple_opcode_frame(RESP_FRAME_ERROR);
				continue;
			}

		// store:
		rx_buf[rx_buf_len++] = b;

		//char buf[10];
		//sprintf(buf,"rx: 0x%02X\n",b);
		//Serial.print(buf);

		if (rx_buf_len==5+rx_buf[2])
		{
			// Check if we have a full frame:
			if (rx_buf[rx_buf_len-1]!=FRAME_END_FLAG) {
				reset_rx_buf();
				send_simple_opcode_frame(RESP_FRAME_ERROR);
				continue;
			}
			const uint8_t opcode  = rx_buf[1];
			const uint8_t datalen = rx_buf[2];
			const uint8_t *data   = rx_buf+3;

			// chksum:
			uint8_t chksum = 0;
			for (uint8_t i=0;i<datalen;i++) chksum+=data[i];
			if (rx_buf[rx_buf_len-2]!=chksum) {
				reset_rx_buf();
				send_simple_opcode_frame(RESP_CHECKSUM_ERROR);
				continue;
			}

			// 100% sure: we have a valid frame: dispatch it:
			process_command(opcode,datalen,data);

			reset_rx_buf();
		}
	}
}

