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

#include "arduinodaq2pc-structs.h"

#include <stdint.h>  // uint8_t, etc.

// Function prototypes:
void reset_rx_buf();
void processIncommingPkts();
void processADCs();
void processEncoders();
void processEMS22A();
void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data);
void process_timeouts();
void flash_led(int ntimes, int nms);
void init_encoders(const TFrameCMD_ENCODERS_start_payload_t &cmd);
void send_simple_opcode_frame(const uint8_t op);

// Global vars:
extern uint8_t        num_active_ADC_channels;
#define MAX_ADC_CHANNELS  8
extern uint8_t        ADC_active_channels[MAX_ADC_CHANNELS];
extern uint16_t       ADC_sampling_period_ms;

