/*
 * process_EMS22A.cpp
 *
 * Created: 20/07/2017 10:13:46
 *  Author: Francisco José Mañas
 */ 

 /*Beginning of Auto generated code by Atmel studio */
 #include <Arduino.h>
 /*End of auto generated code by Atmel studio */

 #include "arduinodaq_declarations.h"
 #include "arduinodaq2pc-structs.h"
 #include "Encoder_EMS22A.h"

/* Absolute encoder EMS22A reading waveforms (from its datasheet) */
//    ____                                                                  ____
// Cs:    |                                                                |    |
//        |________________________________________________________________|    |_____
//    __________   _   _   _   _   _   _   _   _   _   _   _   _   ______________
// Clk:         | | | | | | | | | | | | | | | | | | | | | | | | | |
//              |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|
//
//                  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _
// DO:__________   / \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \______________
//              |_|\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/
//                  D9-D8-D7-D6-D5-D4-D3-D2-D1-D0-S1-S2-S3-S4-S5-P1
//
// D0-D9: Absolute angular position data.
// S1	: End of offset compensation algorithm.
// S2	: Cordic overflow indicating an error in cordic part.
// S3	: Linearity alarm.
// S4	: Increase in magnitude.
// S5	: Decrease in magnitude.
// P1	: Even parity for detecting bits 1-15 transmission error.

/*  Lectura de Encoder Absoluto EMS22A  */
int ENCODER_ABS_CS  = A1;
int ENCODER_ABS_CLK = A3;
int ENCODER_ABS_DO  = A2;
unsigned long  EMS22A_last_millis        = 0;
uint16_t       EMS22A_sampling_period_ms = 200;
bool           EMS22A_active             = false;

bool init_EMS22A(int8_t init_ENCODER_ABS_CS, int8_t init_ENCODER_ABS_CLK, int8_t init_ENCODER_ABS_DO, uint16_t init_sampling_period_ms)
{
	ENCODER_ABS_CS  = init_ENCODER_ABS_CS;
	ENCODER_ABS_CLK = init_ENCODER_ABS_CLK;
	ENCODER_ABS_DO  = init_ENCODER_ABS_DO;
	EMS22A_sampling_period_ms = init_sampling_period_ms;
	if (ENCODER_ABS_CS<=0 || ENCODER_ABS_CLK<=0 || ENCODER_ABS_DO<=0)
		return false; // error
	
	pinMode(ENCODER_ABS_CS, OUTPUT);
	pinMode(ENCODER_ABS_CLK, OUTPUT);
	pinMode(ENCODER_ABS_DO, INPUT);

	digitalWrite(ENCODER_ABS_CLK, HIGH);
	digitalWrite(ENCODER_ABS_CS, HIGH);
	
	return true; // all ok
}

uint16_t read_EMS22A()
{
	pinMode(ENCODER_ABS_CS, OUTPUT);
	pinMode(ENCODER_ABS_CLK, OUTPUT);
	pinMode(ENCODER_ABS_DO, INPUT);

	digitalWrite(ENCODER_ABS_CS, LOW);
	delayMicroseconds(2);

	uint16_t pos = 0;
	for (int i=0; i<16; i++) {
		 digitalWrite(ENCODER_ABS_CLK, LOW);  delayMicroseconds(1);
		 digitalWrite(ENCODER_ABS_CLK, HIGH); delayMicroseconds(1);
		 
		 pos = pos << 1; // shift 1 bit left
		 if (digitalRead(ENCODER_ABS_DO) == HIGH )
		 {
			 pos |= 0x01;
		 }
	 }

	digitalWrite(ENCODER_ABS_CS, HIGH);
	return pos;
}

void processEMS22A()
{
	if (!EMS22A_active)
	{
		return;
	}
	
	const unsigned long tnow = millis(); /* the current time ("now") */

	if (tnow-EMS22A_last_millis < EMS22A_sampling_period_ms)
	{
		return;
	}
	EMS22A_last_millis = tnow;

	const uint16_t dat = read_EMS22A();
	
	// Extract the position part and the status part
	const uint16_t enc_pos = dat >> 6;
	const uint8_t  enc_status = dat & 0x3f;
	
	TFrame_ENCODER_ABS_reading tx;
	// send answer back:
	tx.payload.timestamp_ms = tnow;
	tx.payload.enc_pos = enc_pos;
	tx.payload.enc_status = enc_status;
	tx.calc_and_update_checksum();

	Serial.write((uint8_t*)&tx,sizeof(tx));

 }
  