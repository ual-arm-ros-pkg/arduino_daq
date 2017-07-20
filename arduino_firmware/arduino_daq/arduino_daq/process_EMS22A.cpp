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

 /*	Proceso lectura del encoder según la hoja del fabricante*/
 //    ____								                                    ____
 // Cs:    |								                                   |	|
 //        |________________________________________________________________|	|_____
 //    __________   _   _   _   _   _   _   _   _   _   _   _   _   ______________
 // Clk:			| | | | | | | | | | | | | | | | | | | | | | | | | |
 //				|_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_| |_|
 //
 //					_  _  _  _  _  _  _  _  _  _  _  _  _  _  _  _
 // DO:__________   / \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \/ \______________
 //				|_|\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/\_/
 //				   D9-D8-D7-D6-D5-D4-D3-D2-D1-D0-S1-S2-S3-S4-S5-P1
 //
 // D0-D9: Absolute angular position data.
 // S1	: End of offset compensation algorithm.
 // S2	: Cordic overflow indicating an error in cordic part.
 // S3	: Linearity alarm.
 // S4	: Increase in magnitude.
 // S5	: Decrease in magnitude.
 // P1	: Even parity for detecting bits 1-15 transmission error.

 /*  Lectura de Encoder Absoluto EMS22A  */
 const int		ENCODER_ABS_CS				= A1;
 const int		ENCODER_ABS_CLK				= A3;
 const int		ENCODER_ABS_DO				= A2;
 uint16_t		pos							= 0;
 unsigned long  EMS22A_last_millis			= 0;
 uint16_t       EMS22A_sampling_period_ms	= 200;
 boolean		EMS22A_active				= true;

 void init_EMS22A()
 {
	 pinMode(ENCODER_ABS_CS, OUTPUT);
	 pinMode(ENCODER_ABS_CLK, OUTPUT);
	 pinMode(ENCODER_ABS_DO, INPUT);

	 digitalWrite(ENCODER_ABS_CLK, HIGH);
	 digitalWrite(ENCODER_ABS_CS, LOW);
	 
 }

 uint16_t read_EMS22A()
 {
	 digitalWrite(ENCODER_ABS_CS, HIGH);
	 digitalWrite(ENCODER_ABS_CS, LOW);

	 for (int i=0; i<16; i++) {
		 digitalWrite(ENCODER_ABS_CLK, LOW);
		 digitalWrite(ENCODER_ABS_CLK, HIGH);
		 
		 pos = pos << 1; // shift 1 bit left
		 if (digitalRead(ENCODER_ABS_DO) == HIGH )
		 {
			 pos |= 0x01;
		 }
	 }
	 digitalWrite(ENCODER_ABS_CLK, LOW);
	 digitalWrite(ENCODER_ABS_CLK, HIGH);
	 return pos;
 }

 void processEMS22A()
 {
	if (!EMS22A_active)
	{
		return;
	}
	
	const unsigned long tnow = millis(); /*¿Significado?*/

	if (tnow-EMS22A_last_millis < EMS22A_sampling_period_ms)
	{
		return;
	}
	EMS22A_last_millis = tnow;

	uint16_t dat = read_EMS22A();
	Serial.println(dat);

 }