/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

#include "arduinodaq2pc-structs.h"

/*End of auto generated code by Atmel studio */

// Originally designed for atmega328P.
// --------------------------------------

#include <Wire.h>
#include <SPI.h>

//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// Pin mapping:
const int PIN_DAC_MAX5500_CS  = 9;
const int PIN_LED             = 13;
const int PIN_PULSE_COUNTER   = 2;    // PCINT0


// ADC reading subsystem:
uint8_t        num_active_ADC_channels = 0;
const uint8_t  MAX_ADC_CHANNELS = 8;
uint8_t        ADC_active_channels[MAX_ADC_CHANNELS] = {0,0,0,0,0,0,0,0};
unsigned long  ADC_last_millis = 0;
uint16_t       ADC_sampling_period_ms = 200;

unsigned long  PC_last_millis = 0;
uint16_t       PC_sampling_period_ms = 500;


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

void flash_led(int ntimes, int nms)
{
	pinMode(PIN_LED,OUTPUT);
	for (int i=0;i<ntimes;i++)
	{
		digitalWrite(PIN_LED, HIGH); delay(nms);
		digitalWrite(PIN_LED, LOW); delay(nms);
	}
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

uint16_t  PULSE_COUNTER = 0;

void onPulseEdge()
{
	PULSE_COUNTER++;
}

void setup()
{
	// start the SPI library:
	SPI.begin();

	digitalWrite(PIN_DAC_MAX5500_CS, HIGH);  // /CS=1
	pinMode(PIN_DAC_MAX5500_CS, OUTPUT);

	// Set all chip outputs to 0V
	mod_dac_max5500_send_spi_word(0x8000);

	// Enable ADC with internal reference:
	analogReference(INTERNAL);

	attachInterrupt(digitalPinToInterrupt(PIN_PULSE_COUNTER), &onPulseEdge, RISING );


	Serial.begin(9600);
}


// Frame format: see README.md

uint8_t rx_buf_len = 0;
uint8_t rx_buf[30];

void reset_rx_buf()
{
	rx_buf_len = 0;
}

void process_command(const uint8_t opcode, const uint8_t datalen, const uint8_t*data)
{
	switch (opcode)
	{
	case 0x10:
	{
		if (datalen!=3) return;

		const uint8_t dac_idx = data[0];
		const uint16_t dac_value = (uint16_t(data[1]) << 8)  | data[2];
		mod_dac_max5500_update_single_DAC(dac_idx,dac_value);

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, 0x80, 0x00, 0x00, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;
	case 0x11:
	{
		if (datalen!=2) return;

		const uint8_t pin_no = data[0];
		const uint8_t pin_val = data[1];
		pinMode(pin_no, OUTPUT);
		digitalWrite(pin_no, pin_val);

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, 0x81, 0x00, 0x00, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;
	case 0x12:
	{
		if (datalen!=1) return;

		const uint8_t pin_no = data[0];
		pinMode(pin_no, INPUT);
		const uint8_t val = digitalRead(pin_no);

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, 0x82, 0x01, pin_no, val, 0x00 +pin_no+ val/*checksum*/, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;

	case 0x20:
	{
		if (datalen!=sizeof(TFrameCMD_ADC_start_payload_t)) return;

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
		
		analogReference( adc_req.use_internal_refvolt ? INTERNAL : DEFAULT );

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, 0x90, 0x00, 0x00, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;

	case 0x21:
	{
		if (datalen!=sizeof(TFrameCMD_ADC_stop_payload_t)) return;

		num_active_ADC_channels = 0;

		// send answer back:
		const uint8_t rx[] = { FRAME_START_FLAG, 0x91, 0x00, 0x00, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;

	default:
	{
		// Error:
		const uint8_t rx[] = { FRAME_START_FLAG, 0xfe, 0x00, 0x00, FRAME_END_FLAG };
		Serial.write(rx,sizeof(rx));
	}
	break;
	};
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
				continue;
			}

			// 100% sure: we have a valid frame: dispatch it:
			process_command(opcode,datalen,data);

			reset_rx_buf();
		}
	}
}

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

void processPulseCounter()
{
	const unsigned long tnow = millis();
	if (tnow-PC_last_millis < PC_sampling_period_ms)
	return;

	PC_last_millis = tnow;

	uint16_t read_pulses;

	// ATOMIC READ:
	noInterrupts();
	read_pulses = PULSE_COUNTER;
	PULSE_COUNTER=0;
	interrupts();

	// send answer back:
	TFrame_PULSE_COUNTER_readings tx;
	tx.payload.timestamp_ms = millis();
	tx.payload.period_ms = PC_sampling_period_ms;
	tx.payload.pulse_counter = read_pulses;
	
	tx.calc_and_update_checksum();

	Serial.write((uint8_t*)&tx,sizeof(tx));
}

void loop()
{
	processIncommingPkts();
	processADCs();
	processPulseCounter();

}
