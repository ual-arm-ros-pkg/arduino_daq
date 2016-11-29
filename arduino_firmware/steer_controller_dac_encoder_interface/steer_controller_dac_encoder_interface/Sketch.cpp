/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#include <Wire.h>
#include <SPI.h>
//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// Pin mapping:
const int PIN_DAC_MAX5500_CS  = 9;
const int PIN_LED             =  13;


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


void setup()
{
	// start the SPI library:
	SPI.begin();

	digitalWrite(PIN_DAC_MAX5500_CS, HIGH);  // /CS=1
	pinMode(PIN_DAC_MAX5500_CS, OUTPUT);

	// Set all chip outputs to 0V
	mod_dac_max5500_send_spi_word(0x8000);

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
		const uint8_t rx[] = { 0x69, 0x80, 0x00, 0x00, 0x96 };
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
		const uint8_t rx[] = { 0x69, 0x81, 0x00, 0x00, 0x96 };
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
		const uint8_t rx[] = { 0x69, 0x81, 0x01, val, 0x00 + val, 0x96 };
		Serial.write(rx,sizeof(rx));
	}
	break;

	default:
	{
		// Error:
		const uint8_t rx[] = { 0x69, 0xfe, 0x00, 0x00, 0x96 };
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
			if (b!=0x69) {
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
			if (rx_buf[rx_buf_len-1]!=0x96) {
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


void loop()
{
	processIncommingPkts();
}
