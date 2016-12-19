#pragma once

#include <stdlib.h>

#if !defined(__AVR_MEGA__)
#	pragma pack(push, 1) // exact fit - no padding
#endif

/*
START_FLAG   |  OPCODE  |  DATA_LEN   |   DATA      |    CHECKSUM    | END_FLAG |
  0x69          1 byte      1 byte       N bytes       =sum(data)       0x96

## Computer => controller
* 0x10: Set DAC value. DATA_LEN = 3
	* DATA[0]   = DAC index
	* DATA[1:2] = DAC value (0x0000-0xffff)  (MSByte first!)
* 0x11: Set GPIO pin. DATA_LEN = 2
	* DATA[0]   = Arduino-based pin number
	* DATA[1]   = 0/1
* 0x12: Read GPIO pin. DATA_LEN = 1
	* DATA[0]   = Arduino-based pin number

*/

#define FRAME_START_FLAG  0x69
#define FRAME_END_FLAG    0x96

template <typename Payload>
struct TBaseFrame
{
    const uint8_t  START_FLAG;
    const uint8_t  OPCODE;
    const uint8_t  DATALEN;
    // --------- Payload -------
    Payload  payload;
    // -------------------------
    uint8_t  CHECKSUM;
    const uint8_t  END_FLAG;

    // Defaults:
    TBaseFrame(uint8_t opcode) :
            START_FLAG(FRAME_START_FLAG),
            OPCODE(opcode),
            DATALEN(sizeof(Payload)),
            END_FLAG(FRAME_END_FLAG)
    {
    }

    void calc_and_update_checksum()
    {
        CHECKSUM = calc_checksum();    
    }
    
    uint8_t calc_checksum() const
    {
        const uint8_t len   = DATALEN; //reinterpret_cast<const uint8_t*>(ptr_frame)[2];
        const uint8_t *data = reinterpret_cast<const uint8_t*>(&payload);
        uint8_t ret =0;
        for (unsigned int i=0;i<len;i++) ret+=*data++;
        return ret;
    }
};

struct TFrameCMD_SetDAC_payload_t
{
    uint8_t  dac_index;
    uint8_t  dac_value_HI, dac_value_LO;
};
struct TFrameCMD_SetDAC : public TBaseFrame<TFrameCMD_SetDAC_payload_t>
{
    // Defaults:
    TFrameCMD_SetDAC() : TBaseFrame(0x10)
    {
    }
};

struct TFrameCMD_GPIO_output_payload_t
{
    uint8_t  pin_index;
    uint8_t  pin_value;;
};
struct TFrameCMD_GPIO_output : public TBaseFrame<TFrameCMD_GPIO_output_payload_t>
{
    // Defaults:
    TFrameCMD_GPIO_output() : TBaseFrame(0x11)
    {
    }
};


#if !defined(__AVR_MEGA__)
#	pragma pack(pop)
#endif
