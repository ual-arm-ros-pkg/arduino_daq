# This message holds the ADC output for one sampling period
# for arduino_daq. Note that all 8 channels are always reported despite
# the actual number of active channels. E.g. if only one ADC channel is
# active, the actual reading will be in adc_data[0].

uint32    timestamp_ms   # Timestap of the microcontroller, in millisecs since boot
uint16[8] adc_data       # ADC raw reading. Multiply by VOLT_REF/1023 to obtain volts
