# This message holds the readings from absolute encoder for arduino_daq.

uint32    timestamp_ms   # Timestap of the microcontroller, in millisecs since boot
int32     encoder_value  # Signed position of the encoder sensor (in ticks, or encoder periods. x1, not x4, counts)
int8      encoder_status # Checkout datasheet
