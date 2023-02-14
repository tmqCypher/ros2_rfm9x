# Descriptions from the RFM9x class source file at
# https://github.com/adafruit/Adafruit_CircuitPython_RFM9x/blob/main/adafruit_rfm9x.py

# Payload is limited to 252 bytes by the chip's FIFO size and appended headers.
uint8[<=252] payload
