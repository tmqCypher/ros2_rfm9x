# Descriptions from the RFM9x class source file at
# https://github.com/adafruit/Adafruit_CircuitPython_RFM9x/blob/main/adafruit_rfm9x.py

# The 4 byte header appended to transmissions
# to be compatible with the RadioHead library.
# The default address of this Node. (0-255).
# If not 255 (0xff) then only packets address to this node will be accepted.
# First byte of the RadioHead header.
uint8 node 255

# The default destination address for packet transmissions. (0-255).
# If 255 (0xff) then any receiving node should accept the packet.
# Second byte of the RadioHead header.
uint8 destination 255

# Automatically set to the sequence number when send_with_ack() used.
# Third byte of the RadioHead header.
uint8 identifier 0

# Upper 4 bits reserved for use by Reliable Datagram Mode.
# Lower 4 bits may be used to pass information.
# Fourth byte of the RadioHead header.
uint8 flags 0
