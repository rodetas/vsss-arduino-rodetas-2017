#Codigo para receber mensagem enviada pelo Xbee

#! /usr/bin/python

# Import and init an XBee device
from xbee import XBee
import serial

ser = serial.Serial('/dev/ttyUSB0', 19200)

# Use an XBee 802.15.4 device
xbee = XBee(ser)

while True:
    try:
        response = xbee.wait_read_frame()
        value = ' '.join("{:02X}".format(ord(c)) for c in response['rf_data'])
        print str(int(value[0:2],16)) + " " + str(int(value[2:5], 16))
    except KeyboardInterrupt:
        break

ser.close()