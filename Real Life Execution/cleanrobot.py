from pycreate2 import Create2

#-------------------------------------------------------------------------------------------
#Roombas sometimes go into a weird mode or baudrate changes - this file resets everything
#-------------------------------------------------------------------------------------------


import serial

ser = serial.Serial('COM11', 19200)

ser.write(128)
ser.write(131)
ser.write(135)
ser.write(173)
