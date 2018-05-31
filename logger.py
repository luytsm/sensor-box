#!/bin/python


import serial

addr = "/dev/ttyUSB0" ## serial port to read data from
baud = 9600 ## baud rate for instrument

serial_port =   serial.Serial(
                port = addr,\
                baudrate = baud,\
                parity=serial.PARITY_NONE,\
                stopbits=serial.STOPBITS_ONE,\
                bytesize=serial.EIGHTBITS,\
                timeout=None)


print("Connected to: " + serial_port.portstr)



filename = "sensor_data.txt"
file_out = open(filename, 'a')

try:
    while True:
        x = serial_port.readline()
        file_out.write(x.decode('ascii'))

except KeyboardInterrupt:
        pass

finally:
    file_out.close()
    serial_port.close()

    
