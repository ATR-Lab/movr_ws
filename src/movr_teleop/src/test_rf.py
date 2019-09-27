import serial
import time

port = '/dev/ttyUSB0'
ser = serial.Serial(port, 115200, timeout=.1)
# ser.baudrate = 115200

while True:
    # ser.write("<c 120 100>".encode())
    # ser.write(b'hello')
    # ser.write(65)
    ser.write('<c 100, 120>'.encode('utf-8'))
    ser.write('<m Do you need a ride?>'.encode('utf-8'))
    time.sleep(1)

ser.close()