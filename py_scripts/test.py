import serial
from time import sleep
import R2Protocol

try:
	s = serial.Serial("COM8", 9600, timeout=2)
	print("Connected")
except serial.SerialException:
	print("No serial port found")
	exit()
	
try:
	str = R2Protocol.encode(b"NUC", b"HEAD", b"", b"L0")
	print(str)
	s.write(str)
	print("sent")
	sleep(1)
	line = s.readline()
	print(line)
except serial.SerialTimeoutException:
	print("Serial port write timed out 1!")

sleep(1)

try:
	str = R2Protocol.encode(b"NUC", b"HEAD", b"", b"R0")
	print(str)
	s.write(str)
	print("sent")
	sleep(1)
	line = s.readline()
	print(line)
except serial.SerialTimeoutException:
	print("Serial port write timed out 1!")
