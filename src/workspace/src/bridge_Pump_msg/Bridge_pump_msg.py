'''
# Copyright 2022 © Centre Interdisciplinaire de développement en Cartographie des Océans (CIDCO), Tous droits réservés
@Jacob Bellavance

'''
#serialPort = open("/dev/ttyUSB0", "w")
import sys
import serial
import os
import time


#serialPort.write("gpio write 1\r")


if (len(sys.argv) < 1):
	print("Use : python3 bridgePump.py <PORT>\nEg: python3 bridgePump.py COM1 ")
	sys.exit(0)
else: 
	portName = sys.argv[1]
	
#Open port
serialPort = serial.Serial(portName, 9200, timeout=1)

#Infinite loop
while(1):
	#send "gpio read" command
	serialPort.write(b'gpio read 0\r')
	#Verify if sensor is detecting water
	serialread = serialPort.read(25)
	#print(serialread[13])
	if serialread[13] == 49:
		print("ALERT : Water intrusion !\r")



