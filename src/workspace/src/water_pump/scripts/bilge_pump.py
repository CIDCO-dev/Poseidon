#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from water_pump.msg import WaterIntrusion
from water_pump.msg import PumpControl
import serial
import sys



#serialPort = serial.Serial("/dev/ttyACM0", 9200, timeout=1)
#args = rospy.myargv(argv=sys.argv)
#print(args)
#serialPort = serial.Serial(args, 9200, timeout=1)

def pump_callBack(data):
	if data.ActivatePump == 1:
		serialPort.write(b'gpio set 1\r')
	else :
		serialPort.write(b'gpio clear 1\r') 

def waterAlertInspector(port):
	rospy.init_node('bilgePump',anonymous="true")
	pub = rospy.Publisher('waterInspection', WaterIntrusion, queue_size=5)
	rospy.Subscriber('pump_control', PumpControl, pump_callBack)
	rate = rospy.Rate(10)
	serialPort = serial.Serial(port, 9200, timeout=1)
	#rospy.spin()

	while not rospy.is_shutdown():
		#send "gpio read" command
		serialPort.write(b'gpio read 0\r')
		#Verify if sensor is detecting water
		serialread = serialPort.read(25)
		msg = WaterIntrusion()
		if serialread[13] == 49:
			rospy.loginfo("ALERT : Water intrusion !\r")
			msg.status = 1
			#pub.publish(msg)
		else :
			# No water intrusion !
			msg.status = 0
			#pub.publish(msg)
		msg.header.stamp = rospy.get_rostime()
		pub.publish(msg)
		rate.sleep()


if __name__ == "__main__":
	try:
		args = rospy.myargv(argv=sys.argv)
		port = args[1]
		if len(args) != 2 :
			print("Not good number of arguments : Try writing serial port/r/n for example : '/dev/ttyACM0'")
			sys.exit(1)
		waterAlertInspector(port)
	except rospy.ROSInterruptException:
		pass

