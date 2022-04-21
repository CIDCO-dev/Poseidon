#!/usr/bin/env python

import rospy
from bilge_pump_msg.msg import WaterIntrusion
from std_msgs.msg import String
import serial

def waterAlertInspector():
	rospy.init_node('bilgePump',anonymous="true")
	pub = rospy.Publisher('waterInspection', WaterIntrusion, queue_size=5)
	rate = rospy.Rate(10)
	serialPort = serial.Serial("/dev/ttyACM0", 9200, timeout=1)
	while not rospy.is_shutdown():
		#send "gpio read" command
		serialPort.write(b'gpio read 0\r')
		#Verify if sensor is detecting water
		serialread = serialPort.read(25)
		msg = WaterIntrusion()
		if serialread[13] == 49:
			rospy.loginfo("ALERT : Water intrusion !\r")
			WaterIntrusion.status = 1
		else :
			# No water intrusion !
			WaterIntrusion.status = 0
		#WaterIntrusion.header = rospy.get_time()
		pub.publish()
		rate.sleep()

if __name__ == "__main__":
	try:
		waterAlertInspector()
	except rospy.ROSInterruptException:
		pass

