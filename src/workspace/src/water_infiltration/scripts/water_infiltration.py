#!/usr/bin/env python

import rospy
from std_msgs.msg import Float32
import gpiod
from i2c_controller_service.srv import i2c_controller_service

def water_infiltration_node():
	# Initialize the node
	i2c_service = rospy.ServiceProxy('/i2c_controller_service', i2c_controller_service)
	rospy.init_node('water_infiltration')
	chip = gpiod.Chip('gpiochip0')  # 'gpiochip0' is typically the default, but this can vary.
	line = chip.get_line(18) # set pin

	# Request the line as an input
	line.request(consumer="water_infiltration", type=gpiod.LINE_REQ_DIR_IN)

	rate = rospy.Rate(1)  # 1 Hz
	while not rospy.is_shutdown():
		print(line.get_value())
		if(line.get_value() == 1):
			print("ok 0")
			response = i2c_service("led_error")
			print("ok 1")
		rate.sleep()

if __name__ == '__main__':
	try:
		rospy.wait_for_service('/i2c_controller_service')
		water_infiltration_node()  # Run the node
	except rospy.ROSInterruptException:
		pass
