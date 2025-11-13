#!/usr/bin/env python3
import rospy
from i2c_controller_service.srv import i2c_controller_service, i2c_controller_serviceResponse


def handle_request(req):
    """Return a deterministic response for logger tests."""
    resp = i2c_controller_serviceResponse()
    resp.value = 0.0
    return resp


def main():
    rospy.init_node("mock_i2c_controller_service")
    rospy.Service("i2c_controller_service", i2c_controller_service, handle_request)
    rospy.spin()


if __name__ == "__main__":
    main()
