#include "ros/ros.h"
#include "i2c_controller/i2c_controller.h"


int main(int argc, char **argv)
{
	try{
	
		ros::init(argc, argv, "i2c controller");
		
		I2cController controller;
		
		ros::spin();
		
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
