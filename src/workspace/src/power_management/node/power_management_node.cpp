#include "ros/ros.h"
#include "power_management/power_management.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "power_management");
	
	try{
		PowerManagement pwrmngmt("/dev/i2c-1");
	}
	catch (const std::exception &e) {
		ROS_ERROR_STREAM("Error: " << e.what());
	}
	
	pwrmngmt.run();
}
