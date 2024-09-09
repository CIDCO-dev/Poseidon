#include "ros/ros.h"
#include "led_hydroball/hydroball_led.h"

int main(int argc, char **argv)
{
	try{
	
		ros::init(argc, argv, "led controller");
		LEDController ledController;
		ros::spin();
		
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
