#include "ros/ros.h"
#include "wifi_config/wifi_config.h"


int main(int argc, char **argv)
{
	try{
		ros::init(argc, argv, "wifi_config");

		ros::NodeHandle n;

		WifiConfig wifiConfig;

		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
