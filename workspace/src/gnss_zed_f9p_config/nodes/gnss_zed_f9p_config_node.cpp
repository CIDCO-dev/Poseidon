#include "ros/ros.h"
#include "gnss_zed_f9p_config/gnss_zed_f9p_config.h"


int main(int argc,char** argv){
	ros::init(argc, argv, "zedf9p");
	std::string serialPortPath (argv[1]);
	if (serialPortPath.length()<2){
		ROS_INFO("Missing serial port path\n");
    		return 1;
	}
	ROS_INFO("Using serial port at %s",serialPortPath.c_str());
	ZEDF9P zedf9p(serialPortPath);
	zedf9p.run();

}


