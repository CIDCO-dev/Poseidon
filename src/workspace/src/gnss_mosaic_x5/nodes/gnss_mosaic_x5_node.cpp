#include "ros/ros.h"
#include "gnss_zed_f9p/gnss_zed_f9p.h"


int main(int argc,char** argv){
	ros::init(argc, argv, "zedf9p");

	if(argc != 3){
		std::cerr << "gnss_zed_f9p logPath serialDevice" << std::endl;
		exit(1);
	}

	std::string logPath (argv[1]);
	std::string serialPortPath (argv[2]);

	if (logPath.length()<2){
		ROS_INFO("Missing output log path\n");
    		return 1;
	}
	ROS_INFO("Writing UBX log to %s",logPath.c_str());
	if (serialPortPath.length()<2){
		ROS_INFO("Missing serial port path\n");
    		return 1;
	}
	ROS_INFO("Using serial port at %s",serialPortPath.c_str());

	ZEDF9P zedf9p(logPath , serialPortPath);
	zedf9p.run();

}