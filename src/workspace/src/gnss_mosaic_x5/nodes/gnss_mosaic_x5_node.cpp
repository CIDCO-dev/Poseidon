#include "ros/ros.h"
#include "gnss_mosaic_x5/gnss_mosaic_x5.h"


int main(int argc,char** argv){
	ros::init(argc, argv, "mosaic-X5");

	if(argc != 3){
		std::cerr << "gnss_mosaic_x5 logPath serialDevice" << std::endl;
		exit(1);
	}

	std::string logPath (argv[1]);
	std::string serialPortPath (argv[2]);

	if (logPath.length()<2){
		ROS_INFO("Missing output log path\n");
    		return 1;
	}
	ROS_INFO("Writing sbf log to %s",logPath.c_str());
	if (serialPortPath.length()<2){
		ROS_INFO("Missing serial port path\n");
    		return 1;
	}
	ROS_INFO("Using serial port at %s",serialPortPath.c_str());

	MosaicX5 mosaicX5(logPath , serialPortPath);
	mosaicX5.run();

}
