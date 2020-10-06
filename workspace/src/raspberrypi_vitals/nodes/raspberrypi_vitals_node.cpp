#include "ros/ros.h"
#include "raspberrypi_vitals/raspberrypi_vitals.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "hbv");

	HBV hbv;
	hbv.run();
}


