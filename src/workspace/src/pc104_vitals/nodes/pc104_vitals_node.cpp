#include "ros/ros.h"
#include "pc104_vitals/pc104_vitals.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "hbv");

	HBV hbv;
	hbv.run();
}


