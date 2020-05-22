#include "ros/ros.h"
#include "catarob_control/catarob_control.h"
#include </home/ubuntu/WiringPi/wiringPi/wiringPiI2C.h>


int main(int argc,char** argv){
	ros::init(argc, argv, "catarob");
	CATAROB catarob;
	catarob.run();
	return 0;
}

