#include "ros/ros.h"
#include "pilot/pilot.h"


int main(int argc,char** argv){

	ros::init(argc, argv, "pilot");

	Pilot pilot;
	pilot.run();
}


