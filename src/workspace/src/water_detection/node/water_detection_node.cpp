#include "ros/ros.h"
#include "water_detection/water_detection.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "water detector");

	WaterDetector waterDetector(false);
	waterDetector.run();
}


