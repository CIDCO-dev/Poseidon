#include "ros/ros.h"
#include "power_management/power_management.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "power_management");
	
	PowerManagement powerManagement;
	
	powerManagement.run();
}
