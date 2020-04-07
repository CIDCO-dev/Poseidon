
#include "ros/ros.h"
#include "gnss_dummy/main.h"

#define PI M_PI
#define R2D ((double)180/(double)PI)


int main(int argc,char** argv){
	ros::init(argc, argv, "gnss");

	GNSS gnss;
	gnss.run();
}

