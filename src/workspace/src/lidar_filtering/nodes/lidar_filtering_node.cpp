#include "ros/ros.h"
#include "lidarFilters.h"

int main(int argc, char **argv)
{
	try{
		ros::init(argc, argv, "lidar_filtering");

		LidarFiltering filtering;

		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
