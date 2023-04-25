#include "lidarFilters.h"

int main(int argc, char **argv)
{
	try{
		ros::init(argc, argv, "lidar_filtering");
		
		if(argc != 5){
			ROS_ERROR("Missing filter param");
			exit(1);
		}

		LidarFiltering filtering(std::stod(argv[1]), std::stod(argv[2]), std::stod(argv[3]), std::stod(argv[4]));

		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
