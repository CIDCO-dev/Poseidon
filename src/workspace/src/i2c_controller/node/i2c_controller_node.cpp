#include "ros/ros.h"
#include "i2c_controller/i2c_controller.h"


int main(int argc, char **argv){
	
	double boardVersion = 2.0;
	
	if(argc < 3){
		ROS_ERROR("i2c_controller_node, Missing board version parameter in launch file");
	}
	else if(argc == 4){ // ROS is adding 2 param to each node
		
		try{
			boardVersion = std::stod(argv[1]);
		}
		catch(std::exception & e){
			ROS_ERROR("%s",e.what());
			return 1;
		}
	}

	try{
		
		ros::init(argc, argv, "i2c controller");
		
		I2cController controller(boardVersion);
		
		ros::spin();
		
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
		return 1;
	}

	return 0;
}
