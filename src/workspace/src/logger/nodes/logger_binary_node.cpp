#include "ros/ros.h"
#include "loggerBinary.h"

int main(int argc, char **argv)
{
	if(argc < 2){

		std::cout << "nlogger_text, Missing output folder path" << std::endl;
		return 1;
	}

	try{
		ros::init(argc, argv, "logger");

		ros::NodeHandle nh;

		std::string outputFolder( argv[1] );

		LoggerBinary logger(outputFolder);

		logger.advertiseTransferService(nh);

		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
