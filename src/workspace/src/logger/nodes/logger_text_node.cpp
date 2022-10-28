#include "ros/ros.h"
#include "loggerText.h"

int main(int argc, char **argv)
{
	if(argc < 2){

		std::cout << "nlogger_text, Missing output folder path" << std::endl;
		return 1;
	}

	try{
		ros::init(argc, argv, "logger");

		ros::NodeHandle n;

		std::string outputFolder( argv[1] );

		LoggerText logger(outputFolder, ";");

		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
