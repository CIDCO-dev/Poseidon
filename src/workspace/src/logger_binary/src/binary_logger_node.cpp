#include "BinaryLogger.hpp"

int main(int argc, char** argv){
	
	if(argc < 2){

		std::cout << "logger_binary, Missing output folder path" << std::endl;
		return 1;
	}
	
	try{
	
		ros::init(argc, argv, "logger_binary");

		std::string logPath ( argv[1] );
		
		BinaryLogger logger(logPath);
		logger.run();
	
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}
	
	return 0;
	
}
