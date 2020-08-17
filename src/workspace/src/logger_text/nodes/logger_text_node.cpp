#include "ros/ros.h"
#include "logger_text/logger_text.h"
#include "../../utils/timestamp.h"

int main(int argc, char **argv)
{
	if(argc < 2){

		std::cout << "nlogger_text, Missing output folder path" << std::endl;
		return 1;
	}

	try{
		ros::init(argc, argv, "logger_text");

		ros::NodeHandle n;

		std::string outputFolder( argv[1] );

		Writer writer(outputFolder);

		ros::Subscriber sub1 = n.subscribe("fix", 1000, &Writer::gnssCallback,&writer);
		ros::Subscriber sub2 = n.subscribe("pose", 1000, &Writer::imuCallback,&writer);
		ros::Subscriber sub3 = n.subscribe("depth", 1000, &Writer::sonarCallback,&writer);

		ros::ServiceServer getLoggingStatusService = n.advertiseService("get_logging_status", &Writer::getLoggingStatus,&writer);
		ros::ServiceServer toggleLoggingService    = n.advertiseService("toggle_logging", &Writer::toggleLogging,&writer);

		ros::spin();
	}
	catch(std::exception & e){
		ROS_ERROR("%s",e.what());
	}

	return 0;
}
