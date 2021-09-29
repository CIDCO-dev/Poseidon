#include "ros/ros.h"
#include "ins_piksi/ins_piksi.h"


int main(int argc,char** argv){
	ros::init(argc, argv, "piksi");

	if(argc != 3) {
	    ROS_ERROR_STREAM("ins_piksi logPath serialDevice");
		exit(1);
	}

	std::string logPath (argv[1]);
	std::string serialPortPath (argv[2]);

	if (logPath.length()<2){
	    ROS_ERROR_STREAM("Missing output log path for piksi");
	    return 1;
	}
	ROS_INFO_STREAM("Writing piksi binary data to: " << logPath);

	if (serialPortPath.length()<2){
	    ROS_ERROR_STREAM("Missing serial port path for piksi");
	    return 1;
	}
	ROS_INFO_STREAM("Using serial port at: " << serialPortPath);

	Piksi piksi(logPath , serialPortPath);
	piksi.run();
}


