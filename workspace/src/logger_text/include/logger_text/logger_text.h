#ifndef logger_text
#define logger_text

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"

#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <fstream>
#include <mutex>
#include <cstdio>

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"

class Writer{
public:
        Writer(std::string & outputFolder, std::string separator=";");

        ~Writer();

        void init();
	void finalize();

        void gnssCallback(const sensor_msgs::NavSatFix& gnss);

        void imuCallback(const nav_msgs::Odometry& odom);

        void sonarCallback(const geometry_msgs::PointStamped& sonar);


	//Service callbacks
	bool getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response);

	bool toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response);

private:
	std::mutex mtx;
	bool loggerEnabled = false;
	bool bootstrappedGnssTime = false;

        FILE * gnssOutputFile  = NULL;
        FILE * imuOutputFile   = NULL;
        FILE * sonarOutputFile = NULL;

        std::string outputFolder;

        std::string separator;

	tf2_ros::Buffer buffer;
	tf2_ros::TransformListener transformListener;
};



#endif
