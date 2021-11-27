#ifndef logger_text
#define logger_text

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

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

//Config service
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"

class Writer{
public:
    Writer(std::string & outputFolder, std::string separator=";");

    ~Writer();

    void init();
	void finalize();

    void gnssCallback(const sensor_msgs::NavSatFix& gnss);

    void imuCallback(const sensor_msgs::Imu& imu);

    void sonarCallback(const geometry_msgs::PointStamped& sonar);
    
    void speedCallback(const nav_msgs::Odometry& speed);

	void getSpeedThresholdConfig();
	
	double getSpeedThreshold();

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

	uint64_t lastGnssTimestamp = 0;
	uint64_t lastImuTimestamp  = 0;
	uint64_t lastSonarTimestamp= 0;
	
	std::list<double> kmh_Speed_list;
	double average_speed = 0;
	double speedThresholdKmh = 0.0;
	ros::NodeHandle node;
	ros::ServiceClient	configurationClient;
	double defaultSpeedThreshold = 5.0;
};



#endif
