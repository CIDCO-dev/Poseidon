#ifndef logger_base
#define logger_base

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"

#include "std_msgs/String.h"
#include <iostream>
//#include <string>
//#include <sstream>
//#include <stdint.h>
//#include <stdlib.h>
//#include <inttypes.h>
//#include <fstream>
#include <mutex>
//#include <cstdio>

#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

//Config service
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"


class LoggerBase{
	
	public:
		LoggerBase(std::string & outputFolder);
		~LoggerBase();
		
		/* log management methods */
		virtual void init()=0;
		virtual void finalize()=0;
		void rotate();
		void updateLoggingMode();
		
		/* topic callbacks */
		virtual void gnssCallback(const sensor_msgs::NavSatFix& gnss)=0;
		virtual void imuCallback(const sensor_msgs::Imu& imu)=0;
		virtual void sonarCallback(const geometry_msgs::PointStamped& sonar)=0;
		virtual void speedCallback(const nav_msgs::Odometry& speed)=0;
		virtual void lidarCallBack(const sensor_msgs::PointCloud2& lidar)=0;
		
		/* speed trigger methods */
		void updateSpeedThreshold();
		double getSpeedThreshold();

		void configurationCallBack(const setting_msg::Setting &setting);

		//Service callbacks
		bool getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response);

		bool toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response);

		bool getLoggingMode(logger_service::GetLoggingMode::Request &request, logger_service::GetLoggingMode::Response &response);

		bool setLoggingMode(logger_service::SetLoggingMode::Request &request, logger_service::SetLoggingMode::Response &response);
		
	protected:
		// ros
		ros::NodeHandle node;
		ros::ServiceClient	configurationClient;
		
		// logger
		std::string outputFolder;
    	std::string separator;
		int loggingMode = 1;
		std::mutex mtx;
		bool loggerEnabled = false;
		bool bootstrappedGnssTime = false;

		// Speed-triggered logging mode 
		std::list<double> kmhSpeedList;
		double averageSpeed = 0;
		double speedThresholdKmh = 0.0;
		double defaultSpeedThreshold = 5.0;
		
		ros::Subscriber gnssSubscriber ;
		ros::Subscriber imuSubscriber ;
		ros::Subscriber depthSubscriber;
		ros::Subscriber speedSubscriber ;
		ros::Subscriber configurationSubscriber;
		ros::Subscriber lidarSubscriber ;
		
		ros::ServiceServer getLoggingStatusService ;
		ros::ServiceServer toggleLoggingService;
		
		ros::ServiceServer getLoggingModeService ;
		ros::ServiceServer setLoggingModeService;
};
	
#endif
