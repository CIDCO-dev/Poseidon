#ifndef logger_base
#define logger_base

//Ros
#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "geometry_msgs/PointStamped.h"
#include "geometry_msgs/TransformStamped.h"
#include "nav_msgs/Odometry.h"
#include "std_msgs/String.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


//C++ std lib
#include <iostream>
#include <mutex>
#include <thread>
#include <numeric>
#include <fstream>

//Logger service Poseidon
#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

//Poseidon custom messages
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"
#include "binary_stream_msg/Stream.h"

//Poseidon utils
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.hpp"

class LoggerBase{
	
	public:
		LoggerBase(std::string & outputFolder);
		~LoggerBase();
		
		/* log management methods */
		virtual void init()=0;
		virtual void finalize()=0;
		virtual void rotate()=0;
		
		/* Tranformers */
		void imuTransform(const sensor_msgs::Imu& imu, double & roll , double & pitch, double & heading);
		
		/* topic callbacks */
		virtual void gnssCallback(const sensor_msgs::NavSatFix& gnss)=0;
		virtual void imuCallback(const sensor_msgs::Imu& imu)=0;
		virtual void sonarCallback(const geometry_msgs::PointStamped& sonar)=0;
		virtual void lidarCallBack(const sensor_msgs::PointCloud2& lidar)=0;
		void configurationCallBack(const setting_msg::Setting &setting);
		virtual void gnssBinStreamCallback(const binary_stream_msg::Stream& stream)=0;
		
		/* Speed based logging */
		void updateSpeedThreshold();
		double getSpeedThreshold();
		void speedCallback(const nav_msgs::Odometry& speed);
		
		//Service callbacks
		bool getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response);
		bool toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response);
		bool getLoggingMode(logger_service::GetLoggingMode::Request &request, logger_service::GetLoggingMode::Response &response);
		bool setLoggingMode(logger_service::SetLoggingMode::Request &request, logger_service::SetLoggingMode::Response &response);
		void updateLoggingMode();
		
		/* log transfer */
		virtual void compress(){};
		/*
		void transfer();
		std::string zip_to_base64(std::string zipPath);
		std::string create_json_str(std::string zipFilename, std::string base64Zip);
		bool send_job(std::string json);
		bool can_reach_server();
		*/
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
		
		// log rotation
		std::mutex fileRotationMutex;
		ros::Time lastRotationTime;
		int logRotationIntervalSeconds = 60*60; //1h //TODO: make this a parameter 
		std::string tmpLoggingFolder = "/tmp"; //TODO: make this a parameter?
		uint64_t lastGnssTimestamp =  0;
		uint64_t lastImuTimestamp  =  0;
		uint64_t lastSonarTimestamp = 0;
		uint64_t lastLidarTimestamp = 0;

		// Speed-triggered logging mode 
		std::list<double> kmhSpeedList;
		double averageSpeed = 0;
		double speedThresholdKmh = 0.0;
		double defaultSpeedThreshold = 5.0;
		
		// Imu transform
		tf2_ros::Buffer buffer;
		tf2_ros::TransformListener transformListener;
		
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
		
		// raw gnss binary stream
		std::string  rawGnssFileName;
        std::ofstream rawGnssoutputFile;
        ros::Subscriber streamSubscriber;
};
	
#endif
