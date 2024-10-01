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
#include <filesystem>

//Logger service Poseidon
#include "logger_service/GetLoggingStatus.h"
#include "logger_service/ToggleLogging.h"
#include "logger_service/GetLoggingMode.h"
#include "logger_service/SetLoggingMode.h"

//Poseidon custom messages
#include "setting_msg/Setting.h"
#include "setting_msg/ConfigurationService.h"
#include "binary_stream_msg/Stream.h"
#include "raspberrypi_vitals_msg/sysinfo.h"
#include "i2c_controller_service/i2c_controller_service.h"

//Poseidon utils
#include "../../utils/timestamp.h"
#include "../../utils/QuaternionUtils.h"
#include "../../utils/Constants.hpp"
#include "../../utils/HttpClient.hpp"


//Boost lib
#include <boost/archive/iterators/base64_from_binary.hpp>
#include <boost/archive/iterators/transform_width.hpp>
#include <boost/beast/core.hpp>
#include <boost/beast/http.hpp>
#include <boost/beast/version.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/ip/tcp.hpp>
#include <boost/beast/ssl.hpp>
#include <boost/asio/ssl/error.hpp>
#include <boost/asio/ssl/stream.hpp>

//rapidjson
#include <rapidjson/document.h>
#include <rapidjson/prettywriter.h>

enum LoggingMode{Undefined, AlwaysOn, Manual, SpeedBased};

class LoggerBase{
	
	private:
		/* logging utils */
		void updateLogRotationInterval();
		void updateLoggingMode();
		
		/* callbacks */
		void configurationCallBack(const setting_msg::Setting &setting);
		void gnssBinStreamCallback(const binary_stream_msg::Stream& stream);
		void sonarBinStreamCallback(const binary_stream_msg::Stream& stream);
		void hddVitalsCallback(const raspberrypi_vitals_msg::sysinfo vitals);
		
		//Service callbacks
		bool getLoggingStatus(logger_service::GetLoggingStatus::Request & req,logger_service::GetLoggingStatus::Response & response);
		bool toggleLogging(logger_service::ToggleLogging::Request & request,logger_service::ToggleLogging::Response & response);
		bool getLoggingMode(logger_service::GetLoggingMode::Request &request, logger_service::GetLoggingMode::Response &response);
		bool setLoggingMode(logger_service::SetLoggingMode::Request &request, logger_service::SetLoggingMode::Response &response);
		
		/* Speed based logging */
		void updateSpeedThreshold();
		double getSpeedThreshold();
		void speedCallback(const nav_msgs::Odometry& speed);
		
		/* api transfert */
		std::string zip_to_base64(std::string zipPath);
		std::string create_json_str(std::string &base64Zip);
		bool send_job(std::string json);
		void updateApiTransferConfig();
		
		/* temporary */
		void reset_gnss_timer();
		void gnss_timer_callback(const ros::TimerEvent& event);
		ros::Timer noGnssTimer;
	
	public:
		LoggerBase(std::string & outputFolder);
		~LoggerBase();
		
	protected:
		void processGnssFix(const sensor_msgs::NavSatFix& gnss);
		
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
		
		
		/* log transfer */
		bool compress(std::string &zipFilename, std::vector<std::string> &filesVector);
		void transfer();
		bool can_reach_server();
		
		// ros
		ros::NodeHandle node;
		ros::ServiceClient configurationClient;
		ros::ServiceClient i2cControllerServiceClient;
		
		// logger
		std::string outputFolder;
		std::string separator;
		int loggingMode = AlwaysOn;
		std::mutex mtx;
		bool loggerEnabled = false;
		bool bootstrappedGnssTime = false;
		bool gnssFix = false;
		bool hddFreeSpaceOK = true;
		
		// log rotation
		std::mutex fileRotationMutex;
		ros::Time lastRotationTime;
		int logRotationIntervalSeconds;
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
		ros::Subscriber hddVitalsSubscriber ;
		
		ros::ServiceServer getLoggingStatusService ;
		ros::ServiceServer toggleLoggingService;
		
		ros::ServiceServer getLoggingModeService ;
		ros::ServiceServer setLoggingModeService;
		
		// raw gnss binary stream
		std::string  rawGnssFileName;
		std::ofstream rawGnssOutputFile;
		ros::Subscriber gnssStreamSubscriber;
		std::string fileExtensionForGpsDatagram;
		
		// raw sonar binary stream
		std::string  rawSonarFileName;
		std::ofstream rawSonarOutputFile;
		ros::Subscriber sonarStreamSubscriber;
		std::string fileExtensionForSonarDatagram;
		
		// transfer
		std::string host;
		std::string target;
		bool activatedTransfer;
		std::string apiKey;
		std::string port;
};
	
#endif
