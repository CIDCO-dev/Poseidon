#ifndef logger_text
#define logger_text
#include "loggerBase.h"
#include <tf2_ros/transform_listener.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

class LoggerText : public LoggerBase{
	public:
		LoggerText(std::string & loggingPath, std::string separator=";");
		~LoggerText();
		/* log management methods */
		void init();
		void finalize();
		void rotate();
		void updateLoggingMode();
		
		/* topic callbacks */
		void gnssCallback(const sensor_msgs::NavSatFix& gnss);
		void imuCallback(const sensor_msgs::Imu& imu);
		void sonarCallback(const geometry_msgs::PointStamped& sonar);
		void speedCallback(const nav_msgs::Odometry& speed);
		void lidarCallBack(const sensor_msgs::PointCloud2& lidar);
	
	private:
		// log rotation
		std::mutex fileRotationMutex;
		ros::Time lastRotationTime;
		int logRotationIntervalSeconds = 60*60; //1h //TODO: make this a parameter 
		//TODO: make this a parameter?
		std::string tmpLoggingFolder = "/tmp";
		uint64_t lastGnssTimestamp = 0;
		uint64_t lastImuTimestamp  = 0;
		uint64_t lastSonarTimestamp= 0;
		uint64_t lastLidarTimestamp= 0;
		
		// log files
		std::string gnssFileName;
		std::string imuFileName;
		std::string sonarFileName;
		std::string lidarFileName;

		FILE * gnssOutputFile  = NULL;
		FILE * imuOutputFile   = NULL;
		FILE * sonarOutputFile = NULL;
		FILE * lidarOutputFile = NULL;
		
		// georef
		tf2_ros::Buffer buffer;
		tf2_ros::TransformListener transformListener;
		
		std::string separator;	
};
#endif
