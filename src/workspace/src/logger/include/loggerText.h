#ifndef logger_text
#define logger_text
#include "loggerBase.h"

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
		
		// log files
		std::string gnssFileName;
		std::string imuFileName;
		std::string sonarFileName;
		std::string lidarFileName;

		FILE * gnssOutputFile  = NULL;
		FILE * imuOutputFile   = NULL;
		FILE * sonarOutputFile = NULL;
		FILE * lidarOutputFile = NULL;
		
		std::string separator;
};
#endif
