#ifndef logger_text
#define logger_text
#include "loggerBase.h"

class LoggerText : public LoggerBase{
	public:
		LoggerText(std::string & loggingPath, std::string separator=";");
		~LoggerText();
	
	private:
		/* log management methods */
		void init()override;
		void finalize()override;
		void rotate()override;
		
		/* topic callbacks */
		void gnssCallback(const sensor_msgs::NavSatFix& gnss)override;
		void imuCallback(const sensor_msgs::Imu& imu)override;
		void sonarCallback(const geometry_msgs::PointStamped& sonar)override;
		void lidarCallBack(const sensor_msgs::PointCloud2& lidar)override;
		
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
