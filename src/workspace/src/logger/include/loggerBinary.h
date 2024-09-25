#ifndef logger_text
#define logger_text
#include "loggerBase.h"
#include "types.h"

class LoggerBinary : public LoggerBase{
	public:
		LoggerBinary(std::string & loggingPath);
		~LoggerBinary();
		
	private:
		/* log management methods */
		void init()override;
		void finalize()override;
		void rotate()override;
		
		/* topic callbacks */
		void gnssCallback(const sensor_msgs::NavSatFix& gnss) override;
		void imuCallback(const sensor_msgs::Imu& imu)override;
		void sonarCallback(const geometry_msgs::PointStamped& sonar)override;
		void lidarCallBack(const sensor_msgs::PointCloud2& lidar)override;
				
		// log files
		std::string  outputFileName;
		std::ofstream outputFile;
		std::mutex fileLock;


};
#endif
