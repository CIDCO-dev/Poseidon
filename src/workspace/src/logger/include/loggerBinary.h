#ifndef logger_text
#define logger_text
#include "loggerBase.h"
#include "types.h"

class LoggerBinary : public LoggerBase{
	public:
		LoggerBinary(std::string & loggingPath);
		~LoggerBinary();
		void finalize()override;
		
	private:
		/* log management methods */
		void init()override;
		void rotate()override;
		void saveSpeed(const nav_msgs::Odometry& speed) override;
		void saveVitals(const raspberrypi_vitals_msg::sysinfo& vitals) override;
		
		/* topic callbacks */
		void gnssCallback(const sensor_msgs::NavSatFix& gnss) override;
		void imuCallback(const sensor_msgs::Imu& imu)override;
		void sonarCallback(const geometry_msgs::PointStamped& sonar)override;
		void lidarCallBack(const sensor_msgs::PointCloud2& lidar)override;
				
		// log files
		std::string  outputFileName;
		std::ofstream outputFile;
		std::mutex fileLock;
		
		void readVitalsMsgFile()override;
		std::vector<std::string> vitalsValueName;
		std::vector<size_t> vitalsValueNameSize;


};
#endif
