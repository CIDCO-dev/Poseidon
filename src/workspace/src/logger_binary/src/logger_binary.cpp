#include "logger_binary/logger_binary.h"
Writer *writer;

uint64_t buildTimeStamp(int sec, int nsec){
  std::ostringstream os;
  os << sec;
  os << nsec;
  std::string ntime = os.str();

  uint64_t timestamp;
  std::istringstream iss(ntime.c_str());
  iss >> timestamp;

  return timestamp;
}

void gnssCallback(const sensor_msgs::NavSatFix& gnss)
{
  Position pos;

  pos.timeStamp = buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec);
  pos.x = gnss.longitude;
  pos.y = gnss.latitude;
  pos.z = gnss.altitude;

  writer->writeBinGnss(pos);
}

void imuCallback(const nav_msgs::Odometry& odom)
{
	Imu imu;

	imu.timeStamp = buildTimeStamp(odom.header.stamp.sec, odom.header.stamp.nsec);
	imu.x = odom.pose.pose.orientation.x;
	imu.y = odom.pose.pose.orientation.y;
	imu.z = odom.pose.pose.orientation.z;
	imu.w = odom.pose.pose.orientation.w;

	writer->writeImu(imu);
}

void sonarCallback(const geometry_msgs::PointStamped& sonarMsgs)
{
  Sonar sonar;
  sonar.timeStamp = buildTimeStamp(sonarMsgs.header.stamp.sec,
                                    sonarMsgs.header.stamp.nsec);
  sonar.depth = sonarMsgs.point.z;

  writer->writeBinSonar(sonar);
}
