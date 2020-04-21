#include "logger_text/logger_text.h"

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

  writer->writeGnss(pos);
}

void imuCallback(const sensor_msgs::Imu& imuMsgs)
{
  Imu imu;
 
  imu.timeStamp = buildTimeStamp(imuMsgs.header.stamp.sec, imuMsgs.header.stamp.nsec);
  imu.w = imuMsgs.orientation.w;
  imu.x = imuMsgs.orientation.x;
  imu.y = imuMsgs.orientation.y;
  imu.z = imuMsgs.orientation.z;

  writer->writeImu(imu);
}

void sonarCallback(const geometry_msgs::PointStamped& sonarMsgs)
{
  Sonar sonar;
  
  sonar.timeStamp = buildTimeStamp(sonarMsgs.header.stamp.sec,
                                    sonarMsgs.header.stamp.nsec);
  sonar.depth = sonarMsgs.point.z;

  writer->writeSonar(sonar);
}
