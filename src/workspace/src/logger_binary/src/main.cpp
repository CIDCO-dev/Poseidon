#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

struct Position {
  uint64_t timeStamp;
  double x;
  double y;
  double z;
};

struct Imu {
  uint64_t timeStamp;
  double w;
  double x;
  double y;
  double z;
};

struct Sonar {
  uint64_t timeStamp;
  double depth;
};

Position pos;
Imu imu;
Sonar sonar;

std::string outputGnssFile;
std::string outputImuFile;
std::string outputSonarFile;
std::string outputFolder;
std::ofstream outputGnss;
std::ofstream outputImu;
std::ofstream outputSonar;
std::string sep = ";";

void gnssCallback(const geometry_msgs::PoseStamped& gnss)
{
  std::ostringstream os;
  os << gnss.header.stamp.sec;
  os << gnss.header.stamp.nsec;
  std::string ntime = os.str();

  uint64_t timestamp;
  std::istringstream iss(ntime.c_str());
  iss >> timestamp;

  pos.timeStamp = timestamp;
  pos.x = gnss.pose.position.x;
  pos.y = gnss.pose.position.y;
  pos.z = gnss.pose.position.z;

  outputGnss.write((char *)&pos, sizeof(pos));
}

void imuCallback(const geometry_msgs::PoseStamped& imuMsgs)
{
  std::ostringstream os;
  os << imuMsgs.header.stamp.sec;
  os << imuMsgs.header.stamp.nsec;
  std::string ntime = os.str();

  uint64_t timestamp;
  std::istringstream iss(ntime.c_str());
  iss >> timestamp;

  imu.timeStamp = timestamp;
  imu.w = imuMsgs.pose.orientation.w;
  imu.x = imuMsgs.pose.orientation.x;
  imu.y = imuMsgs.pose.orientation.y;
  imu.z = imuMsgs.pose.orientation.z;

  outputImu.write((char *)&imu, sizeof(imu));
}

void sonarCallback(const geometry_msgs::PointStamped& sonarMsgs)
{
  std::ostringstream os;
  os << sonarMsgs.header.stamp.sec;
  os << sonarMsgs.header.stamp.nsec;
  std::string ntime = os.str();

  uint64_t timestamp;
  std::istringstream iss(ntime.c_str());
  iss >> timestamp;

  sonar.timeStamp = timestamp;
  sonar.depth = sonarMsgs.point.z;

  outputSonar.write((char *)&sonar, sizeof(sonar));
}

bool init(){
    outputGnss.open(outputGnssFile.c_str(), std::ios::binary);
    outputImu.open(outputImuFile.c_str(), std::ios::binary);
    outputSonar.open(outputSonarFile.c_str(), std::ios::binary);

  if (!outputGnss) {
     std::cerr << "Could not open file: " << outputGnssFile << std::endl;
     return false;
  }
  if (!outputImu) {
     std::cerr << "Could not open file: " << outputImuFile << std::endl;
     return false;
  }
  if (!outputSonar) {
     std::cerr << "Could not open file: " << outputSonarFile << std::endl;
     return false;
  }

  outputGnss << std::fixed << std::setprecision(10);
  outputImu << std::fixed << std::setprecision(10);
  outputSonar << std::fixed << std::setprecision(10);

  return true;
}

std::string getStringDate(){

  time_t rawtime;
  struct tm * timeinfo;
  char buffer[80];

  time (&rawtime);
  timeinfo = localtime(&rawtime);

  strftime(buffer,sizeof(buffer),"%Y_%m_%d",timeinfo);
  std::string date(buffer);

  return date;
}

int main(int argc, char **argv)
{
  if(argc != 2){
    std::cout << "Missing output file path" << std::endl;
    return 1;
  }

  std::string folderPath(argv[1]);
  outputFolder = folderPath;

  outputGnssFile = outputFolder + "/" + getStringDate() + "_gnss.bin";
  outputImuFile = outputFolder + + "/" + getStringDate() + "_imu.bin";
  outputSonarFile = outputFolder + "/" + getStringDate() + "_sonar.bin";

  if (init()){
     ros::init(argc, argv, "logger_binary");

     ros::NodeHandle n;

     ros::Subscriber sub1 = n.subscribe("position", 1000, gnssCallback);
     ros::Subscriber sub2 = n.subscribe("pose", 1000, imuCallback);
     ros::Subscriber sub3 = n.subscribe("depth", 1000, sonarCallback);

     ros::spin();
  }

  return 0;
}

#endif
