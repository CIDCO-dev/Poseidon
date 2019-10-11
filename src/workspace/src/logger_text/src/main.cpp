#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
#include <ctime>
#include <iostream>
#include <fstream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include "../../utils/ReaderWriter.hpp"

std::string outputGnssFile;
std::string outputImuFile;
std::string outputSonarFile;
std::string outputFolder;
std::ofstream outputGnss;
std::ofstream outputImu;
std::ofstream outputSonar;
std::string sep = ";";

Writer *writer;
Position pos;
Imu imu;
Sonar sonar;

uint64_t buildTimeStamp(int sec, int nsec);

void gnssCallback(const geometry_msgs::PoseStamped& gnss)
{
  pos.timeStamp = buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec);
  pos.x = gnss.pose.position.x;
  pos.y = gnss.pose.position.y;
  pos.z = gnss.pose.position.z;

  writer->writeGnss(pos);
}

void imuCallback(const geometry_msgs::PoseStamped& imuMsgs)
{
  imu.timeStamp = buildTimeStamp(imuMsgs.header.stamp.sec, imuMsgs.header.stamp.nsec);
  imu.w = imuMsgs.pose.orientation.w;
  imu.x = imuMsgs.pose.orientation.x;
  imu.y = imuMsgs.pose.orientation.y;
  imu.z = imuMsgs.pose.orientation.z;

  writer->writeImu(imu);
}

void sonarCallback(const geometry_msgs::PointStamped& sonarMsgs)
{
  sonar.timeStamp = buildTimeStamp(sonarMsgs.header.stamp.sec, sonarMsgs.header.stamp.nsec);
  sonar.depth = sonarMsgs.point.z;

  writer->writeSonar(sonar);
}

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

int main(int argc, char **argv)
{
  if(argc != 2){
    std::cout << "Missing output folder path" << std::endl;
    return 1;
  }

  std::string folderPath(argv[1]);
  outputFolder = folderPath;

  outputGnssFile = outputFolder + "/" + getStringDate() + "_gnss.txt";
  outputImuFile = outputFolder + + "/" + getStringDate() + "_imu.txt";
  outputSonarFile = outputFolder + "/" + getStringDate() + "_sonar.txt";

  writer = new Writer(outputGnssFile, outputImuFile, outputSonarFile, false);

  ros::init(argc, argv, "logger_text");

  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("position", 1000, gnssCallback);
  ros::Subscriber sub2 = n.subscribe("pose", 1000, imuCallback);
  ros::Subscriber sub3 = n.subscribe("depth", 1000, sonarCallback);

  ros::spin();

  return 0;
}

#endif
