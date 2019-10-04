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
  outputGnss << gnss.header.stamp.toSec()
     << sep << gnss.pose.position.x
     << sep << gnss.pose.position.y
     << sep << gnss.pose.position.z
     << std::endl;
}

void imuCallback(const geometry_msgs::PoseStamped& imu)
{
  outputImu << imu.header.stamp.toSec()
     << sep << imu.pose.orientation.w
     << sep << imu.pose.orientation.x
     << sep << imu.pose.orientation.y
     << sep << imu.pose.orientation.z
     << std::endl;
}

void sonarCallback(const geometry_msgs::PointStamped& sonar)
{
  outputSonar << sonar.header.stamp.toSec()
     << sep << sonar.point.z
     << std::endl; 
}

bool init(){
    outputGnss.open(outputGnssFile.c_str());
    outputImu.open(outputImuFile.c_str());
    outputSonar.open(outputSonarFile.c_str());

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

  outputGnss << "TimeStamp"
     << sep << "Longitude"
     << sep << "Latitude"
     << sep << "Ellopsoidal Height"
     << std::endl;

  outputImu << "TimeStamp"
     << sep << "Orientation X"
     << sep << "Orientation Y"
     << sep << "Orientation Z"
     << sep << "Orientation W"
     << std::endl;

  outputSonar << "TimeStamp"
     << sep << "Depth"
     << std::endl;

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

  outputGnssFile = outputFolder + "/" + getStringDate() + "_gnss.txt";
  outputImuFile = outputFolder + + "/" + getStringDate() + "_imu.txt";
  outputSonarFile = outputFolder + "/" + getStringDate() + "_sonar.txt";

  if (init()){
     ros::init(argc, argv, "logger_dummy");

     ros::NodeHandle n;

     ros::Subscriber sub1 = n.subscribe("position", 1000, gnssCallback);
     ros::Subscriber sub2 = n.subscribe("pose", 1000, imuCallback);
     ros::Subscriber sub3 = n.subscribe("depth", 1000, sonarCallback);

     ros::spin();
  }

  return 0;
}

#endif
