// logger_text

#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include "../../utils/ReaderWriter.hpp"


// It would be possible to have the callback functions
// be part of class Writer so that a global variable Writer *writer
// would not be required
// http://docs.ros.org/melodic/api/roscpp/html/classros_1_1NodeHandle.html#a317fe4c05919e0bf3fb5162ccb2f7c28

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

void gnssCallback(const geometry_msgs::PoseStamped& gnss)
{
  Position pos;

  pos.timeStamp = buildTimeStamp(gnss.header.stamp.sec, gnss.header.stamp.nsec);
  pos.x = gnss.pose.position.x;
  pos.y = gnss.pose.position.y;
  pos.z = gnss.pose.position.z;

  writer->writeGnss(pos);
}

void imuCallback(const geometry_msgs::PoseStamped& imuMsgs)
{
  Imu imu;

  imu.timeStamp = buildTimeStamp(imuMsgs.header.stamp.sec, imuMsgs.header.stamp.nsec);
  imu.w = imuMsgs.pose.orientation.w;
  imu.x = imuMsgs.pose.orientation.x;
  imu.y = imuMsgs.pose.orientation.y;
  imu.z = imuMsgs.pose.orientation.z;

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


int main(int argc, char **argv)
{

//   std::cout << "\nlogger_text, argc: " << argc << "\n" << std::endl;

//   for ( int count = 0; count < argc; count++)
//     std::cout << count << ", '" << argv[ count ] << "'" << std::endl;

//   std::cout << "\n" << std::endl;

  if(argc < 2){

 //    std::cout << "\nif(argc < 2), argc: " << argc << std::endl;

    std::cout << "nlogger_text, Missing output folder path" << std::endl;
    return 1;
  }


  std::string outputFolder( argv[1] );

  std::string outputGnssFile = outputFolder + "/" 
                                + getStringDate() + "_gnss.txt";
  std::string outputImuFile = outputFolder + + "/" 
                                + getStringDate() + "_imu.txt";
  std::string outputSonarFile = outputFolder + "/" 
                                + getStringDate() + "_sonar.txt";

  writer = new Writer(outputGnssFile, outputImuFile, outputSonarFile, false);

  if ( writer->getSetupOK() == false ) {
    std::cout << "logger_text, could not setup the writer" << std::endl;
    return 1;
  }

  ros::init(argc, argv, "logger_text");

  ros::NodeHandle n;

  ros::Subscriber sub1 = n.subscribe("position", 1000, gnssCallback);
  ros::Subscriber sub2 = n.subscribe("pose", 1000, imuCallback);
  ros::Subscriber sub3 = n.subscribe("depth", 1000, sonarCallback);

  ros::spin();

  return 0;
}

#endif
