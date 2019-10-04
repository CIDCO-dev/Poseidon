#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "std_msgs/String.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <iostream>
#include <fstream>
#include <string>

std::string outputFileName;
std::string outputFolder;
std::ofstream output;
std::string sep = ";";

//using namespace sensor_msgs;
using namespace message_filters;

void loggerCallback(const geometry_msgs::PoseStamped& gnss,
                    const geometry_msgs::PoseStamped& imu,
                    const geometry_msgs::PoseStamped& sonar)
{
  ROS_INFO("");
  ROS_INFO("-----------------------------------------");
//  ROS_INFO("[%lf];[%lf]", sonar.header.stamp, sonar);
  ROS_INFO("[%lf];[%lf];[%lf];[%lf];[%lf];[%lf];[%lf];[%lf];[%lf];[%lf];[%lf];", 
                    gnss.header.stamp, gnss.pose.position.x, gnss.pose.position.y, gnss.pose.position.z,
                    imu.header.stamp, imu.pose.orientation.w, imu.pose.orientation.x, imu.pose.orientation.y, imu.pose.orientation.z,
		    sonar.header.stamp, sonar);
  ROS_INFO("-----------------------------------------");

/*  output  << gnss.header.stamp
     << sep << gnss.pose.position.y
     << sep << gnss.pose.position.z
     << std::endl;*/
}

bool init(){
    output.open(outputFileName.c_str());

  if (!output) {
     std::cerr << "Could not open file: " << outputFileName << std::endl;
     return false;
  }

  output << std::fixed << std::setprecision(10);

  output  << "TimeStamp"
     << sep <<"Longitude"
     << sep << "Latitude"
     << sep << "Ellopsoidal Height"
     << std::endl;

  return true;
}

int main(int argc, char **argv)
{
  if(argc != 2){
    std::cout << "Missing output file path" << std::endl;
    return 1;
  }

  std::string folderPath(argv[1]);
  outputFolder = folderPath;
  outputFileName = folderPath + "/" + "testOutput.txt";

  if (init()){
     ros::init(argc, argv, "listener");

     ros::NodeHandle n;
     message_filters::Subscriber<geometry_msgs::PoseStamped> position_sub(n, "position", 1);
     message_filters::Subscriber<geometry_msgs::PoseStamped> orientation_sub(n, "pose", 1);
     message_filters::Subscriber<geometry_msgs::PoseStamped> depth_sub(n, "depth", 1);

     typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseStamped, geometry_msgs::PoseStamped> MySyncPolicy;
     Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), position_sub, orientation_sub, depth_sub);
     sync.registerCallback(boost::bind(&loggerCallback, _1, _2, _3));
     //ros::Subscriber sub = n.subscribe("depth", 1000, loggerCallback);
     ros::spin();
  }

  return 0;
}

#endif
