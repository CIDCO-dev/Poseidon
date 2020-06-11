#ifndef logger_binary
#define logger_binary

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseStamped.h"
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

void gnssCallback(const sensor_msgs::NavSatFix& gnss);
void imuCallback(const nav_msgs::Odometry& imuMsgs);
void sonarCallback(const geometry_msgs::PointStamped& sonarMsgs);



#endif
