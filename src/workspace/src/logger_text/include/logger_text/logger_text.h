#ifndef logger_text
#define logger_text

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
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
void imuCallback(const sensor_msgs::Imu& imuMsgs);
void sonarCallback(const geometry_msgs::PointStamped& sonarMsgs);



#endif
