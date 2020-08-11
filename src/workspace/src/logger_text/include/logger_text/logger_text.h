#ifndef logger_text
#define logger_text

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "nav_msgs/Odometry.h"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <fstream>

class Writer{
public:
        Writer(std::string & gnssFilePath, std::string & imuFilePath, std::string & sonarFilePath, std::string separator=";");

        ~Writer();

        void init();

        void gnssCallback(const sensor_msgs::NavSatFix& gnss);

        void imuCallback(const nav_msgs::Odometry& odom);

        void sonarCallback(const geometry_msgs::PointStamped& sonar);

private:
        std::ofstream gnssOutputFile;
        std::ofstream imuOutputFile;
        std::ofstream sonarOutputFile;

        std::string gnssFilePath;
        std::string imuFilePath;
        std::string sonarFilePath;

        std::string separator;
};



#endif
