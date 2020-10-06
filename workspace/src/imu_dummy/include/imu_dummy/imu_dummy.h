#ifndef imu_dummy
#define imu_dummy

#include "ros/ros.h"
#include "nav_msgs/Odometry.h"

class IMU{
	private:
		ros::NodeHandle node;
		ros::Publisher imuTopic;

		uint32_t sequenceNumber;
	public:
		IMU(){
			imuTopic = node.advertise<nav_msgs::Odometry>("odom", 1000);
		}

		void run();
		void message(uint32_t sequenceNumber,double yaw, double pitch, double roll);
};

#endif
