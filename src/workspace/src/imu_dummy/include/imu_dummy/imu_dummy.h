#ifndef imu_dummy
#define imu_dummy



#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class IMU{
	private:
		ros::NodeHandle node;
		ros::Publisher imuTopic;

		uint32_t sequenceNumber;
		
	public:
		IMU(){
			imuTopic = node.advertise<sensor_msgs::Imu>("pose", 1000);
		}
		
		
		void run();
		void convertToQuaternion(double yaw, double pitch, double roll,sensor_msgs::Imu& pose);
};

#endif
