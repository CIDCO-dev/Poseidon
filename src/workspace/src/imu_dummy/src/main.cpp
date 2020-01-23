#ifndef MAIN_CPP
#define MAIN_CPP

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

		//Angles in radians
		void convertToQuaternion(double yaw, double pitch, double roll,sensor_msgs::Imu& pose){
			// Abbreviations for the various angular functions
			double cy = cos(yaw * 0.5);
			double sy = sin(yaw * 0.5);
			double cp = cos(pitch * 0.5);
			double sp = sin(pitch * 0.5);
			double cr = cos(roll * 0.5);
			double sr = sin(roll * 0.5);

			pose.orientation.w = cy * cp * cr + sy * sp * sr;
			pose.orientation.x = cy * cp * sr - sy * sp * cr;
			pose.orientation.y = sy * cp * sr + cy * sp * cr;
			pose.orientation.z = sy * cp * cr - cy * sp * sr;
		}

		void run(){
			ros::Rate loop_rate(200);

		        std::string msgString("yay");

		        while(ros::ok()){

                		sensor_msgs::Imu msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

				double heading = sin(sequenceNumber)*30;
				double pitch   = cos(sequenceNumber)*20;
				double roll    = sin(sequenceNumber*42+100)*10;

				convertToQuaternion(heading,pitch,roll,msg);

		                imuTopic.publish(msg);
                		ros::spinOnce();
                		loop_rate.sleep();
        		}
		}
};

int main(int argc,char** argv){
	ros::init(argc, argv, "imu");

	IMU imu;
	imu.run();
}


#endif
