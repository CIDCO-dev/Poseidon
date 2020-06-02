#include "imu_dummy/imu_dummy.h"


void IMU::convertToQuaternion(double yaw, double pitch, double roll,nav_msgs::Odometry& odom){
			// Abbreviations for the various angular functions
			double cy = cos(yaw * 0.5);
			double sy = sin(yaw * 0.5);
			double cp = cos(pitch * 0.5);
			double sp = sin(pitch * 0.5);
			double cr = cos(roll * 0.5);
			double sr = sin(roll * 0.5);

			odom.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;
			odom.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
			odom.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
			odom.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;
		}

		void IMU::run(){
			

		        

		        

                		nav_msgs::Odometry msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

				double heading = sin(sequenceNumber)*30;
				double pitch   = cos(sequenceNumber)*20;
				double roll    = sin(sequenceNumber*42+100)*10;

				convertToQuaternion(heading,pitch,roll,msg);

		                imuTopic.publish(msg);
                	
        	
		}
