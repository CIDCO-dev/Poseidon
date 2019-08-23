#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class IMU{
	private:
		ros::NodeHandle node;
		ros::Publisher imuTopic;

		uint32_t sequenceNumber;

	public:
		IMU(){
			imuTopic = node.advertise<geometry_msgs::PoseStamped>("imu", 1000);
		}


		void run(){
			ros::Rate loop_rate(200);

		        std::string msgString("yay");

		        while(ros::ok()){

                		geometry_msgs::PoseStamped msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

				msg.pose.orientation.x=1;
				msg.pose.orientation.y=2;
				msg.pose.orientation.z=3;
				msg.pose.orientation.w=4;

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
