#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"

class GNSS{
	private:
		ros::NodeHandle node;
		ros::Publisher gnssTopic;

		uint32_t sequenceNumber;

		double longitude;
		double latitude;
		double ellipsoidalHeight;

	public:
		GNSS(){
			gnssTopic = node.advertise<geometry_msgs::PoseStamped>("position", 1000);
		}

		void run(){
			ros::Rate loop_rate(200);

		        while(ros::ok()){

                		geometry_msgs::PoseStamped msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

				longitude += 0.0000001 * (rand() % 10 - 5);
				latitude  += 0.0000001 * (rand() % 10 - 5);

				double ellipsoidalHeight    = sin(sequenceNumber*42+100)*10;

				msg.pose.position.x=longitude;
				msg.pose.position.y=latitude;
				msg.pose.position.z=ellipsoidalHeight;

		                gnssTopic.publish(msg);
                		ros::spinOnce();
                		loop_rate.sleep();
        		}
		}
};

int main(int argc,char** argv){
	ros::init(argc, argv, "gnss");

	GNSS gnss;
	gnss.run();
}


#endif
