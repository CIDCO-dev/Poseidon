#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "geometry_msgs/PointStamped.h"

class Sonar{
	private:
		ros::NodeHandle node;
		ros::Publisher sonarTopic;

		uint32_t sequenceNumber;

	public:
		Sonar(){
			sonarTopic = node.advertise<geometry_msgs::PointStamped>("depth", 1000);
		}

		void run(){
			ros::Rate loop_rate(1);

		        while(ros::ok()){

                		geometry_msgs::PointStamped msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();

				msg.point.z = sin(sequenceNumber)*30;

		                sonarTopic.publish(msg);
                		ros::spinOnce();
                		loop_rate.sleep();
        		}
		}
};

int main(int argc,char** argv){
	ros::init(argc, argv, "sonar");

	Sonar sonar;
	sonar.run();
}


#endif
