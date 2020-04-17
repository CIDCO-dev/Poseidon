#ifndef sonar_dummy
#define sonar_dummy



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
		
		
		void run();
};

#endif
