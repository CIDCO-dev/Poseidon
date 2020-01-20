#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"

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
			gnssTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 1000);
		}

		void run(){
			ros::Rate loop_rate(10);

		        while(ros::ok()){

                		sensor_msgs::NavSatFix msg;

				msg.header.seq=++sequenceNumber;
				msg.header.stamp=ros::Time::now();
				msg.header.stamp.nsec=0;

				longitude += 0.0000001 * (rand() % 10 - 5);
				latitude  += 0.0000001 * (rand() % 10 - 5);

				double ellipsoidalHeight    = sin(sequenceNumber*42+100)*10;

				msg.longitude=longitude;
				msg.latitude=latitude;
				msg.altitude=ellipsoidalHeight;

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
