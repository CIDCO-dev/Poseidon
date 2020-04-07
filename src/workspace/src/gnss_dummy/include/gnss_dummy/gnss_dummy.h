#ifndef gnss_dummy
#define gnss_dummy

#include <math.h>  // for M_PI

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>

#define PI M_PI
#define R2D ((double)180/(double)PI)

class GNSS{
	private:
		ros::NodeHandle node;
		ros::Publisher gnssTopic;

		uint32_t sequenceNumber;

		double longitude;
		double latitude;
		
	public:
		GNSS(void){
			gnssTopic = node.advertise<sensor_msgs::NavSatFix>("fix", 1000);
		}
		
		
		void talk();
		double ellipsoidalHeight(int sequenceNumber);
               	void message(int sequenceNumber,double longitude,double latitude);
};


#endif
