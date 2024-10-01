#ifndef gnss_dummy
#define gnss_dummy

#include <math.h>  // for M_PI

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include <sstream>

#include "../../utils/Constants.hpp"

class GNSS{
	private:
		ros::NodeHandle node;
		ros::Publisher gnssTopic;

		uint32_t sequenceNumber = 0;

		double longitude;
		double latitude;

	public:
		GNSS();

		void talk(int status);
		double ellipsoidalHeight(uint32_t sequenceNumber);
        void message(uint32_t msgSequenceNumber,double longitude,double latitude, int status);
};


#endif
