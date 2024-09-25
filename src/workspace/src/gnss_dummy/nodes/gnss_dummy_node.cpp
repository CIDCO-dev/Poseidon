#include "ros/ros.h"
#include "gnss_dummy/gnss_dummy.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "gnss");
	GNSS gnss;
	ros::Rate loop_rate(1);
	
	int gnssFixChange = 0;
	
	while(ros::ok()){
		if(gnssFixChange < 30){
			gnss.talk(1);  // Send 1 signal
			gnssFixChange++;  // Increment count
		}
		else if(gnssFixChange >= 30 && gnssFixChange < 60){
			gnss.talk(-1);  // Send -1 signal
			gnssFixChange++;  // Increment count
		}
		else if(gnssFixChange >= 60){
			gnssFixChange = 0;  // Reset count
		}
		
		
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

