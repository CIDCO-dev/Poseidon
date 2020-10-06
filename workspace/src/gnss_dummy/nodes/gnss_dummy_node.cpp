#include "ros/ros.h"
#include "gnss_dummy/gnss_dummy.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "gnss");
	GNSS gnss;
	ros::Rate loop_rate( 1 );
	while(ros::ok()){
		gnss.talk();
		ros::spinOnce();
                loop_rate.sleep();
	}
	return 0;
}

