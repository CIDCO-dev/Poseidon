#include "ros/ros.h"
#include "sonar_dummy/sonar_dummy.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "sonar");
	Sonar sonar;
	ros::Rate loop_rate( 1 );
	while(ros::ok()){
		sonar.run();
		ros::spinOnce();
                loop_rate.sleep();
	}
	return 0;
}

