#include "ros/ros.h"
#include "imu_null/imu_null.h"

int main(int argc,char** argv){
	ros::init(argc, argv, "imu");

	IMU imu;

	ros::Rate loop_rate( 200 );

	while(ros::ok()){
		imu.run();
		ros::spinOnce();
                loop_rate.sleep();
	}

	return 0;
}

