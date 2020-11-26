/* bno055_i2c_calibration.cpp
 * Author: Guillaume Labbe-Morissette <guillaume.morissette@cidco.ca>
 *
 */

#include <imu_bno055/bno055_i2c_activity.h>
#include "watchdog/watchdog.h"
#include <csignal>

int main(int argc, char *argv[]) {

    ros::init(argc, argv, "bno055_node");

    ros::NodeHandle * nh = new ros::NodeHandle();
    if(!nh) {
        ROS_FATAL("Failed to initialize NodeHandle");
        ros::shutdown();
        return -1;
    }

    std::string calibrationFile;
    nh->param<std::string>("calibrationFile",calibrationFile,"/home/ubuntu/Poseidon/calibration.dat");

    ROS_INFO("Using %s",calibrationFile.c_str());

    ros::NodeHandle * nh_priv = new ros::NodeHandle("~");

    if(!nh_priv) {
        ROS_FATAL("Failed to initialize private NodeHandle");
        delete nh;
        ros::shutdown();
        return -2;
    }

    imu_bno055::BNO055I2CActivity* activity = new imu_bno055::BNO055I2CActivity(*nh, *nh_priv,calibrationFile);

    if(!activity) {
	ROS_FATAL("Error while creating BNO055 activity");
        return -3;
    }

    try{
	    if(!activity->start()) {
		ROS_FATAL("Error while starting activity");
        	return -4;
    	    }
	    activity->start();

	    activity->startCalibration();

	    ros::Rate rate(10);
	    while(ros::ok() && !activity->spinCalibrationOnce()) {
        	rate.sleep();
   	    }

	    activity->stop();
    }
    catch(std::exception &e){
	ROS_FATAL("Error: %s",e.what());
    }
	    delete activity;

    return 0;
}
