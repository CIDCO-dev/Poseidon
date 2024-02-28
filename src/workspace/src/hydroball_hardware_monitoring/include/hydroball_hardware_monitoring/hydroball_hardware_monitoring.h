#pragma once

#include "ros/ros.h"
#include <iostream>
#include <fstream>
#include <string>
#include <hardware_monitoring_msg/hardware_monitoring.h>

class HydroballHardwareMonitor{
	private:
		
		ros::NodeHandle node;
		ros::Publisher monitoringMsgPublisher;

	public:
		HydroballHardwareMonitor(){
			monitoringMsgPublisher = node.advertise<hardware_monitoring_msg::hardware_monitoring>("hb_monitor", 1000);
			
		}
		
		void run(){
			ros::Rate loop_rate(1);

			while(ros::ok()){
				hardware_monitoring_msg::hardware_monitoring msg;
				
				//TODO
				
				monitoringMsgPublisher.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();
			}
		}
};
