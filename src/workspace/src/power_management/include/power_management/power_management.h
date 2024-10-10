#include <iostream>
#include "ros/ros.h"
#include "i2c_controller_service/i2c_controller_service.h"
#include "power_management_msg/batteryMsg.h"


class PowerManagement {
public:
	PowerManagement(){
		i2c_ctrl_service_client  = n.serviceClient<i2c_controller_service::i2c_controller_service>("i2c_controller_service");
		i2c_ctrl_service_client.waitForExistence();
	}

	~PowerManagement() {
		
	}

	
	
	void run(){
		ros::Rate loop_rate(1);
		while (ros::ok()) {
			
			i2c_controller_service::i2c_controller_service srv;
			srv.request.action2perform = "get_voltage";
			
			if(i2c_ctrl_service_client.call(srv)){
/*
				if(srv.response.value < Threshold){
					graceful_shutdown();
				}
*/
			}
			else{
				ROS_ERROR("PowerManagement::run get_voltage service call failed");
			}
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

private:
	ros::NodeHandle n;
	ros::ServiceClient i2c_ctrl_service_client;
	
	void graceful_shutdown(){
		if (system("sh /opt/Poseidon/src/workspace/src/power_management/include/power_management") == -1) {
			ROS_ERROR("Failed to stop ROS service.");
		}
	}
};

