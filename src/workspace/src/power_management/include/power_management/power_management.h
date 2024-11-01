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

	~PowerManagement() {}

	void run(){
		ros::Rate loop_rate(1);
		while (ros::ok()) {
			
			i2c_controller_service::i2c_controller_service srv;
			srv.request.action2perform = "get_voltage";
			
			if(i2c_ctrl_service_client.call(srv)){
				
				/*
					vital node will trigger a critical warning at 11.3V and a warning at 11.9v
					In the case scenario that by accident this node is in the launfile of a an equippement
					that is not eqquipped with a newer board version, the graceful shutdown should not be triggered
					by the i2cController::read_chip_v0 function
				*/
				
				if(srv.response.value < 11.0 && srv.response.value > 0.0){
					ROS_WARN("Launching shutdown procedure");
					graceful_shutdown();
				}
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
	
	/*
		by calling to stop the service
		the service will launch the stopping procedure
		the service procedures are defined here : /etc/systemd/system/ros.service
		the install script Poseidon/intstall/stages/4-x64.sh is creating the service
	*/
	
	void graceful_shutdown(){
		if (system("systemctl stop ros") == -1) {
			ROS_ERROR("Failed to stop ros service.");
		}
		
	}
};

