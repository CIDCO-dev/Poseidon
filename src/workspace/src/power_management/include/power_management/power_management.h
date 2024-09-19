#include <iostream>
#include "ros/ros.h"
#include "i2c_controller_service/i2c_controller_service.h"
#include "power_management_msg/batteryMsg.h"


class PowerManagement {
public:
	PowerManagement(){
		i2c_ctrl_service_client  = n.serviceClient<i2c_controller_service::i2c_controller_service>("i2c_controller_service");
		powerManagementTopic = n.advertise<power_management_msg::batteryMsg>("batteryStatus", 1000);
	}

	~PowerManagement() {
		
	}

	
	
	void run(){
		ros::Rate loop_rate(1);
		while (ros::ok()) {
			
			i2c_controller_service::i2c_controller_service srv;
			
			srv.request.action2perform = "get_voltage";
			
			if(i2c_ctrl_service_client.call(srv)){
				power_management_msg::batteryMsg msg;
				msg.voltage = srv.response.value;
				//ROS_INFO_STREAM("PowerManagement voltage call: " << msg.voltage);
				powerManagementTopic.publish(msg);
				
			}
			
			ros::spinOnce();
			loop_rate.sleep();
		}
	}

private:
	ros::NodeHandle n;
	ros::ServiceClient i2c_ctrl_service_client;
	ros::Publisher powerManagementTopic;
};

