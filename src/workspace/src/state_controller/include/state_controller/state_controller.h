#ifndef state_controller
#define state_controller

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
#include "raspberrypi_vitals_msg/sysinfo.h"

#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>
#include <mutex>

#include "state_controller_msg/State.h"
#include "state_controller_msg/GetStateService.h"

class StateController{
	public:
		StateController(){
			positionTopic = n.subscribe("fix", 1000, &StateController::gnssCallback,this);
			attitudeTopic = n.subscribe("/imu/data", 1000, &StateController::imuCallback,this);
			sonarTopic    = n.subscribe("depth", 1000, &StateController::sonarCallback,this);
		    vitalsTopic   = n.subscribe("vitals", 1000, &StateController::vitalsCallback,this);
            stateTopic    = n.advertise<state_controller_msg::State>("state", 1000);
			getStateServiceServer = n.advertiseService("get_state",&StateController::getStateService,this);
      		state.position.status.status = -1;
		}

		void gnssCallback(const sensor_msgs::NavSatFix& gnss);
		void imuCallback(const sensor_msgs::Imu& imu);
		void sonarCallback(const geometry_msgs::PointStamped& sonar);
		void vitalsCallback(const raspberrypi_vitals_msg::sysinfo& vital);
		void stateUpdated();

		bool getStateService(state_controller_msg::GetStateService::Request & req,state_controller_msg::GetStateService::Response & res);

	private:
        //ROS handles
        ros::NodeHandle n;

        //Input topics
        ros::Subscriber positionTopic;
        ros::Subscriber attitudeTopic;
        ros::Subscriber sonarTopic;
	    ros::Subscriber vitalsTopic;

        //output topic
        ros::Publisher  stateTopic;

	//services
	ros::ServiceServer getStateServiceServer;

	std::mutex stateMtx;
	state_controller_msg::State state;
};

#endif
