#ifndef state_controller
#define state_controller

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PointStamped.h"
#include "state_controller/State.h"
#include "std_msgs/String.h"
#include "raspberrypi_vitals/sysinfo.h"

#include <ctime>
#include <iostream>
#include <string>
#include <sstream>
#include <stdint.h>
#include <stdlib.h>
#include <inttypes.h>

int old_fix_seq;




class StateController{
	public:
		state_controller::State state;
		StateController(){
			positionTopic = n.subscribe("fix", 1000, &StateController::gnssCallback,this);
			attitudeTopic = n.subscribe("pose", 1000, &StateController::imuCallback,this);
			sonarTopic    = n.subscribe("depth", 1000, &StateController::sonarCallback,this);
                        vitalsTopic   = n.subscribe("vitals", 1000, &StateController::vitalsCallback,this);
                        

                        stateTopic    = n.advertise<state_controller::State>("state", 1000);
                        
                        state.position.status.status = -1;
		}


		void gnssCallback(const sensor_msgs::NavSatFix& gnss);
		void imuCallback(const sensor_msgs::Imu& imu);
		void sonarCallback(const geometry_msgs::PointStamped& sonar);
		void vitalsCallback(const raspberrypi_vitals::sysinfo& vital);
		void stateUpdated();
		uint64_t buildTimeStamp(int sec, int nsec);

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
};

#endif
