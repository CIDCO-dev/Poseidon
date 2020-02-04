#ifndef MAIN_CPP
#define MAIN_CPP

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

class StateController{
	public:
		StateController(){
			positionTopic = n.subscribe("fix", 1000, &StateController::gnssCallback,this);
			attitudeTopic = n.subscribe("pose", 1000, &StateController::imuCallback,this);
			sonarTopic    = n.subscribe("depth", 1000, &StateController::sonarCallback,this);
                        vitalsTopic   = n.subscribe("vitals", 1000, &StateController::vitalsCallback,this);
                        

                        stateTopic    = n.advertise<state_controller::State>("state", 1000);
                        
                        state.position.status.status = -1;
		}

		void gnssCallback(const sensor_msgs::NavSatFix& gnss){
                    memcpy(&state.position,&gnss,sizeof(gnss));
                    stateUpdated();
		}

		void imuCallback(const sensor_msgs::Imu& imu){
                    memcpy(&state.attitude,&imu,sizeof(imu));
                    stateUpdated();
		}

		void sonarCallback(const geometry_msgs::PointStamped& sonar){
                    memcpy(&state.depth,&sonar,sizeof(sonar));
                    stateUpdated();
		}

		void vitalsCallback(const raspberrypi_vitals::sysinfo& vital){
                    memcpy(&state.vitals,&vital,sizeof(vital));
                    stateUpdated();
		}


		//TODO: add other sensor callbacks

		void stateUpdated(){
                    //TODO: kalmanize?
                    stateTopic.publish(state);
		}

	private:
            //State
            state_controller::State state;

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

uint64_t buildTimeStamp(int sec, int nsec){
  std::ostringstream os;
  os << sec;
  os << nsec;
  std::string ntime = os.str();

  uint64_t timestamp;
  std::istringstream iss(ntime.c_str());
  iss >> timestamp;

  return timestamp;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "state_controller");

  StateController stateControl;

  ros::spin();

  return 0;
}

#endif
