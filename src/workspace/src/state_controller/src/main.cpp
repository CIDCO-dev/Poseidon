#ifndef MAIN_CPP
#define MAIN_CPP

#include "ros/ros.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
//#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/String.h"
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
		}

		void gnssCallback(const sensor_msgs::NavSatFix& gnss){
			lastPosition = &gnss;
                        
                        //TODO: check if position/ attitude / depth are not too far away in time
                        longitude = lastPosition->longitude;
                        latitude  = lastPosition->latitude;
                        altitude  = lastPosition->altitude;
                        memcpy(&covarianceMatrix,&(lastPosition->position_covariance),9*sizeof(double));
                        
                        stateUpdated();
		}

		void imuCallback(const sensor_msgs::Imu& imu){
			lastAttitude = &imu;
                        
//                        heading = 
                        
                        stateUpdated();
		}

		void sonarCallback(const geometry_msgs::PointStamped& sonar){
			lastDepth = &sonar;
                        
                        stateUpdated();
		}

		//TODO: add other sensor callbacks

		void stateUpdated(){
                    //TODO: kalmanize?
		}

	private:
		//State
		double      longitude;
		double      latitude;
		double      altitude;
                double      covarianceMatrix[9];
                
		double      pitch;
		double      heading;
		double      roll;

		double      depth;

		//Observed state
		const sensor_msgs::NavSatFix *   	lastPosition = NULL;
		const sensor_msgs::Imu *		lastAttitude = NULL;
		const geometry_msgs::PointStamped *	lastDepth    = NULL;

		ros::NodeHandle n;
		ros::Subscriber positionTopic;
		ros::Subscriber attitudeTopic;
		ros::Subscriber sonarTopic;
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

  StateController state;

  ros::spin();

  return 0;
}

#endif
