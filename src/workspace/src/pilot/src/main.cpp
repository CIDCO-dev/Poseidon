#ifndef MAIN_CPP
#define MAIN_CPP



#include "ros/ros.h"

#include "state_controller/State.h"

#include "catarob_control/motor.h"

#include "goal_planner/Waypoint.h"

#include "pilot/Junk.h"

#include "../../utils/Structs.hpp"


// #include "raspberrypi_vitals/sysinfo.h"
// #include <iostream>
// #include <fstream>
// #include <string>
// #include "sys/statvfs.h"

using namespace std;

class Pilot{

	private:
		
		// ROS handle
		ros::NodeHandle node;

		//Input topics
		ros::Subscriber stateTopic;
		ros::Subscriber waypointTopic;

		// Output topics
		ros::Publisher motor_L;
		ros::Publisher motor_R;



		Position currentWaypoint;
		Position currentPosition;



	public:
		Pilot(){



			// Advertise to topics: motor/left and motor/right
			motor_L = node.advertise<catarob_control::motor>("motor/left", 1000);
			motor_R = node.advertise<catarob_control::motor>("motor/right", 1000);

			// Subscribe to topics: state (from state_controller), to know the current position
			stateTopic = node.subscribe("state", 1000, &Pilot::stateCallback, this);

			// Subscribe to topics: waypoint (from GoalPlanner)
			waypointTopic = node.subscribe("waypoint", 1000, &Pilot::waypointCallback, this);

			
		}


		// Callback for state from state_controller
		void stateCallback( const state_controller::State& state ) {

		}

		// Callback for waypoint
		void waypointCallback( const goal_planner::Waypoint& waypoint ) {

		}

		void run(){
			ros::Rate loop_rate( 0.01 );

			while ( ros::ok() ){


				// If there is a currentWaypoint
				//{ 
						// Based on the currentPosition, control the motors to get to the currentWaypoint

				// }

				ros::spinOnce();
				loop_rate.sleep();

			}

		}
};

int main(int argc,char** argv){

	ros::init(argc, argv, "pilot");

	Pilot pilot;
	pilot.run();
}

#endif
