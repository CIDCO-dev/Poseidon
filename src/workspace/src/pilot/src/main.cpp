#ifndef MAIN_CPP
#define MAIN_CPP



#include "ros/ros.h"

#include "../../state_controller/State.h"


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
		ros::Subscriber destinationTopic;

		// Output topics
		ros::Publisher motor_L;
		ros::Publisher motor_R;



		Position currentDestination;
		Position currentPosition;



	public:
		Pilot(){



			// Advertise to topics: motor/left and motor/right
			motor_L = node.advertise<catarob_control::motorL>("motor/left", 1000);
			motor_R = node.advertise<catarob_control::motorR>("motor/right", 1000);

			// Subscribe to topics: state (from state_controller), to know the current position
			stateTopic = node.subscribe("state", 1000, &Pilot::stateCallback, this);

			// Subscribe to topics: destination (from GoalPlanner)
			destinationTopic = node.subscribe("destination", 1000, &Pilot::destinationCallback, this);

			
		}


		// Callback for state from state_controller
		void Pilot::stateCallback( const state_controller::State& state ) {

		}

		// Callback for destination
		void Pilot::destinationCallback( const goal_planner::Destination& destination ) {

		}

		void run(){
			ros::Rate loop_rate( 0.01 );

			while ( ros::ok() ){


				// If there is a destination
				//{ 
						// Based on the currentPosition, control the motors to get to the currentDestination

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