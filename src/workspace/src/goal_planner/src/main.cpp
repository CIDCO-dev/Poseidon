#ifndef MAIN_CPP
#define MAIN_CPP

#include <list>


#include "ros/ros.h"

#include "../../state_controller/State.h"

#include "../../utils/Goal.hpp"


// #include "raspberrypi_vitals/sysinfo.h"

// #include <iostream>
// #include <fstream>
// #include <string>

// #include "sys/statvfs.h"



using namespace std;

class GoalPlanner{

	private:
		
		//ROS handle
		ros::NodeHandle node;

		//Input topics
		ros::Subscriber stateTopic;

		// Output topics
		ros::Publisher destinationTopic;


		std::list<Goal> goals;


		// Position currentPosition;

	public:
		GoalPlanner(){


			// Advertise to topics: destination (to pilot)
			destinationTopic = node.advertise<goal_planner::Destination>("destination", 1000);

			// Subscribe to topics: state (from state_controller)
			stateTopic = node.subscribe("state", 1000, &GoalPlanner::stateCallback, this);



			
		}


		// Callback for state from state_controller
		void GoalPlanner::stateCallback( const state_controller::State& state ) {

			// Extract current position

			// Extract other information, e.g. raspberrypi_vitals?

		}


		void run(){
			ros::Rate loop_rate( 0.1 );

			while ( ros::ok() ){


				// Check state's raspberrypi_vitals?



				// If there is a current goal:
				// {

					// If goal is a destinaton and currentPosition from the state_contoller 
					// is close enough to the currentDestination
					// {

						// The destination goal is met

						// If the next goal in the list is a destination, publish this new destination
						// {

						// }

						// Else: publish a destination that will tell the motors to stop (or maintain the position?)
						// {

						// }


					// }


				// }


				// If no current goal, load goal from the list if any
				// {

				// }	

				ros::spinOnce();
				loop_rate.sleep();

			}

		}
};

int main(int argc,char** argv){

	ros::init(argc, argv, "goalPlanner");

	GoalPlanner goalPlanner;
	goalPlanner.run();
}

#endif