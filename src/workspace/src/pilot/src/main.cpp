#ifndef MAIN_CPP
#define MAIN_CPP

#include <iostream>

#include "ros/ros.h"

#include "state_controller/State.h"

#include "catarob_control/motor.h"

#include "goal_planner/Waypoint.h"

// #include "../../utils/Structs.hpp"

#include "../../utils/Waypoint.hpp"

#include "../../utils/BoolWithMutex.hpp"



// #include "raspberrypi_vitals/sysinfo.h"
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



		//Waypoint currentWaypoint;
		//Waypoint currentPosition;


		double waypointLatitude;
        double waypointLongitude;

		double currentPositionLatitude;
        double currentPositionLongitude;

        BoolWithMutex accessingWaypoint;
        BoolWithMutex accessingCurrentPosition;
    





	public:
		Pilot()
         :  accessingWaypoint( false ),
            accessingCurrentPosition( false )
{



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

           // std::cout << "Pilot::waypointCallback\n"
           //    << "  waypoint.latitude: " << waypoint.latitude << "\n"
           //     << "  waypoint.longitude: " << waypoint.longitude << std::endl; 


            //accessingWaypoint.setValue( true );

		    waypointLatitude = waypoint.latitude;
            waypointLongitude = waypoint.longitude;

            //mutexaccessingWaypoint.setValue( false );


            std::cout << "Pilot::waypointCallback\n"
                << "  waypointLatitude: " << waypointLatitude << "\n"
                << "  waypointLongitude: " << waypointLongitude << std::endl;          

		}

		void run(){
			ros::Rate loop_rate( 100 );

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
