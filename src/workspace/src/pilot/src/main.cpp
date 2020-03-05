#ifndef MAIN_CPP
#define MAIN_CPP

#include <iostream>

#include "ros/ros.h"

#include "state_controller/State.h"

#include "catarob_control/motor.h"

#include "goal_planner/Waypoint.h"

// #include "../../utils/Structs.hpp"

//#include "../../utils/Waypoint.hpp"

//#include "../../utils/TwoDoublesWithMutex.hpp"
#include "../../utils/TwoDoublesRosTimeWithMutex.hpp"


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


		//DoubleWithMutex waypointLatitude;
        //DoubleWithMutex waypointLongitude;

		//DoubleWithMutex currentPositionLatitude;
        //DoubleWithMutex currentPositionLongitude;

        //ros::Time waypointReceivedTime;
        //TwoDoublesWithMutex waypointLatitudeLongitude;

        //ros::Time currentPositionTime;
        //TwoDoublesWithMutex currentPositionLatitudeLongitude;


        TwoDoublesRosTimeWithMutex waypointLatitudeLongitude;
        TwoDoublesRosTimeWithMutex currentPositionLatitudeLongitude;


        BoolWithMutex pilotActive;
    


	public:
		Pilot()
         :  
            //waypointReceivedTime.sec( 0 ),
            //waypointReceivedTime.nsec( 0 ),
            //waypointLatitudeLongitude( 0, 0 ),

            //currentPositionTime.sec( 0 ),
            //currentPositionTime.nsec( 0 ),
            //currentPositionLatitudeLongitude( 0, 0 ),

            waypointLatitudeLongitude( 0, 0, ros::Time( 0, 0 ) ),
            currentPositionLatitudeLongitude( 0, 0, ros::Time( 0, 0 ) ),

            pilotActive( false )
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


            // Verify that this message is an update of the position,
            // not a message for the IMU, etc...

            // ? Validate state position's time vs ros::Time::now(); ?
            // state.position.header.stamp


            // the time of the current position in the message must also be kept

            // Set current position from positon in the message

            //currentPositionLatitudeLongitude.setValues( 
            //       state.position.latitude, state.position.longitude );

            // currentPositionTime = state.position.header.stamp


            currentPositionLatitudeLongitude.setValues( 
                   state.position.latitude, state.position.longitude,
                   state.position.header.stamp );



		}

		// Callback for waypoint
		void waypointCallback( const goal_planner::Waypoint& waypointIn ) {

           // std::cout << "Pilot::waypointCallback\n"
           //    << "  waypoint.latitude: " << waypoint.latitude << "\n"
           //     << "  waypoint.longitude: " << waypoint.longitude << std::endl; 

            
            
            // waypointLatitudeLongitude.setValues( 
            //    waypointIn.latitude, waypointIn.longitude );

            // waypointReceivedTime = ros::Time::now();


            waypointLatitudeLongitude.setValues( 
                waypointIn.latitude, waypointIn.longitude,
                ros::Time::now() );


            // For display
            double latitude;
            double longitude;

            ros::Time timeToDisplay;

            waypointLatitudeLongitude.getValues( latitude, longitude, timeToDisplay );

            std::cout << "Pilot::waypointCallback\n"
                << "  timeToDisplay sec.nsec: " << timeToDisplay.sec << "."
                << timeToDisplay.nsec << "\n"
                << "  latitude: " << latitude << "\n"
                << "  longitude: " << longitude << std::endl;     

		}

		void run(){
			ros::Rate loop_rate( 100 );

			while ( ros::ok() ){


				// If there is a currentWaypoint 
				//{ 

                        // If the currentPosition is recent enough
                        // {

						    // Based on the currentPosition, control the motors to get to the currentWaypoint

                        // }
                        // else     // currentPosition is too old
                        // {
                                // What to do?

                        // }


				// }


                // What to do if the currentPosition is too old?

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
