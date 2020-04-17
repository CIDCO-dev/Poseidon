#ifndef the_pilot
#define the_pilot

#include <iostream>

#include "ros/ros.h"

#include "state_controller/State.h"

#include "catarob_control/motor.h"

#include "goal_planner/Waypoint.h"

// #include "../../utils/Structs.hpp"

//#include "../../utils/Waypoint.hpp"


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

        TwoDoublesRosTimeWithMutex waypointLatitudeLongitude;

        // Add heading to currentPosition
        TwoDoublesRosTimeWithMutex currentPositionLatitudeLongitude;

        BoolWithMutex pilotActive;
    
        // Max speed?
        // max motor drive?

        uint32_t secMaxFromNowToCurrentPosition = 20;


	public:
		Pilot()
         :  
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


            std::cout << "\n\nwaypointLatitudeLongitude.isTimeZero(): " 
                << std::boolalpha << waypointLatitudeLongitude.isTimeZero()
                << std::noboolalpha << "\n" << std::endl;

		}


		// Callback for state from state_controller
		void stateCallback( const state_controller::State& state ) {


            // Verify that this message is an update of the position,
            // not a message for the IMU, etc...

            // ? Validate state position's time vs ros::Time::now(); ?
            // state.position.header.stamp


            // Set current position from positon in the message

            currentPositionLatitudeLongitude.setValues( 
                   state.position.latitude, state.position.longitude,
                   state.position.header.stamp );

		}

		// Callback for waypoint
		void waypointCallback( const goal_planner::Waypoint& waypointIn ) {

           // std::cout << "Pilot::waypointCallback\n"
           //    << "  waypoint.latitude: " << waypoint.latitude << "\n"
           //     << "  waypoint.longitude: " << waypoint.longitude << std::endl; 

            if ( waypointIn.pilotActive != 0 ) {

                waypointLatitudeLongitude.setValues( 
                    waypointIn.latitude, waypointIn.longitude,
                    ros::Time::now() );

                pilotActive.setValue( true );

            } else {
                pilotActive.setValue( false );

                waypointLatitudeLongitude.setValues( 0, 0, ros::Time( 0, 0 ) );

                // ? Message for motors to stop
            }


            // For display
            double latitude;
            double longitude;

            ros::Time timeToDisplay;

            waypointLatitudeLongitude.getValues( latitude, longitude, timeToDisplay );

            std::cout << std::setprecision(10) << std::fixed
                << "Pilot::waypointCallback\n"
                << "  timeToDisplay sec.nsec: " << timeToDisplay.sec << "."
                << timeToDisplay.nsec << "\n"
                << "  latitude: " << latitude << "\n"
                << "  longitude: " << longitude  << "\n"
                << "  pilotActive.getValue(): " 
                << std::boolalpha << pilotActive.getValue() << std::noboolalpha 
                << "\n" << std::endl;     

		}

        

		void run(){
			ros::Rate loop_rate( 100 );

            		ros::Time currentPositionTime;
            		double currentPositionlatitude;
            		double currentPositionlongitude;

			while ( ros::ok() ) {

				// If there is a currentWaypoint
                if ( pilotActive.getValue() ) { 

                    currentPositionLatitudeLongitude.getValues(
                            currentPositionlatitude, 
                            currentPositionlongitude, 
                            currentPositionTime );

                    // If the currentPosition is recent enough
                    if ( ros::Time::now().sec - currentPositionTime.sec 
                        <= secMaxFromNowToCurrentPosition ) {

					    // Based on the currentPosition, 
                        // control the motors to get to the currentWaypoint

                    } else {     // currentPosition is too old
                        // What to do?

                    }


				}


				ros::spinOnce();
				loop_rate.sleep();

			}

		}
};


#endif
