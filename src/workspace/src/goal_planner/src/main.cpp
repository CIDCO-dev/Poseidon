#ifndef MAIN_CPP
#define MAIN_CPP


#include <memory>

#include <iostream>
#include <iomanip>


#include <list>


#include "ros/ros.h"

#include "state_controller/State.h"

#include "goal_planner/Waypoint.h"

#include "../../utils/Goal.hpp"
#include "../../utils/Waypoint.hpp"
#include "../../utils/SVPprofile.hpp"

#include "../../utils/TwoDoublesRosTimeWithMutex.hpp"


// #include "raspberrypi_vitals/sysinfo.h"

// #include <iostream>
// #include <fstream>
// #include <string>

// #include "sys/statvfs.h"



using namespace std;

class GoalPlanner{

	private:
		
		// ROS handle
		ros::NodeHandle node;

		// Input topics
		ros::Subscriber stateTopic;

		// Output topics
		ros::Publisher waypointTopic;

        // TODO: use mutex?
		std::list< std::shared_ptr< Goal > > goals;


        // TODO: use mutex? (currentGoal would never be set from a callback)
        std::shared_ptr< Goal > currentGoal;


        // Add heading to currentPosition
        TwoDoublesRosTimeWithMutex currentPositionLatitudeLongitude;


		// Position currentPosition;

	public:
		GoalPlanner() 
            : currentPositionLatitudeLongitude( 0, 0, ros::Time( 0, 0 ) )    
        {

			// Advertise to topics: destination (to pilot)
			waypointTopic = node.advertise<goal_planner::Waypoint>("waypoint", 1000);

			// Subscribe to topics: state (from state_controller)
			stateTopic = node.subscribe("state", 1000, &GoalPlanner::stateCallback, this);


            // Reset currentGoal so it does not point to anything
            currentGoal.reset();

		}


		// Callback for state from state_controller
		void stateCallback( const state_controller::State& state ) {

			// Extract current position

            // Verify that this message is an update of the position,
            // not a message for the IMU, etc...

            // ? Validate state position's time vs ros::Time::now(); ?
            // state.position.header.stamp

            // Set current position from positon in the message
            currentPositionLatitudeLongitude.setValues( 
                   state.position.latitude, state.position.longitude,
                   state.position.header.stamp );


			// Extract other information, e.g. raspberrypi_vitals?

		}


        

		void run(){

			ros::Rate loop_rate( 10 );

            double distanceForWaypointReached = 10;

            // Populate the list for test purposes
            goals.push_back( std::make_shared< Waypoint > ( 12345, -12345, distanceForWaypointReached ) );
            goals.push_back( std::make_shared< SVPprofile > () );
            goals.push_back( std::make_shared< Waypoint > ( 123456789, -123456789, distanceForWaypointReached ) );


            for ( auto iter = goals.begin(); iter != goals.end(); ++iter ) {
            
                // Waypoint * ptr = dynamic_cast< Waypoint * >( *iter );

                std::shared_ptr< Waypoint > ptr = std::dynamic_pointer_cast<Waypoint>( *iter );

                if ( ptr )
                    std::cout << ptr-> getLatitude() << ", " 
                        << ptr-> getlongitude() << std::endl;
                else
                    std::cout << "Not a Waypoint" << std::endl;
            }


            // Waypoint message
            goal_planner::Waypoint waypointMessage;


            int count = 0;


            double currentPositionlatitude;
            double currentPositionlongitude;
            ros::Time currentPositionTime;




			while ( ros::ok() ){

/*
                // Create a waypoint message for test purposes
                std::cout << "GoalPlanner::run(), count: " << count << std::endl;

                waypointMessage.latitude = count;
                waypointMessage.longitude = -count;
                waypointMessage.pilotActive = 1;
                               
                waypointTopic.publish( waypointMessage );

                count++;
*/

				// Check state's raspberrypi_vitals?


				// If no current goal
                if ( currentGoal == nullptr )
				{
					// If there is a goal in the list
                    if ( goals.size() != 0 )
					{
						// Set this as the current goal,
                        currentGoal = goals.front();

						// Remove this goal from the list
                        goals.pop_front();
                        
						// Publish this goal
					}
				}


/*
				// If there is a current goal:
                if ( currentGoal != nullptr )
				{

                    // What if the current position is too old?

                    currentPositionLatitudeLongitude.getValues(
                                                currentPositionlatitude, 
                                                currentPositionlongitude, 
                                                currentPositionTime );

                    // If goal is reached
                    if ( currentGoal.execute( currentPositionlatitude, 
                                            currentPositionTime ) )
                    {
                        currentGoal = nullptr;

					// Publish a waypoint that will tell the motors to stop 
                    // (or maintain the position?)


                    }





					// If goal is a waypoint
                    if ( ptr )
					{


                        currentPositionLatitudeLongitude.getValues(
                                                    currentPositionlatitude, 
                                                    currentPositionlongitude, 
                                                    currentPositionTime );

                        // What if the current position is too old?


						// If currentPosition from the state_contoller 
						// is close enough to the currentWaypoint 
                        // (MBES-lib: src/math/Distance.hpp, function haversine)
						// {

								// The destination goal is reached

								// If the next goal in the list is a waypoint
								// {
										// Set this as the current goal, 
										// Remove this goal from the list
										// Publish this new waypoint
								// }
								// else 
								// {
										// Publish a waypoint that will tell the motors to stop 
                                        // (or maintain the position?)
                                        
                                        // Remove the waypoint from the current goal
                                        // currentGoal.reset();
								// }


						// }

					}
					else 
					{
					    // Deal with other goal types
					}



				}


		*/

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
