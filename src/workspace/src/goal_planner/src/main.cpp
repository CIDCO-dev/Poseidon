#ifndef MAIN_CPP
#define MAIN_CPP


#include <memory>


#include <list>


#include "ros/ros.h"

#include "state_controller/State.h"

#include "goal_planner/Waypoint.h"

#include "../../utils/Goal.hpp"
#include "../../utils/Waypoint.hpp"
#include "../../utils/SVPprofile.hpp"




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


		std::list< std::shared_ptr< Goal > > goals;


        // TODO: use mutex?
        std::shared_ptr< Goal > currentGoal;


		// Position currentPosition;

	public:
		GoalPlanner(){

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

			// Extract other information, e.g. raspberrypi_vitals?

		}


        

		void run(){

			ros::Rate loop_rate( 10 );

            // Populate the list for test purposes
            goals.push_back( std::make_shared< Waypoint > ( 12345, -12345 ) );
            goals.push_back( std::make_shared< SVPprofile > () );
            goals.push_back( std::make_shared< Waypoint > ( 123456789, -123456789 ) );


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

			while ( ros::ok() ){

                // Create a waypoint message for test purposes
                std::cout << "GoalPlanner::run(), count: " << count << std::endl;

                waypointMessage.latitude = count;
                waypointMessage.longitude = -count;
                waypointMessage.pilotActive = 1;
                               
                waypointTopic.publish( waypointMessage );

                count++;


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



				// If there is a current goal:
                if ( currentGoal != nullptr )
				{

                    std::shared_ptr< Waypoint > ptr = std::dynamic_pointer_cast<Waypoint>( currentGoal );

					// If goal is a waypoint
                    if ( ptr )
					{


                        // What if the current position is too old?


						// If currentPosition from the state_contoller 
						// is close enough to the currentWaypoint (MBES-lib: src/math/Distance.hpp, function haversine)
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
										// Publish a waypoint that will tell the motors to stop (or maintain the position?)
                                        
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
