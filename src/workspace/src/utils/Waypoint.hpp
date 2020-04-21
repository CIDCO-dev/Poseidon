
#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP

#include "ros/ros.h"



#include "Goal.hpp"


#include "haversine.hpp"


class Waypoint : public Goal 
{

public:

    Waypoint( const double latitude, const double longitude, 
                const double distanceForWaypointReached, 
                ros::Publisher & waypointTopic ) 
        : latitude( latitude ), longitude( longitude ),
            distanceForWaypointReached( distanceForWaypointReached ),
            waypointTopic( waypointTopic )
    {        
    }

    virtual ~Waypoint() {}


    virtual void start() override {

        goal_planner_msg::Waypoint waypointMessage;

        waypointMessage.latitude = latitude;
        waypointMessage.longitude = longitude;
        waypointMessage.pilotActive = 1;
                       
        waypointTopic.publish( waypointMessage );

        std::cout << "\nWaypoint::start(), after publish\n" << std::endl;  

    }


    // Returned value:  true if goal is met
    //                  false if goal is not yet met
    virtual bool execute( const double currentLatitude, 
                    const double currentLongitude ) override {

		// If currentPosition from the state_contoller 
		// is close enough to the currentWaypoint 
        // (MBES-lib: src/math/Distance.hpp, function haversine)


        double distance = haversine( longitude, latitude, 
                        currentLongitude, currentLatitude );

/*        std::cout << std::setprecision(10) << std::fixed
            << "Waypoint::execute(), waypoint lat-long: " 
            << latitude << " " << longitude << ", current lat-long "
            << currentLatitude << " " << currentLongitude << " distance "
            << distance << std::endl;
*/

		if ( distance <= distanceForWaypointReached )
        {
            
            goal_planner_msg::Waypoint waypointMessage;

            waypointMessage.latitude = latitude;
            waypointMessage.longitude = longitude;
            waypointMessage.pilotActive = 0;
                           
            waypointTopic.publish( waypointMessage );

            return true;
        } else {
            return false;
        }
    }

    double getLatitude() const {
        return latitude;
    }

    double getlongitude() const {
        return longitude;
    }


private:

    const double latitude;
    const double longitude;

    const double distanceForWaypointReached;

    ros::Publisher & waypointTopic;

};

#endif
