
#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP



#include "Goal.hpp"


#include "haversine.hpp"


class Waypoint : public Goal 
{

public:

    Waypoint( const double latitude, const double longitude, 
                const double distanceForWaypointReached )
        : latitude( latitude ), longitude( longitude ),
            distanceForWaypointReached( distanceForWaypointReached )
    {        
    }

    virtual ~Waypoint() {}

/*
    Waypoint getGoal() const override {
        Waypoint waypoint( latitude, longitude );
        return waypoint;
    }
*/


    bool execute( const double currentLatitude, 
                    const double currentLongitude ) {

		// If currentPosition from the state_contoller 
		// is close enough to the currentWaypoint 
        // (MBES-lib: src/math/Distance.hpp, function haversine)
		if ( haversine( longitude, latitude, 
                        currentLongitude, currentLatitude ) 
                <= distanceForWaypointReached )
        {
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
    virtual void junkFunctionForPureVirtual() override {} 

    const double latitude;
    const double longitude;

    const double distanceForWaypointReached;


};

#endif
