
#ifndef WAYPOINT_HPP
#define WAYPOINT_HPP



#include "Goal.hpp"


class Waypoint : public Goal 
{

public:

    Waypoint( const double latitude, const double longitude )
        : latitude( latitude ), longitude( longitude )
    {        
    }

    virtual ~Waypoint() {}

/*
    Waypoint getGoal() const override {
        Waypoint waypoint( latitude, longitude );
        return waypoint;
    }
*/

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


};

#endif
