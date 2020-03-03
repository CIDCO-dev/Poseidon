#include "Goal.hpp"


class Waypoint : Goal 
{

public:

    Waypoint( const double latitude, const double longitude )
        : latitude( latitude ), longitude( longitude )
    {        
    }


/*
    Position getGoal() const override {
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

    const double latitude;
    const double longitude


};
