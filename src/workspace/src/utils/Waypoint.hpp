#include "Goal.hpp"

#include "Structs.hpp"


class Waypoint : Goal 
{

public:

    Waypoint( Position & position )
        : waypoint( position )
    {        
    }


    Position getGoal() const override {
        return waypoint;
    }


private:

    Position waypoint;


};