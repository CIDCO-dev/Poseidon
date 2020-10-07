
#ifndef HAVERSINE_HPP
#define HAVERSINE_HPP


#include <math.h>

#include "Constants.hpp"


    /**
     * Gives the distance in meters between two points on a spheroid 
     * @param longitude1 Point1's lon
<node pkg="state_controller" name="state_controller" type="state_controller" output="screen"/><node pkg="state_controller" name="state_controller" type="state_controller" output="screen"/gitude
     * @param latitude1 Point1's latitude
     * @param longitude2 Point2's longitude
     * @param latitude2 Point2's latitude
     * @return the distance in meters
     */


double haversine(double longitude1, double latitude1, double longitude2, double latitude2){

// From MBES-lib
/*
	double dx, dy, dz;
	latitude1 -= latitude2;
	latitude1 *= D2R, longitude1 *= D2R, longitude2 *= D2R;
 
	dz = sin(longitude1) - sin(longitude2);
	dx = cos(latitude1) * cos(longitude1) - cos(longitude2);
	dy = sin(latitude1) * cos(longitude1);
	return asin(sqrt(dx * dx + dy * dy + dz * dz) / 2) * 2 * 6371000;   

*/


    longitude1 *= D2R(1);
    latitude1 *= D2R(1);
    longitude2 *= D2R(1);
    latitude2 *= D2R(1);


    // https://en.wikipedia.org/wiki/Haversine_formula

    double sin2LatDiffOver2 = pow( sin( ( latitude2 - latitude1 ) / 2 ), 2 );
    double sin2LongDiffOver2 = pow( sin( ( longitude2 - longitude1 ) / 2 ), 2 );

    double insideSqrt = sin2LatDiffOver2 + cos( latitude1 ) * cos( latitude2 )
                            * sin2LongDiffOver2;

    return 2 * 6371000 * asin( sqrt( insideSqrt ) );

 
}


#endif
