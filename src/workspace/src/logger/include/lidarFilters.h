#ifndef lidarFilters
#define lidarFilters
#include "types.h"
#include <cmath>

#define LPI 3.14159265; // local PI

class LidarFilter{
	
	public:
		// horizontal angle in degrees
		static bool horizontalAngleFilter(LidarPacket point, double minAngle, double maxAngle){
			// 90° is the heading
			// 0° is on left side
			double theta = atan2(point.laser_y, point.laser_x) * 180 / LPI;
			//std::cerr<<theta << " " << point.laser_x << " " << point.laser_y << " " << point.laser_z << std::endl;
			
			// discard point in that region
			if(theta > minAngle && theta < maxAngle ){
				
				return true;
			}
			else{
				return false;
			}
			
		}
		
		// eucledian distance in meters
		static bool distanceFilter(LidarPacket point, double minDistance, double maxDistance){
			double distance = sqrt(pow(point.laser_x, 2) + pow(point.laser_y, 2) + pow(point.laser_z, 2));
			
			// discard point
			if(distance < minDistance || distance > maxDistance){
				return true;
			}
			else{
				return false;
			}
			
		}
		// elevation angle in degrees
		static bool azimutFilter(LidarPacket point, double minAngle, double maxAngle){
		
			double azimut = atan2(point.laser_z, point.laser_x) * 180 / LPI;
			
			//std::cerr<< azimut << " " << point.laser_z << " " << point.laser_x << std::endl;
			
			if(azimut < minAngle || azimut > maxAngle){
				return true;
			}
			else{
				return false;
			}
		}
		
};

#endif
