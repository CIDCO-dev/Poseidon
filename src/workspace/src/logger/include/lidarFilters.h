#ifndef lidarFilters
#define lidarFilters
#include "types.h"
#include <cmath>

#define LPI 3.14159265; // local PI

class LidarFilter{
	
	public:
		static bool keepPoint(LidarPacket point, double minAngle, double maxAngle, double minDistance, double maxDistance){
			
			double theta = atan2(point.laser_y, point.laser_x) * 180 / LPI;
			double distance = sqrt(pow(point.laser_x, 2) + pow(point.laser_y, 2) + pow(point.laser_z, 2));
			
			
			if(theta < minAngle || theta > maxAngle || distance < minDistance || distance > maxDistance){
				//std::cerr<<theta << " " << distance << " " << point.laser_x << " " << point.laser_y << " " << point.laser_z << std::endl;
				return true;
			}
			else{
				return false;
			}
			
		}
		
		static bool badPoint(LidarPacket point, double minAngle, double maxAngle, double minDistance, double maxDistance){
		
			double theta = atan2(point.laser_y, point.laser_x) * 180 / LPI;
			double distance = sqrt(pow(point.laser_x, 2) + pow(point.laser_y, 2) + pow(point.laser_z, 2));
			
			if( (theta > minAngle && theta < maxAngle)  || (distance < minDistance || distance > maxDistance)){
				return true;
			}
			else{
				return false;
			}
		}
		
		
		static bool distanceFilter(LidarPacket point, double minDistance, double maxDistance){
			double distance = sqrt(pow(point.laser_x, 2) + pow(point.laser_y, 2) + pow(point.laser_z, 2));
			
			if(distance > 15.0){
				std::cerr<< distance << " " << point.laser_x << " " << point.laser_y << " " << point.laser_z << std::endl;
			}
			
			if(distance < minDistance || distance > maxDistance){
				return false;
			}
			else{
				return true;
			}
			
		}
		
		static bool azimutFilter(LidarPacket point, double minAngle, double maxAngle){
		
			double azimut = atan2(point.laser_z, point.laser_x) * 180 / LPI;
			
			//std::cerr<< azimut << " " << point.laser_z << " " << point.laser_x << std::endl;
			
			if(azimut < minAngle || azimut > maxAngle){
				return false;
			}
			else{
				return true;
			}
		}
		
};

#endif
