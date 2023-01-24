#ifndef lidarFilters
#define lidarFilters
#include "ros/ros.h"
#include <cmath>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#define LPI 3.14159265; //local PI

class Filters{
	public:
		// horizontal angle in degrees
		static bool horizontalAngleFilter(double x, double y, double minAngle, double maxAngle){
			// 90° is the heading
			// 0° is on left side
			double theta = atan2(y, x) * 180 / LPI;
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
		static bool distanceFilter(double x, double y, double z, double minDistance, double maxDistance){
			double distance = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
			
			// discard point
			if(distance < minDistance || distance > maxDistance){
				return true;
			}
			else{
				return false;
			}
			
		}
		// elevation angle in degrees
		static bool azimutFilter(double x, double z, double minAngle, double maxAngle){
		
			double azimut = atan2(z, x) * 180 / LPI;
			
			//std::cerr<< azimut << " " << point.laser_z << " " << point.laser_x << std::endl;
			
			if(azimut < minAngle || azimut > maxAngle){
				return true;
			}
			else{
				return false;
			}
		}
		
};


class LidarFiltering{

	private:
		ros::NodeHandle node;
		ros::Subscriber lidarSubscriber ;
		ros::Publisher lidarFilteredPublisher ;
	
	public:
		LidarFiltering(){
			lidarSubscriber = node.subscribe("velodyne_points", 1000, &LidarFiltering::lidarCallback,this);
			lidarFilteredPublisher = node.advertise<sensor_msgs::PointCloud2>("filtered_lidar",1000);
		}
		
		void lidarCallback(const sensor_msgs::PointCloud2& lidar){
		
			sensor_msgs::PointCloud lidarXYZ;
			sensor_msgs::convertPointCloud2ToPointCloud(lidar, lidarXYZ);
			std::vector<geometry_msgs::Point32> points = lidarXYZ.points;
			std::vector<geometry_msgs::Point32> filteredPoints;
			
			for(auto const& point : points){
				if(! (Filters::horizontalAngleFilter(point.x, point.y, -135, -45) || Filters::distanceFilter(point.x, point.y, point.z, 1.0, 50.0))){
					filteredPoints.push_back(point);
					
				}
			}
			
			sensor_msgs::PointCloud lidarFiltered;
			sensor_msgs::PointCloud2 lidarFilteredMsgs;
			lidarFiltered.points = filteredPoints;
			sensor_msgs::convertPointCloudToPointCloud2(lidarFiltered, lidarFilteredMsgs);
			
			lidarFilteredMsgs.header = lidar.header;
			
			lidarFilteredPublisher.publish(lidarFilteredMsgs);
			
		}
};

#endif
