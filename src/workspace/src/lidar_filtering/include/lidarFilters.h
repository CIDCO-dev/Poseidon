#ifndef lidarFilters
#define lidarFilters
#include <cmath>
#include "sensor_msgs/PointCloud2.h"
#include "sensor_msgs/point_cloud_conversion.h"
#include "sensor_msgs/PointCloud.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"


#define PI 3.14159265;

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
				if(! (horizontalAngleFilter(point, -135, -45) || distanceFilter(point, 1.0, 50.0))){
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
		
		// horizontal angle in degrees
		static bool horizontalAngleFilter(geometry_msgs::Point32 point, double minAngle, double maxAngle){
			// 90° is the heading
			// 0° is on left side
			double theta = atan2(point.y, point.x) * 180 / PI;
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
		static bool distanceFilter(geometry_msgs::Point32 point, double minDistance, double maxDistance){
			double distance = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));
			
			// discard point
			if(distance < minDistance || distance > maxDistance){
				return true;
			}
			else{
				return false;
			}
			
		}
		// elevation angle in degrees
		static bool azimutFilter(geometry_msgs::Point32 point, double minAngle, double maxAngle){
		
			double azimut = atan2(point.z, point.x) * 180 / PI;
			
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
