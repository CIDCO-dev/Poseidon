#ifndef lidarFilters
#define lidarFilters
#include <cmath>

#define PI 3.14159265;

class LidarFiltering{

	private:
		ros::NodeHandle node;
		ros::Subscriber lidarSubscriber ;
		ros::Publisher lidarFilteredPublisher ;
	
	public:
		LidarFiltering(){
			lidarSubscriber = n.subscribe("velodyne_points", 1000, &LidarFiltering::lidarCallback,this);
			lidarFilteredPublisher = n.advertise<geometry_msgs::Point32>("filtered_lidar",1000);
		}
		
		void lidarCallback(const sensor_msgs::PointCloud2& lidar){
		
			sensor_msgs::PointCloud lidarXYZ;
			sensor_msgs::convertPointCloud2ToPointCloud(lidar, lidarXYZ);
			std::vector<geometry_msgs::Point32> points = lidarXYZ.points;
			std::vector<geometry_msgs::Point32> filteredPoints;
			
			for(auto const& point : points){
				if(!horizontalAngleFilter(point, 45, 135)){
					filteredPoints.push_back(point);
				}
			}
			
			
			
		}
		
		// horizontal angle in degrees
		static bool horizontalAngleFilter(geometry_msgs::Point32 point, double minAngle, double maxAngle){
			// 90° is the heading
			// 0° is on left side
			double theta = atan2(point.y, point.x) * 180 / LPI;
			//std::cerr<<theta << " " << point.laser_x << " " << point.laser_y << " " << point.laser_z << std::endl;
			
			// discard point in that region
			if(theta > minAngle && theta < maxAngle ){
				
				return true;
			}
			else{
				return false;
			}
			
		}
		/*
		// eucledian distance in meters
		static bool distanceFilter(geometry_msgs::Point32 point, double minDistance, double maxDistance){
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
		static bool azimutFilter(geometry_msgs::Point32 point, double minAngle, double maxAngle){
		
			double azimut = atan2(point.laser_z, point.laser_x) * 180 / LPI;
			
			//std::cerr<< azimut << " " << point.laser_z << " " << point.laser_x << std::endl;
			
			if(azimut < minAngle || azimut > maxAngle){
				return true;
			}
			else{
				return false;
			}
		}
		*/
		
};

#endif
