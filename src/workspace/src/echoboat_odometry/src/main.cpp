#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <Eigen/Dense>

//TODO: this might be better off in a util class / module

#define D2R(x) ((M_PI*x)/180.0)

void geo2ecef(Eigen::Vector3d & positionECEF, double latitude, double longitude, double ellipsoidalHeight) {
	/**WGS84 ellipsoid semi-major axis*/
	double a = 6378137.0;

	/**WGS84 ellipsoid first eccentricity squared*/
	double e2 = 0.081819190842622 * 0.081819190842622;

	double slat = sin(D2R(latitude));
	double clat = cos(D2R(latitude));
	double slon = sin(D2R(longitude));
	double clon = cos(D2R(longitude));

	double N = a / (sqrt(1 - e2 * slat * slat));
	double xTRF = (N + ellipsoidalHeight) * clat * clon;
	double yTRF = (N + ellipsoidalHeight) * clat * slon;
	double zTRF = (N * (1 - e2) + ellipsoidalHeight) * slat;

	positionECEF << xTRF, yTRF, zTRF;
};

class GeoreferenceProvider{
public:
	GeoreferenceProvider(): ecefLocalOrigin(0,0,0){
	
	odomPublisher=n.advertise<nav_msgs::Odometry>("odom", 50);

	depthRepublisher = n.advertise<geometry_msgs::PointStamped>("depth_ned",50);

	gpsSubscriber = n.subscribe("fix", 50, &GeoreferenceProvider::processGpsCallback,this);
	//attitudeSubscriber = n.subscribe("/imu/pos_ecef", 50, &GeoreferenceProvider::processPositionCallback,this);
	attitudeSubscriber = n.subscribe("/imu/data", 50, &GeoreferenceProvider::processAttitudeCallback,this);
	lidarSubscriber = n.subscribe("/velodyne_points",100,&GeoreferenceProvider::processLidarCallback,this);
	depthSubscriber = n.subscribe("/depth",100,&GeoreferenceProvider::processDepthCallback,this);

	}

	void processGpsCallback(const sensor_msgs::NavSatFix & msg){

		//Init ENU frame origin once we receive a first fix
		if(!gpsFixObtained && msg.status.status >= 0){
			gpsFixObtained = true;
			latitudeLocalOrigin = msg.latitude;
			longitudeLocalOrigin = msg.longitude;
			altitudeLocalOrigin = msg.altitude;

			ROS_INFO("GPS Fix obtained. Initializing NED transform at (%f,%f,%f)",latitudeLocalOrigin,longitudeLocalOrigin,altitudeLocalOrigin);

			geo2ecef(ecefLocalOrigin,latitudeLocalOrigin,longitudeLocalOrigin,altitudeLocalOrigin);

			ROS_INFO_STREAM("Local Origin:" << ecefLocalOrigin);

			//ENU
			//ecef2local << 	-sin(D2R(longitudeLocalOrigin)),	cos(D2R(longitudeLocalOrigin)),	0,
			//		-sin(D2R(latitudeLocalOrigin))*cos(D2R(longitudeLocalOrigin)), -sin(D2R(latitudeLocalOrigin))*sin(D2R(longitudeLocalOrigin)), cos(D2R(latitudeLocalOrigin)),
			//		cos(D2R(latitudeLocalOrigin))*cos(D2R(longitudeLocalOrigin)),cos(D2R(latitudeLocalOrigin))*sin(D2R(longitudeLocalOrigin)),sin(D2R(latitudeLocalOrigin));

			//NED
			ecef2local <<-sin(D2R(latitudeLocalOrigin))*cos(D2R(longitudeLocalOrigin)),-sin(D2R(latitudeLocalOrigin))*sin(D2R(longitudeLocalOrigin)),cos(D2R(latitudeLocalOrigin)),
				   			-sin(D2R(longitudeLocalOrigin)) , cos(D2R(longitudeLocalOrigin)), 0,
							-cos(D2R(latitudeLocalOrigin))*cos(D2R(longitudeLocalOrigin)), -cos(D2R(latitudeLocalOrigin))*sin(D2R(longitudeLocalOrigin)), -sin(D2R(latitudeLocalOrigin)) ;

			//ROS_INFO_STREAM("DCM:" << ecef2local);
		}
		else if(gpsFixObtained){
			processPosition(msg);
		}
	}

	void processAttitudeCallback(const sensor_msgs::Imu & msg){
		lastAttitude = msg;
	}

	void processLidarCallback(const sensor_msgs::PointCloud2 & msg){
		//std::cout << "Got lidar data" << std::endl;
	}

	void processDepthCallback(const geometry_msgs::PointStamped & msg){
		//FIXME: this is solved in the imagenex driver but not in the rosbag so we need to republish to NED frame
		geometry_msgs::PointStamped newMsg = msg;
		newMsg.point.z=-1 * msg.point.z;
		depthRepublisher.publish(newMsg);		
	}

	/* Publish the odometry transform and Odometry message */
	void publishOdometry(Eigen::Vector3d & prpPointLocal){

		/* Broadcast the base_link to map transform */
        geometry_msgs::TransformStamped map_transform;
        map_transform.header.stamp = ros::Time::now(); //msg.header.stamp;
        map_transform.header.frame_id = "map"; //FIXME: we are assuming that the prp is the base_link origin
        map_transform.child_frame_id = "base_link";

        map_transform.transform.translation.x = prpPointLocal[0];
        map_transform.transform.translation.y = prpPointLocal[1];
        map_transform.transform.translation.z = prpPointLocal[2];
        map_transform.transform.rotation = lastAttitude.orientation;

        transformBroadcaster.sendTransform(map_transform);

        /* Publish the odometry message over ROS */
        nav_msgs::Odometry odom;
        odom.header.stamp = ros::Time::now();//msg.header.stamp;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = prpPointLocal[0];
        odom.pose.pose.position.y = prpPointLocal[1];
        odom.pose.pose.position.z = prpPointLocal[2];

		//TODO: compensate attitude value using delta t, since our last attitude value dates from a little while back
        odom.pose.pose.orientation = lastAttitude.orientation;

        //set the velocity
        odom.child_frame_id = "base_link";
        //odom.twist.twist.linear.x = vx;
        //odom.twist.twist.linear.y = vy;
        //odom.twist.twist.angular.z = vth;

        //publish the message
        odomPublisher.publish(odom);
	}

	void processPosition(const sensor_msgs::NavSatFix & msg){
		if(gpsFixObtained){
			lastPosition = msg;

			//transform to ENU
			Eigen::Vector3d pointEcef;

			geo2ecef(pointEcef,msg.latitude,msg.longitude,msg.altitude);

			Eigen::Vector3d pointLocal = ecef2local * (pointEcef - ecefLocalOrigin);

			//Update odometry
			publishOdometry(pointLocal);
		}
	}



	void run(){
		ros::Rate r(10.0);

		//just spin...
		while(n.ok()){
			ros::spinOnce();
			r.sleep();
		}
	}

private:
	ros::NodeHandle n;
	ros::Publisher  odomPublisher;

	ros::Publisher  depthRepublisher;//FIXME: delete this

	ros::Subscriber gpsSubscriber;

	ros::Subscriber positionSubscriber;
	ros::Subscriber attitudeSubscriber;
	ros::Subscriber lidarSubscriber;
	ros::Subscriber depthSubscriber;



	tf2_ros::TransformBroadcaster transformBroadcaster;

	//These are used to georeference
	sensor_msgs::Imu 	lastAttitude;
	sensor_msgs::NavSatFix  lastPosition;

	bool gpsFixObtained = false;

	//This DCM is used to transform ECEF coordinates into a local NED frame centered at ecefEnuOrigin
	Eigen::Matrix3d	ecef2local;

	//The NED origin is set as the first position when a proper GPS fix is obtained
	Eigen::Vector3d ecefLocalOrigin;

	//NED origin in geodesic coordinates
	double latitudeLocalOrigin;
	double longitudeLocalOrigin;
	double altitudeLocalOrigin;
};

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  GeoreferenceProvider georefProvider;

  georefProvider.run();
}
