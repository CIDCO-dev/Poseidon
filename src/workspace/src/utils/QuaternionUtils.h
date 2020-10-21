#ifndef QUATERNIONUTILS_H
#define QUATERNIONUTILS_H

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>


#define TF2_EULER_DEFAULT_ZYX 1

#include "Constants.hpp"

class QuaternionUtils{
public:

	static void applyTransform(geometry_msgs::Quaternion transform,geometry_msgs::Quaternion pose, double & headingDegrees,double & pitchDegrees,double & rollDegrees){
                tf2::Quaternion rotation, tfImuPose;
                tf2::fromMsg(transform, rotation);
                tf2::fromMsg(pose, tfImuPose);
                tf2::Quaternion tfBodyPose = rotation * tfImuPose;
                tfBodyPose.normalize();

		tf2::Matrix3x3 mat(tfBodyPose);
		mat.getEulerYPR(headingDegrees,pitchDegrees,rollDegrees);


                //geometry_msgs::Quaternion bodyPose = tf2::toMsg(tfBodyPose);

		

        	//       QuaternionUtils::convertToEulerAngles(bodyPose,headingDegrees,pitchDegrees,rollDegrees);
	}

	/* Returns angles in DEGREES */
	static void convertToEulerAngles(const geometry_msgs::Quaternion & q,double & heading,double & pitch, double & roll){
	

		// roll (x-axis rotation)
		double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
		double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
		roll = R2D( std::atan2(sinr_cosp, cosr_cosp) );

		// pitch (y-axis rotation)
		double sinp = 2 * (q.w * q.y - q.z * q.x);

		if (std::abs(sinp) >= 1){
			pitch = R2D( std::copysign(M_PI / 2, sinp) ); // use 90 degrees if out of range
		}
		else{
			pitch = R2D( std::asin(sinp) );
		}

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
		heading = R2D( std::atan2(siny_cosp, cosy_cosp) );


		
	}
};

#endif
