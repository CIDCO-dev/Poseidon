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
		rotation.normalize();
		tfImuPose.normalize();
                tf2::Quaternion tfBodyPose = rotation * tfImuPose;
                tfBodyPose.normalize();

		tf2::Matrix3x3 mat(tfBodyPose);


		mat.getEulerYPR(headingDegrees,pitchDegrees,rollDegrees);

		headingDegrees = 360-R2D(headingDegrees);
		pitchDegrees   = R2D(pitchDegrees);
		rollDegrees    = R2D(rollDegrees);
	}

};

#endif
