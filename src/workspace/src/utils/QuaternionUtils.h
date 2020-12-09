#ifndef QUATERNIONUTILS_H
#define QUATERNIONUTILS_H

#define TF2_EULER_DEFAULT_ZYX 1

#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "Constants.hpp"

class QuaternionUtils{
public:

	static void applyTransform(geometry_msgs::Quaternion transform,geometry_msgs::Quaternion pose, double & headingDegrees,double & pitchDegrees,double & rollDegrees){

        tf2::Quaternion tfBodyPose;
        body2Enu(transform, pose, tfBodyPose);
        tfBodyPose.normalize();

        quaternion2YPR(tfBodyPose,headingDegrees,pitchDegrees,rollDegrees);
        headingDegrees = flipHeading(headingDegrees);
	}

protected:

    // make heading left handed like a compass
    static double flipHeading(double headingDegrees) {
        if(headingDegrees > 0 && headingDegrees < 360) {
            return 360-headingDegrees;
        } else if(headingDegrees < 0 || headingDegrees > 360) {
            // angle is inside [0, 360[
            return 360-modulo360(headingDegrees);
        }

        // angle is 0
	    return 0;
	}

	static double modulo360(double angleDegrees) {
	    // the nested fmod will convert any angle to ]-360,360[
	    // Adding 360 and second fmod will convert the result to [0, 360[
	    return std::fmod(std::fmod(angleDegrees, 360)+360, 360);
	}

    static void quaternion2YPR(tf2::Quaternion q, double & headingDegrees,double & pitchDegrees,double & rollDegrees) {
        tf2::Matrix3x3 mat(q);
		mat.getEulerYPR(headingDegrees,pitchDegrees,rollDegrees);

		headingDegrees = R2D(headingDegrees);
		pitchDegrees   = R2D(pitchDegrees);
		rollDegrees    = R2D(rollDegrees);
    }

    static void body2Enu(geometry_msgs::Quaternion & transform,geometry_msgs::Quaternion & pose, tf2::Quaternion & result) {
        tf2::Quaternion rotation; // the body's rotation wrt to IMU coordinated in IMU frame
        tf2::fromMsg(transform, rotation);
        rotation.normalize();

        tf2::Quaternion tfImuPose; // the IMU's pose
        tf2::fromMsg(pose, tfImuPose);
		tfImuPose.normalize();

        result = tfImuPose * rotation.inverse();
        result.normalize();
    }


};

#endif
