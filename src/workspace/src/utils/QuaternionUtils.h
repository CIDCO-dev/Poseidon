#ifndef QUATERNIONUTILS_H
#define QUATERNIONUTILS_H

#define R2D(x) (x * ((double)180/(double)M_PI))

class QuaternionUtils{
public:

    static void convertToQuaternion(double yaw, double pitch, double roll, nav_msgs::Odometry& odom){
        // Abbreviations for the various angular functions
        double cy = cos(yaw * 0.5);
        double sy = sin(yaw * 0.5);
        double cp = cos(pitch * 0.5);
        double sp = sin(pitch * 0.5);
        double cr = cos(roll * 0.5);
        double sr = sin(roll * 0.5);

        odom.pose.pose.orientation.w = cy * cp * cr + sy * sp * sr;
        odom.pose.pose.orientation.x = cy * cp * sr - sy * sp * cr;
        odom.pose.pose.orientation.y = sy * cp * sr + cy * sp * cr;
        odom.pose.pose.orientation.z = sy * cp * cr - cy * sp * sr;
    }

	static void convertToEulerAngles(const geometry_msgs::Quaternion & q,double & heading,double & pitch, double & roll){
		// roll (x-axis rotation)
		double sinr_cosp = 2 * (q.w * q.x + q.y * q.z);
		double cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y);
		roll = std::atan2(sinr_cosp, cosr_cosp);

		// pitch (y-axis rotation)
		double sinp = 2 * (q.w * q.y - q.z * q.x);

		if (std::abs(sinp) >= 1){
			pitch = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
		}
		else{
			pitch = std::asin(sinp);
		}

		// yaw (z-axis rotation)
		double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
		double cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
		heading = std::atan2(siny_cosp, cosy_cosp);
	}
};

#endif
