#include "imu_null/imu_null.h"
#include "../../utils/QuaternionUtils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void IMU::run(){
    sensor_msgs::Imu msg;

    msg.header.seq=++sequenceNumber;
    msg.header.stamp=ros::Time::now();

    double heading = 0;
    double pitch   = 0;
    double roll    = 0;

    tf2::Quaternion q;
    q.setRPY(D2R(roll),D2R(pitch),D2R(heading));
    msg.orientation = tf2::toMsg(q);

    imuTopic.publish(msg);
}

void IMU::message(uint32_t sequenceNumber,double yaw, double pitch, double roll){
    sensor_msgs::Imu msg;
    msg.header.seq=sequenceNumber;
    msg.header.stamp=ros::Time::now();

    tf2::Quaternion q;
    q.setRPY(D2R(roll),D2R(pitch),D2R(yaw));
    msg.orientation = tf2::toMsg(q);

    imuTopic.publish(msg);
}
