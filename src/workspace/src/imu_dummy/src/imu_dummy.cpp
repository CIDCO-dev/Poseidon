#include "imu_dummy/imu_dummy.h"
#include "../../utils/QuaternionUtils.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

void IMU::run(){
    sensor_msgs::Imu msg;

    msg.header.seq=++sequenceNumber;
    msg.header.stamp=ros::Time::now();

    double heading = sin((double)sequenceNumber/(double)M_PI)*30;
    double pitch   = cos((double)sequenceNumber/(double)M_PI)*20;
    double roll    = sin((double)sequenceNumber/(double)M_PI)*10;

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
